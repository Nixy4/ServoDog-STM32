#include "PCA9685.h"

#define PCA_ADDR 		 0x80

#define REG_MODE 0x00
#define REG_PSC  0xFE

#define REG_LED0_BASE     0x06
#define REG_REG_LED0_ON_L 0x06

#define REG_LEDX_BASE(x)  (0x06+4*x)
#define REG_LEDX_ON_L(x)  (0x06+4*x)
#define REG_LEDX_ON_H(x)  (0x07+4*x)
#define REG_LEDX_OFF_L(x) (0x08+4*x)
#define REG_LEDX_OFF_H(x) (0x09+4*x)

#define HAL_I2C_TRANSFER_TIMEOUT 100

static void PCA9685_Write(uint8_t reg, uint8_t data)
{
	uint8_t buf[2] = {reg, data};
	HAL_I2C_Master_Transmit(&hi2c2,PCA_ADDR,buf,2, HAL_I2C_TRANSFER_TIMEOUT);
}
 
static uint8_t PCA9685_Read(uint8_t reg)
{
	uint8_t val = 0;
	HAL_I2C_Master_Transmit(&hi2c2, PCA_ADDR, &reg, 1, HAL_I2C_TRANSFER_TIMEOUT);	
	HAL_I2C_Master_Receive (&hi2c2, PCA_ADDR, &val, 1, HAL_I2C_TRANSFER_TIMEOUT); 
	return val;
}

void PCA9685_setPWM(uint8_t ledx, uint32_t on, uint32_t off)
{
	uint8_t buf[5] = {
		REG_LEDX_BASE(ledx), 
		on&0xFF, 
		on>>8, 
		off&0xFF, 
		off>>8
		};
	HAL_I2C_Master_Transmit(&hi2c2,PCA_ADDR,buf,5,100);
}

//����pwm���Ƶ��
void PCA9685_setFreq(float freq)
{
	uint8_t prescale,oldmode,newmode;             //�����������޷��� 8 λ���ͱ��� ���ڴ洢Ԥ��Ƶ��ֵ���ɵ�ģʽ�Ĵ���ֵ���µ�ģʽ�Ĵ���ֵ
	double prescaleval;                       		//������һ��˫���ȸ����ͱ��� prescaleval�����ڼ���Ԥ��Ƶ����ֵ��
	freq *= 0.98f;                              	//�������Ƶ��ֵ���� 0.98������Ϊ��΢��Ƶ��ֵ����Ӧ PCA9685 ��ʵ������
	prescaleval = 25000000;                   		//���� PCA9685 �ڲ�������Ƶ��
	prescaleval /= 4096;                      		//ÿ�����ڴ�0������4095������ 4096���õ�ÿ�����������ڵ�ʱ�䣬
	prescaleval /= freq;                      		//���������Ƶ��ֵ���õ�Ԥ��Ƶ����ֵ��
	prescaleval -= 1;                        			//��ȥ 1���õ����յ�Ԥ��Ƶ��ֵ

	PCA9685_Write(REG_MODE,0x01); 						//��ӦI2C�㲥��ַ
	prescale = (uint8_t)(prescaleval+0.5f);   //������õ���Ԥ��Ƶ��ֵ��������ȡ���������丳ֵ�� prescale ������
	oldmode = PCA9685_Read(REG_MODE);         //ͨ������ PCA9685_Read ������ȡ��ǰ PCA9685 �Ĵ����е�ģʽֵ��������洢�� oldmode �����С�
	newmode = (oldmode&0x7F)|0x10;            //���ݾɵ�ģʽֵ������µ�ģʽֵ�������λ���㣨bit 7�������� 5 λ��Ϊ1��bit 4������ʾ�� PCA9685 ����Ϊ˯��ģʽ��
	PCA9685_Write(REG_MODE,newmode);          //���µ�ģʽֵд�� PCA9685 ��ģʽ�Ĵ�����
	PCA9685_Write(REG_PSC,prescale);          //������õ���Ԥ��Ƶ��ֵд�� PCA9685 ��Ԥ��Ƶ���Ĵ�����
	PCA9685_Write(REG_MODE,oldmode);          //�ָ��ɵ�ģʽֵ��
	HAL_Delay(5);                             //��ʱ5���룬�ȴ� PCA9685 ��ȫ������
	PCA9685_Write(REG_MODE,oldmode|0xa1);     //��ģʽֵ�����λ�͵� 1 λ��Ϊ1����ʾ�� PCA9685 ����Ϊ��������ģʽ��
}

//��ʼ�� pca9685 �������Ƶ��,��ʼ����Ƕ�
int PCA9685_init(float hz, uint16_t angle)
{
//	uint32_t off = 0;
//	uint8_t test;
	
	//PCA9685_Write(REG_MODE,0x01);
	//test = PCA9685_Read(REG_MODE); 
	//printf("test=%x\n",test);
	PCA9685_setFreq(hz);
	PCA9685_setAllAngle(angle);
	return 0;
}

//���ö�Ӧͨ��ת��ָ���Ƕ�
void PCA9685_setAngle(uint8_t num,uint16_t angle)
{
	uint32_t off = 0; 
	if(angle > 180) return ;
	off = (uint32_t)(102+angle*2.276);  //180�ȶ����ÿת��һ��=2.28   0����ʼλ�ã�102
	PCA9685_setPWM(num,0,off);
}

void PCA9685_setAllAngle(uint16_t angle)
{
	uint32_t off = 0;
	int i = 0;
	off = (uint32_t)(102+angle*2.276);  //180�ȶ����ÿת��һ��=2.28   0����ʼλ�ã�102
	for(;i<16;++i)
	{
		PCA9685_setPWM(i,0,off);
	}
}

//TargetAngle: Ŀ��Ƕ�  ����0-7 
//CurrentAngle: ��ǰ�Ƕ� ����0-7 
//Step: �i�� 	����0-7 
int AngleMove(const uint8_t *TargetAngle, uint8_t *CurrentAngle,const uint8_t *Step)
{
	int ret = 0;
	int i = 0;
	int diff;
	for(;i<8;++i)
	{
		if(TargetAngle[i] > CurrentAngle[i])
		{//Ŀ��Ƕȴ�
			ret = 1;
			diff = TargetAngle[i] - CurrentAngle[i];
			if(diff <= Step[i])
			{//Ŀ���<�i��
				CurrentAngle[i] = TargetAngle[i];				
			}
			else
			{//Ŀ���>�i��
				CurrentAngle[i] += Step[i];
			}
			//���ýǶ�
			PCA9685_setAngle(i,CurrentAngle[i]);
		}
		else if(TargetAngle[i] < CurrentAngle[i])
		{//Ŀ��Ƕ�С
			ret = 1;
			diff = CurrentAngle[i] - TargetAngle[i];
			if(diff <= Step[i])
			{//Ŀ���<�i��
				CurrentAngle[i] = TargetAngle[i];				
			}
			else
			{//Ŀ���>�i��
				CurrentAngle[i] -= Step[i];
			}
			//���ýǶ�
			PCA9685_setAngle(i,CurrentAngle[i]);
		}	
	}
	return ret;
}


