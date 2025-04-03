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

//设置pwm输出频率
void PCA9685_setFreq(float freq)
{
	uint8_t prescale,oldmode,newmode;             //定义了三个无符号 8 位整型变量 用于存储预分频器值、旧的模式寄存器值和新的模式寄存器值
	double prescaleval;                       		//定义了一个双精度浮点型变量 prescaleval，用于计算预分频器的值。
	freq *= 0.98f;                              	//将传入的频率值乘以 0.98，这是为了微调频率值以适应 PCA9685 的实际需求
	prescaleval = 25000000;                   		//这是 PCA9685 内部振荡器的频率
	prescaleval /= 4096;                      		//每个周期从0计数到4095，除以 4096，得到每个计数器周期的时间，
	prescaleval /= freq;                      		//除以所需的频率值，得到预分频器的值。
	prescaleval -= 1;                        			//减去 1，得到最终的预分频器值

	PCA9685_Write(REG_MODE,0x01); 						//响应I2C广播地址
	prescale = (uint8_t)(prescaleval+0.5f);   //将计算得到的预分频器值四舍五入取整，并将其赋值给 prescale 变量。
	oldmode = PCA9685_Read(REG_MODE);         //通过调用 PCA9685_Read 函数读取当前 PCA9685 寄存器中的模式值，并将其存储在 oldmode 变量中。
	newmode = (oldmode&0x7F)|0x10;            //根据旧的模式值计算出新的模式值，将最高位清零（bit 7）并将第 5 位设为1（bit 4），表示将 PCA9685 设置为睡眠模式。
	PCA9685_Write(REG_MODE,newmode);          //将新的模式值写入 PCA9685 的模式寄存器。
	PCA9685_Write(REG_PSC,prescale);          //将计算得到的预分频器值写入 PCA9685 的预分频器寄存器。
	PCA9685_Write(REG_MODE,oldmode);          //恢复旧的模式值。
	HAL_Delay(5);                             //延时5毫秒，等待 PCA9685 完全启动。
	PCA9685_Write(REG_MODE,oldmode|0xa1);     //将模式值的最高位和第 1 位设为1，表示将 PCA9685 设置为正常工作模式。
}

//初始化 pca9685 设置输出频率,初始舵机角度
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

//设置对应通道转到指定角度
void PCA9685_setAngle(uint8_t num,uint16_t angle)
{
	uint32_t off = 0; 
	if(angle > 180) return ;
	off = (uint32_t)(102+angle*2.276);  //180度舵机，每转动一度=2.28   0度起始位置：102
	PCA9685_setPWM(num,0,off);
}

void PCA9685_setAllAngle(uint16_t angle)
{
	uint32_t off = 0;
	int i = 0;
	off = (uint32_t)(102+angle*2.276);  //180度舵机，每转动一度=2.28   0度起始位置：102
	for(;i<16;++i)
	{
		PCA9685_setPWM(i,0,off);
	}
}

//TargetAngle: 目标角度  数组0-7 
//CurrentAngle: 当前角度 数组0-7 
//Step: 歩长 	数组0-7 
int AngleMove(const uint8_t *TargetAngle, uint8_t *CurrentAngle,const uint8_t *Step)
{
	int ret = 0;
	int i = 0;
	int diff;
	for(;i<8;++i)
	{
		if(TargetAngle[i] > CurrentAngle[i])
		{//目标角度大
			ret = 1;
			diff = TargetAngle[i] - CurrentAngle[i];
			if(diff <= Step[i])
			{//目标差<歩长
				CurrentAngle[i] = TargetAngle[i];				
			}
			else
			{//目标差>歩长
				CurrentAngle[i] += Step[i];
			}
			//设置角度
			PCA9685_setAngle(i,CurrentAngle[i]);
		}
		else if(TargetAngle[i] < CurrentAngle[i])
		{//目标角度小
			ret = 1;
			diff = CurrentAngle[i] - TargetAngle[i];
			if(diff <= Step[i])
			{//目标差<歩长
				CurrentAngle[i] = TargetAngle[i];				
			}
			else
			{//目标差>歩长
				CurrentAngle[i] -= Step[i];
			}
			//设置角度
			PCA9685_setAngle(i,CurrentAngle[i]);
		}	
	}
	return ret;
}


