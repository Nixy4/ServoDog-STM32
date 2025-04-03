#include "stdio.h"
#include <stdlib.h>
#include <string.h>
#include "PCA9685.h"
#include "attitudeAlgorithm.h"
#include "DogMotion.h"

extern volatile uint8_t RecvedCmd;

//����ÿ������Ƕ�ƫ�� ID0-7
//const int Px[8] = {0,0,0,0,10,15,5,0};
const int Px[8] = {0,0,0,5,10,0,0,0};
//��������ƫ�� ����,С��
const int APx[2] = {-30,0};

//����������
typedef struct Leg_t
{
	uint8_t id; //��ID
	int x; //������
	int y; 
}Leg_t;

//����һ����������
typedef struct Action_t
{
	uint16_t Delay; // ����ִ�е�ʱ��
	Leg_t Leg[4]; //4��������
}Action_t;


//���嶯�� ���� �� NULL��β
static const char *run[] = {
	"100>4:-20,25;1:-20,25;2:20,30;3:20,30;",
	"100>4:-20,30;1:-20,30;2:20,30;3:20,30;",
	"100>4:0,30;1:0,30;2:0,25;3:0,25;",
	"100>4:20,30;1:20,30;2:-20,25;3:-20,25;",
	NULL
};

static const char *back[] = {
//	#  ̧��Խ������� �������
"200>4:20,25;1:21,25;2:-21,30;3:-20,30;",
 
"100>4:20,30;1:21,30;2:-21,30;3:-20,30;",
//  ���µ�����ǰ�� ̧����� ������� 
"200>4:-20,30;1:-21,30;2:21,25;3:20,25;",
	NULL
};

static const char *Right[] = 
{
//#  ̧��Խ������� ����ǰ�� 
"100>2:0,25;3:-20,25;4:20,30;1:0,30;",
//#  ����̧�����  
"50>2:0,30;3:-20,30;4:20,30;1:0,30;",
//#  ���µ�����ǰ�� ̧����� ����ǰ�� 10
"100>2:0,30;3:20,30;4:-20,25;1:0,25;",
	NULL
};

static const char *Left[] = 
{
"100>4:0,25;1:-20,25;2:20,30;3:0,30;",
//#  ����̧�����  
"50>4:0,30;1:-20,30;2:20,30;3:0,30;",
//#  ���µ�����ǰ�� ̧����� ����ǰ�� 10
"100>4:0,30;1:20,30;2:-20,25;3:0,25;",
	NULL
};

//ſ��
static const char *Down[] = 
{
	"50>1:0,0;2:0,0;3:0,0;4:0,0;",
	NULL
};

//վ��
static const char *Up[] = 
{
	"50>1:0,30;2:0,30;3:0,30;4:0,30;",
	NULL
};

//����
static const char *Dun[] = 
{
	"50>1:0,30;2:0,0;3:0,30;4:0,0;",
	NULL
};

//�Ϲ�
static const char *Bow[] = {
	"1500>1:0,30;2:0,30;3:0,30;4:0,30;",
	"1500>1:0,0;2:0,30;3:0,0;4:0,30",
	NULL
};

/*
const Action_t Action_Run[4] = {
{100, {4,-20,25},},
{},
{},
{},
};
*/

//ͨ����ID���ö�� ͬʱ����ƫ��
void SetAngelByID(uint8_t ID, int A1,int A2)
{
	switch(ID)
	{
		case 1: 
			PCA9685_setAngle(0,A1+Px[0]);
			PCA9685_setAngle(5,A2+Px[5]);
			break;
		case 2: 
			PCA9685_setAngle(1,A1+Px[1]);
			PCA9685_setAngle(4,A2+Px[4]);
			break;
		//�������Ҫ��תһ��
		case 3: 
			PCA9685_setAngle(3,180 - (A1+Px[3]));
			PCA9685_setAngle(7,180 - (A2+Px[7]));
			break;
		case 4: 
			PCA9685_setAngle(2,180 - (A1+Px[2]));
			PCA9685_setAngle(6,180 - (A2+Px[6]));	
			break;
	}
}
	
//��������������	���ض�����ʱʱ��
int XYLineToAngel(const char *cmdline)	
{
	int id,x,y;
	char *p = strstr(cmdline,";");
	double A1,A2;
	int ret;
	int time = 0;
	time = atoi(cmdline);
	cmdline = strstr(cmdline,">") +1;
	printf("time=%d\n",time);
	for(;p && *cmdline;cmdline = p+1)
	{
		id = *cmdline - '0';
		x = atoi(cmdline+2);
		p = strstr(cmdline, ",");
		y = atoi(p+1);
		y += 100; //y����ƫ�� 100
		printf("id=%d,x=%d,y=%d\n",id,x,y);
		//�������
		
		ret = InverseResolve(x,y,&A1,&A2);
		printf("ret=%d,A1=%.2f,A2=%.2f\n",ret,A1,A2);
		if(ret == 0)
		{
			A1+=APx[0];//�������ƫ��
			A2+=APx[1];
			SetAngelByID(id,A1,A2);
		}
		p = strstr(cmdline,";");
	}
	return time;
}

//ִ�ж�������
void DogAction(const char *ActionList[])
{
	uint8_t tmp;
	uint32_t tic;
	tmp = RecvedCmd;
	int delay;
	for(int i=0;ActionList[i];i++)
	{
		delay = XYLineToAngel(ActionList[i]);
		tic = HAL_GetTick();
		while(HAL_GetTick() - tic < delay)
		{
			if(tmp != RecvedCmd) return;
		}
	}
}


void DogRun(void) // ����ǰ��
{
	printf("DogRun...\n");
	DogAction(run);
}

void DogBack(void) //����
{
	DogAction(back);
}

void DogLeft(void) // ��ת
{
	DogAction(Left);
}

void DogRight(void) //��ת
{
	DogAction(Right);
}

void DogDown(void) // ſ��
{
	DogAction(Down);
}

void DogUp(void) //վ��
{
	DogAction(Up);
}

void DogSquat(void) //��
{
	DogAction(Dun);
}

void DogBow(void) //�Ϲ�
{
	DogAction(Bow);
}

void DogWave(void)//����
{
	uint8_t tmp;
	uint32_t tic;
	tmp = RecvedCmd;
	int delay = 300;
	//1�ȶ�
	DogAction(Dun);
	while(1)
	{
		PCA9685_setAngle(5,130);
		PCA9685_setAngle(0,108);
		//2�ٻ���
		tic = HAL_GetTick();
		while(HAL_GetTick() - tic < delay)
		{
			if(tmp != RecvedCmd) return;
		}
		
		PCA9685_setAngle(5,113);
		PCA9685_setAngle(0,126);
		//2�ٻ���
		tic = HAL_GetTick();
		while(HAL_GetTick() - tic < delay)
		{
			if(tmp != RecvedCmd) return;
		}	
	}
}

