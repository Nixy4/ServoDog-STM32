
#include "attitudeAlgorithm.h"
#include "math.h"
#include <stdio.h>

#define  L1  80.0f    //���ȳ��� mm
#define  L2  65.0f    //С�ȳ��� mm 
#define  L3  15.0f    //С�ȶ��Ť�˳��� mm
#define  L4  75.0f	  //���˳��� mm
#define  L5  20.0f    //���˶˵㵽С�ȹյ�ľ��� mm
#define  L6  80.0f    //С�ȶ����С�ȹյ�ľ��� mm
#define  PI  3.14f

//double fabs(double num)
//{
//	if(num>=0) return num;
//	else return -num;
//}

//�Ƕ������
// ����ֵ <0 ����ʧ�ܳ���
//x,y �������, A1���ȶ���Ƕ�,A2С�ȶ���Ƕ�
int InverseResolve(double x,double y,double *A1, double *A2)
{

    //����С�ȽǶ� ���Ҷ���
    double a2 = PI - acos((x*x + y*y - L1*L1 - L2*L2) / (-2*L1*L2));

    //����С�ȶ���Ƕ�
    double Lx = sqrt(L6*L6+L5*L5 - 2* L6*L5*cos(a2));
		printf("lx=%.2f,a2=%.2f\n",Lx,a2);
	
    if(Lx > (L3+L4) || Lx < L4-L3)
        return -1;
    double ax = acos((Lx*Lx+L3*L3 - L4*L4)/(2*L3*Lx));

    *A2 = 180*ax/PI;

    //������ȽǶ�
    double tmp = acos((L1*L1+x*x+y*y-L2*L2)/(2*L1*sqrt(x*x+y*y)));
    double a1;
    if (x>0)
        a1=fabs(atan(y/x))-tmp;
    else if(x < 0)
        a1=PI-fabs(atan(y/x))-tmp;
    else
        a1=PI - 1.5707f - tmp;

    *A1 = 180*a1/PI; 
    return 0;
}


