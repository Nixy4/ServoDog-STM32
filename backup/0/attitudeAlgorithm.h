#ifndef ATTITUDEALGORITHM_H
#define ATTITUDEALGORITHM_H

//角度逆解算
// 返回值 <0 解算失败超限
//x,y 足尖坐标, A1大腿舵机角度,A2小腿舵机角度
int InverseResolve(double x,double y,double *A1, double *A2);

#endif
