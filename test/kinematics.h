#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

typedef struct {
    double AS1, AS2;
    double RS1, RS2;
    double L6, L7;
    double R12, R13, R17, R35, R7X;
    double X, Z;
} Data;

double I1_XZ_to_L7(double X, double Z);
double I2_L7_to_R17(double L7);
double I3_L7_to_R12(double L7);
double I4_XZ_to_RX7(double X, double Z);
double I5_R17R7X_to_RS1(double R17, double R7X);
double I6_R12_to_R35(double R12);
double I7_R35_to_L6(double R35);
double I8_L6_to_RS2(double L6);
Data caculateFromXZ(double X, double Z);
bool dataCompare(const Data* d1, const Data* d2);
void dataElog(const char* data_tag, const Data* data);

extern const Data x_min;
extern const Data x_max;
extern const Data z_min;
extern const Data z_max;
extern const Data start;
extern const Data end;

#endif // KINEMATICS_H
