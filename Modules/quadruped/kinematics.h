#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include "base.h"

typedef struct {
    double AS1, AS2;
    double RS1, RS2;
    double L6, L7;
    double R12, R13, R17, R35, R7X;
    double X, Z;
} kinematics_data_t;

double I1_XZ_to_L7(double X, double Z);
double I2_L7_to_R17(double L7);
double I3_L7_to_R12(double L7);
double I4_XZ_to_RX7(double X, double Z);
double I5_R17R7X_to_RS1(double R17, double R7X);
double I6_R12_to_R35(double R12);
double I7_R35_to_L6(double R35);
double I8_L6_to_RS2(double L6);
kinematics_data_t inverse_kinematics(double X, double Z);

double F1_RS2_to_L6(double RS2);
double F2_L6_to_R35(double L6);
double F3_R15R35_to_R13(double R15, double R35);
double F4_R13_to_R12(double R13);
double F5_R12_to_L7(double R12);
double F6_L7_to_R17(double L7);
double F7_RS1R17_to_R7X(double RS1, double R17);
void F8_L7R7X_to_xz(double L7, double R7X, double *X, double *Z);
kinematics_data_t forward_kinematics(double AS1, double AS2);

bool dataCompare(const kinematics_data_t* d1, const kinematics_data_t* d2);
void dataElog(const char* data_tag, const kinematics_data_t* data);

//测试程序
extern const kinematics_data_t x_min;
extern const kinematics_data_t x_max;
extern const kinematics_data_t z_min;
extern const kinematics_data_t z_max;
extern const kinematics_data_t start;
extern const kinematics_data_t end;
extern const kinematics_data_t x0_z_min;
extern const kinematics_data_t x0_z_max;
extern const int forward_x_values[181][121];
extern const int forward_z_values[181][121];
void kinematics_special_points_test();
void kinematics_range_test();

#endif // KINEMATICS_H
