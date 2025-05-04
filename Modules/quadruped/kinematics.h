#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "base.h"
#include "kinematics.h"

typedef struct {
    quad_float AS1, AS2;
    quad_float RS1, RS2;
    quad_float L6, L7;
    quad_float R12, R13, R17, R35, R7X;
    quad_float X, Z;
} kinematics_data_t;

quad_float I1_XZ_to_L7(quad_float X, quad_float Z);
quad_float I2_L7_to_R17(quad_float L7);
quad_float I3_L7_to_R12(quad_float L7);
quad_float I4_XZ_to_RX7(quad_float X, quad_float Z);
quad_float I5_R17R7X_to_RS1(quad_float R17, quad_float R7X);
quad_float I6_R12_to_R35(quad_float R12);
quad_float I7_R35_to_L6(quad_float R35);
quad_float I8_L6_to_RS2(quad_float L6);
kinematics_data_t inverse_kinematics(quad_float X, quad_float Z);

quad_float F1_RS2_to_L6(quad_float RS2);
quad_float F2_L6_to_R35(quad_float L6);
quad_float F3_R15R35_to_R13(quad_float R15, quad_float R35);
quad_float F4_R13_to_R12(quad_float R13);
quad_float F5_R12_to_L7(quad_float R12);
quad_float F6_L7_to_R17(quad_float L7);
quad_float F7_RS1R17_to_R7X(quad_float RS1, quad_float R17);
void F8_L7R7X_to_xz(quad_float L7, quad_float R7X, quad_float *X, quad_float *Z);
kinematics_data_t forward_kinematics(quad_float AS1, quad_float AS2);

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
