#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

#include "base.h"
#include "kinematics.h"

typedef struct {
    QUAD_TYPE AS1, AS2;
    QUAD_TYPE RS1, RS2;
    QUAD_TYPE L6, L7;
    QUAD_TYPE R12, R13, R17, R35, R7X;
    QUAD_TYPE X, Z;
} kinematics_data_t;

QUAD_TYPE I1_XZ_to_L7(QUAD_TYPE X, QUAD_TYPE Z);
QUAD_TYPE I2_L7_to_R17(QUAD_TYPE L7);
QUAD_TYPE I3_L7_to_R12(QUAD_TYPE L7);
QUAD_TYPE I4_XZ_to_RX7(QUAD_TYPE X, QUAD_TYPE Z);
QUAD_TYPE I5_R17R7X_to_RS1(QUAD_TYPE R17, QUAD_TYPE R7X);
QUAD_TYPE I6_R12_to_R35(QUAD_TYPE R12);
QUAD_TYPE I7_R35_to_L6(QUAD_TYPE R35);
QUAD_TYPE I8_L6_to_RS2(QUAD_TYPE L6);
kinematics_data_t inverse_kinematics(QUAD_TYPE X, QUAD_TYPE Z);

QUAD_TYPE F1_RS2_to_L6(QUAD_TYPE RS2);
QUAD_TYPE F2_L6_to_R35(QUAD_TYPE L6);
QUAD_TYPE F3_R15R35_to_R13(QUAD_TYPE R15, QUAD_TYPE R35);
QUAD_TYPE F4_R13_to_R12(QUAD_TYPE R13);
QUAD_TYPE F5_R12_to_L7(QUAD_TYPE R12);
QUAD_TYPE F6_L7_to_R17(QUAD_TYPE L7);
QUAD_TYPE F7_RS1R17_to_R7X(QUAD_TYPE RS1, QUAD_TYPE R17);
void F8_L7R7X_to_xz(QUAD_TYPE L7, QUAD_TYPE R7X, QUAD_TYPE *X, QUAD_TYPE *Z);
kinematics_data_t forward_kinematics(QUAD_TYPE AS1, QUAD_TYPE AS2);

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
