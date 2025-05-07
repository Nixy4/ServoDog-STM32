#include "./define.h"

#define TAG "Kine"

void kine_elog_d(quad_kine* kine)
{
  elog_d(TAG, "|%30s|","------------------------------");
  elog_d(TAG, "| %5s : %20.10f |", "AS1", kine->AS1);
  elog_d(TAG, "| %5s : %20.10f |", "AS2", kine->AS2);
  elog_d(TAG, "| %5s : %20.10f |", "X",   kine->X);
  elog_d(TAG, "| %5s : %20.10f |", "Z",   kine->Z);
  elog_d(TAG, "| %5s : %20.10f |", "RS1", kine->RS1);
  elog_d(TAG, "| %5s : %20.10f |", "RS2", kine->RS2);
  elog_d(TAG, "| %5s : %20.10f |", "L6",  kine->L6);
  elog_d(TAG, "| %5s : %20.10f |", "L7",  kine->L7);
  elog_d(TAG, "| %5s : %20.10f |", "R12", kine->R12);
  elog_d(TAG, "| %5s : %20.10f |", "R13", kine->R13);
  elog_d(TAG, "| %5s : %20.10f |", "R17", kine->R17);
  elog_d(TAG, "| %5s : %20.10f |", "R35", kine->R35);
  elog_d(TAG, "| %5s : %20.10f |", "R7X", kine->R7X);
}

static void kine_compare_elog(bool eq, const char* tag, quad_fp val1, quad_fp val2) 
{
	if (eq) {
		elog_d(TAG,"| %5s | %20.10f | %20.10f | %20.10f |", tag, val1, val2, fabs(val1 - val2));
	} else {
		elog_e(TAG,"| %5s | %20.10f | %20.10f | %20.10f |", tag, val1, val2, fabs(val1 - val2));
	}
}

bool kine_compare(const quad_kine* kine1, const quad_kine* kine2)
{
  bool AS1_eq = fabs(kine1->AS1 - kine2->AS1) <= CONFIG_DEGREE_ERROR_RANGE;
  bool AS2_eq = fabs(kine1->AS2 - kine2->AS2) <= CONFIG_DEGREE_ERROR_RANGE;
  bool X_eq   = fabs(kine1->X - kine2->X) <= CONFIG_LENGTH_ERROR_RANGE;
  bool Z_eq   = fabs(kine1->Z - kine2->Z) <= CONFIG_LENGTH_ERROR_RANGE;

  bool RS1_eq = fabs(kine1->RS1 - kine2->RS1) <= CONFIG_RADIAN_ERROR_RANGE;
  bool RS2_eq = fabs(kine1->RS2 - kine2->RS2) <= CONFIG_RADIAN_ERROR_RANGE;
  bool L6_eq  = fabs(kine1->L6 - kine2->L6) <= CONFIG_LENGTH_ERROR_RANGE;
  bool L7_eq  = fabs(kine1->L7 - kine2->L7) <= CONFIG_LENGTH_ERROR_RANGE;
  bool R12_eq = fabs(kine1->R12 - kine2->R12) <= CONFIG_RADIAN_ERROR_RANGE;
  bool R13_eq = fabs(kine1->R13 - kine2->R13) <= CONFIG_RADIAN_ERROR_RANGE;
  bool R17_eq = fabs(kine1->R17 - kine2->R17) <= CONFIG_RADIAN_ERROR_RANGE;
  bool R35_eq = fabs(kine1->R35 - kine2->R35) <= CONFIG_RADIAN_ERROR_RANGE;
  bool R7X_eq = fabs(kine1->R7X - kine2->R7X) <= CONFIG_RADIAN_ERROR_RANGE;

  bool ALL_eq = AS1_eq && AS2_eq && X_eq && Z_eq &&
               RS1_eq && RS2_eq && L6_eq && L7_eq &&
               R12_eq && R13_eq && R17_eq && R35_eq && R7X_eq;

  if(ALL_eq==false)
  {
    elog_e(TAG,"|运动学计算数据异常|");
    kine_compare_elog(AS1_eq, "AS1",  kine1->AS1, kine2->AS1);
    kine_compare_elog(AS2_eq, "AS2",  kine1->AS2, kine2->AS2);
    kine_compare_elog(X_eq,   "X",    kine1->X,   kine2->X  );
    kine_compare_elog(Z_eq,   "Z",    kine1->Z,   kine2->Z  );
    kine_compare_elog(RS1_eq, "RS1",  kine1->RS1, kine2->RS1);
    kine_compare_elog(RS2_eq, "RS2",  kine1->RS2, kine2->RS2);
    kine_compare_elog(L6_eq,  "L6",   kine1->L6,  kine2->L6 );
    kine_compare_elog(L7_eq,  "L7",   kine1->L7,  kine2->L7 );
    kine_compare_elog(R12_eq, "R12",  kine1->R12, kine2->R12);
    kine_compare_elog(R13_eq, "R13",  kine1->R13, kine2->R13);
    kine_compare_elog(R17_eq, "R17",  kine1->R17, kine2->R17);
    kine_compare_elog(R35_eq, "R35",  kine1->R35, kine2->R35);
    kine_compare_elog(R7X_eq, "R7X",  kine1->R7X, kine2->R7X);
  }
  return ALL_eq;
}

void kine_forward(quad_kine* kine, quad_fp AS1, quad_fp AS2)
{
  quad_fp RS1,RS2;
  quad_fp L6,L7;
  quad_fp R12,R13;
  quad_fp R17,R35;
  quad_fp R7X;
  quad_fp X,Z;
  quad_fp T,_a,_b,_c,_d;

  // 角度检测
  // if(AS1 < CONFIG_THIGH_SERVO_ANGLE_MIN)
  // {
  //   elog_w(TAG, "AS1 < CONFIG_THIGH_SERVO_ANGLE_MIN");
  // }
  // else if(AS1 > CONFIG_THIGH_SERVO_ANGLE_MAX)
  // {
  //   elog_w(TAG, "AS1 > CONFIG_THIGH_SERVO_ANGLE_MAX");
  // }
  // if(AS2 < CONFIG_SHANK_SERVO_ANGLE_MIN)
  // {
  //   elog_w(TAG, "AS2 < CONFIG_SHANK_SERVO_ANGLE_MIN");
  // }
  // else if(AS2 > CONFIG_SHANK_SERVO_ANGLE_MAX)
  // {
  //   elog_w(TAG, "AS2 > CONFIG_SHANK_SERVO_ANGLE_MAX");
  // }

  //角度转换
  RS1 = radians(AS1);
  RS2 = radians(AS2);

  //F1_RS2_to_L6
  _a = 1;
  _b = -2 * L8 * cos(RS2);
  _c = pow(L8,2) - pow(L9,2);
  _d = pow(_b,2) - 4 * _a * _c;
  if(_d < 0)
  {
    elog_e(TAG, "F1:_d < 0");
    return;
  }
  L6 = (-_b + sqrt(_d)) / (2 * _a);

  //F2_L6_to_R35
  T = (pow(L3, 2) + pow(L5, 2) - pow(L6, 2)) / (2 * L3 * L5);
  if(T < -1 || T > 1)
  {
    elog_e(TAG, "F2:T < -1 || T > 1");
    // return;
  }
  R35 = acos(T);

  //F3_R15R35_to_R13
  R13 = R15 + R35;

  //F4_R13_to_R12
  R12 = PI - R13;

  //F5_R12_to_L7
  L7 = sqrt( pow(L1,2)  + pow(L2,2)  - 2 * L1 * L2 * cos(R12));

  //F6_L7_to_R17
  T = (pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7);
  if(T < -1 || T > 1)
  {
    elog_e(TAG, "F6:T < -1 || T > 1");
    // return;
  }
  R17 = acos(T);  

  //F7_RS1R17_to_R7X
  R7X = RS1 + R17;

  //F8_L7R7X_to_xz
  X = L7 * cos(R7X);
  Z = L7 * sin(R7X);

  kine->AS1 = AS1;
  kine->AS2 = AS2;
  kine->RS1 = RS1;
  kine->RS2 = RS2;
  kine->L6 = L6;
  kine->L7 = L7;
  kine->R12 = R12;
  kine->R13 = R13;
  kine->R17 = R17;
  kine->R35 = R35;
  kine->R7X = R7X;
  kine->X = X;
  kine->Z = Z;
}

void kine_inverse(quad_kine* kine, quad_fp X, quad_fp Z)
{ 
  quad_fp AS1,AS2;
  quad_fp RS1,RS2;
  quad_fp L6,L7;
  quad_fp R12,R13;
  quad_fp R17,R35;
  quad_fp R7X;
  quad_fp T,K;

  //I1_XZ_to_L7
  L7 = sqrt(pow(X, 2) + pow(Z, 2));

  //I2_L7_to_R17
  T = (pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7);
  R17 = acos(T);

  //I3_L7_to_R12
  T = (pow(L1, 2) + pow(L2, 2) - pow(L7, 2)) / (2 * L1 * L2);
  R12 = acos(T);

  //I4_XZ_to_RX7
  if (X == 0) { 
    R7X = HPI;
  } else {
    K = Z / X;
    if (X > 0 && Z >= 0) {
      R7X = atan(K);
    } else if (X < 0 && Z >= 0) {
      R7X = PI + atan(K);
    } else if (Z < 0 && X < 0) {
      R7X = PI + atan(K);
    } else {
      R7X = 0.0f;
    }
  }

  //I5_R17R7X_to_RS1
  RS1 = R7X - R17;
  R13 = PI - R12;

  //I6_R12_to_R35
  R35 = PI - R12 - R15;

  //I7_R35_to_L6
  L6 = sqrt( pow(L3, 2) + pow(L5, 2) - 2 * L3 * L5 * cos(R35) );
  if( L6 > CONFIG_L6_MAX ) {
    // elog_e(TAG, "I7:L6 > CONFIG_L6_MAX");
    L6 = CONFIG_L6_MAX;
  } else if( L6 < CONFIG_L6_MIN ) {
    // elog_e(TAG, "I7:L6 < CONFIG_L6_MIN");
    L6 = CONFIG_L6_MIN;
  }

  //I8_L6_to_RS2
  T = (pow(L6,2) + pow(L8,2) - pow(L9,2)) / (2 * L6 * L8);
  RS2 = acos(T);
  AS1 = degrees(RS1);
  AS2 = degrees(RS2);

  // if (AS1 < CONFIG_THIGH_SERVO_ANGLE_MIN) {
  //   elog_w(TAG, "AS1 < CONFIG_THIGH_SERVO_ANGLE_MIN");
  // } else if (AS1 > CONFIG_THIGH_SERVO_ANGLE_MAX) {
  //   elog_w(TAG, "AS1 > CONFIG_THIGH_SERVO_ANGLE_MAX");
  // }
  // if (AS2 < CONFIG_SHANK_SERVO_ANGLE_MIN) {
  //   elog_w(TAG, "AS2 < CONFIG_SHANK_SERVO_ANGLE_MIN");
  // } else if (AS2 > CONFIG_SHANK_SERVO_ANGLE_MAX) {
  //   elog_w(TAG, "AS2 > CONFIG_SHANK_SERVO_ANGLE_MAX");
  // }

  kine->AS1 = AS1;
  kine->AS2 = AS2;
  kine->RS1 = RS1;
  kine->RS2 = RS2;
  kine->L6 = L6;
  kine->L7 = L7;
  kine->R12 = R12;
  kine->R13 = R13;
  kine->R17 = R17;
  kine->R35 = R35;
  kine->R7X = R7X;
  kine->X = X;
  kine->Z = Z;
}

void kine_sptest()
{
  quad_kine kine = {0};
  kine_forward(&kine, ksp_x_min.AS1, ksp_x_min.AS2);
  elog_i(TAG, "kine_forward : ksp_x_min");
  kine_compare(&kine, &ksp_x_min);
  elog_i(TAG, "kine_forward : ksp_x_max");
  kine_forward(&kine, ksp_x_max.AS1, ksp_x_max.AS2);
  kine_compare(&kine, &ksp_x_max);
  elog_i(TAG, "kine_forward : ksp_z_min");
  kine_forward(&kine, ksp_z_min.AS1, ksp_z_min.AS2);
  kine_compare(&kine, &ksp_z_min);
  elog_i(TAG, "kine_forward : ksp_z_max");
  kine_forward(&kine, ksp_z_max.AS1, ksp_z_max.AS2);
  kine_compare(&kine, &ksp_z_max);
  elog_i(TAG, "kine_forward : ksp_x0_z_min");
  kine_forward(&kine, ksp_x0_z_min.AS1, ksp_x0_z_min.AS2);
  kine_compare(&kine, &ksp_x0_z_min);
  elog_i(TAG, "kine_forward : ksp_x0_z_max");
  kine_forward(&kine, ksp_x0_z_max.AS1, ksp_x0_z_max.AS2);
  kine_compare(&kine, &ksp_x0_z_max);
  elog_i(TAG, "kine_forward : ksp_start");
  kine_forward(&kine, ksp_start.AS1, ksp_start.AS2);
  kine_compare(&kine, &ksp_start);
  elog_i(TAG, "kine_forward : ksp_end");
  kine_forward(&kine, ksp_end.AS1, ksp_end.AS2);
  kine_compare(&kine, &ksp_end);

  elog_i(TAG, "kine_inverse : ksp_x_min");
  kine_inverse(&kine, ksp_x_min.X, ksp_x_min.Z);
  kine_compare(&kine, &ksp_x_min);
  elog_i(TAG, "kine_inverse : ksp_x_max");
  kine_inverse(&kine, ksp_x_max.X, ksp_x_max.Z);
  kine_compare(&kine, &ksp_x_max);
  elog_i(TAG, "kine_inverse : ksp_z_min");
  kine_inverse(&kine, ksp_z_min.X, ksp_z_min.Z);
  kine_compare(&kine, &ksp_z_min);
  elog_i(TAG, "kine_inverse : ksp_z_max");
  kine_inverse(&kine, ksp_z_max.X, ksp_z_max.Z);
  kine_compare(&kine, &ksp_z_max);
  elog_i(TAG, "kine_inverse : ksp_x0_z_min");
  kine_inverse(&kine, ksp_x0_z_min.X, ksp_x0_z_min.Z);
  kine_compare(&kine, &ksp_x0_z_min);
  elog_i(TAG, "kine_inverse : ksp_x0_z_max");
  kine_inverse(&kine, ksp_x0_z_max.X, ksp_x0_z_max.Z);
  kine_compare(&kine, &ksp_x0_z_max);
  elog_i(TAG, "kine_inverse : ksp_start");
  kine_inverse(&kine, ksp_start.X, ksp_start.Z);
  kine_compare(&kine, &ksp_start);
  elog_i(TAG, "kine_inverse : ksp_end");
  kine_inverse(&kine, ksp_end.X, ksp_end.Z);
  kine_compare(&kine, &ksp_end);
}

void kine_fptest()
{
  for(uint32_t AS1 = CONFIG_THIGH_SERVO_ANGLE_MIN; AS1 < CONFIG_THIGH_SERVO_ANGLE_MAX+1; AS1 += 1)
  {
    for(uint32_t AS2 = CONFIG_SHANK_SERVO_ANGLE_MIN; AS2 < CONFIG_SHANK_SERVO_ANGLE_MAX+1; AS2 += 1)
    {
      quad_kine forward = {0};
      quad_kine inverse = {0};
      elog_i(TAG, "kine_forward : AS1 = %u, AS2 = %u", AS1, AS2);
      kine_forward(&forward, (quad_fp)AS1, (quad_fp)AS2);
      kine_inverse(&inverse, kfp[AS1][AS2].X, kfp[AS1][AS2].Z);
      kine_compare(&forward, &inverse);
    }
  }
}