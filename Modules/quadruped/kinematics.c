#include "quadruped_def.h"

#define TAG "KinematicsData"

void kinematics_debug_data(KinematicsData* kdata)
{
  elog_d(TAG, "| %10s | %20s |","----------","--------------------");
  elog_d(TAG, "| %10s | %20.10f |","AS1",kdata->AS1);
  elog_d(TAG, "| %10s | %20.10f |","AS2",kdata->AS2);
  elog_d(TAG, "| %10s | %20.10f |","RS1",kdata->RS1);
  elog_d(TAG, "| %10s | %20.10f |","RS2",kdata->RS2);
  elog_d(TAG, "| %10s | %20.10f |","L6",kdata->L6);
  elog_d(TAG, "| %10s | %20.10f |","L7",kdata->L7);
  elog_d(TAG, "| %10s | %20.10f |","R12",kdata->R12);
  elog_d(TAG, "| %10s | %20.10f |","R17",kdata->R17);
  elog_d(TAG, "| %10s | %20.10f |","R35",kdata->R35);
  elog_d(TAG, "| %10s | %20.10f |","R7X",kdata->R7X);
  elog_d(TAG, "| %10s | %20.10f |","X",kdata->COORD.X);
  elog_d(TAG, "| %10s | %20.10f |","Z",kdata->COORD.Z);
}

void kinematics_inverse(Leg* leg, Coord coord)
{
  float L6, L7, R12, R17, R35, R7X, T, K, RS1, RS2;
  float X = coord.X;
  float Z = coord.Z;
  L7 = sqrt(pow(X, 2) + pow(Z, 2));
  R17 = acos((pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7));
  T = (pow(L1, 2) + pow(L2, 2) - pow(L7, 2)) / (2 * L1 * L2);
  R12 = acos(T);

  if(X==0) {
    R7X=HPI; 
  } else {
    K= Z / X;
    if(X>0&&Z>=0) R7X=atan(K);
    else if(X<0&&Z>=0) R7X=PI+atan(K);
    else if(X<0&&Z<0) R7X=PI+atan(K);
    else {R7X=0.0f;}
  }

  RS1 = R7X - R17;
  R35 = PI - R12 - R15;
  L6 = sqrt(pow(L3, 2) + pow(L5, 2) - 2 * L3 * L5 * cos(R35));
  if(L6 > L8 + L9) {goto err1;}
  T = (L6 * L6 + L8 * L8 - L9 * L9) / (2 * L6 * L8);
  RS2 = acos(T);
  leg->IKINE.AS1     = DEGREES(RS1);
  leg->IKINE.AS2     = DEGREES(RS2);
  leg->IKINE.RS1     = RS1;
  leg->IKINE.RS2     = RS2;
  leg->IKINE.L6      = L6;
  leg->IKINE.L7      = L7;
  leg->IKINE.R12     = R12;
  leg->IKINE.R17     = R17;
  leg->IKINE.R35     = R35;
  leg->IKINE.R7X     = R7X;
  leg->IKINE.COORD   = coord;
  kinematics_debug_data(&leg->IKINE);
  return;
err1:
  elog_e(TAG, "L6 Error >> X:%f Z:%f", X, Z);
  return;
}