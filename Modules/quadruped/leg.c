#include "leg.h"
#include "elog.h"

#define TAG "LEG"

void leg_init(Leg *leg, LegID id ,
  uint8_t thightServoId, uint8_t shankServoId, 
  float thightAngle, float shankAngle)
{
  if(leg == NULL) {
    elog_a(TAG, "leg_init: leg is NULL");
    return;
  }
  leg->id = id;
  leg->thightServoId = thightServoId;
  leg->shankServoId = shankServoId;
  leg->X = 0;
  leg->Z = 0;

  leg_set_angle(leg, thightAngle, shankAngle);
}

void leg_set_angle(Leg *leg, float thightAngle, float shankAngle)
{
  if( leg->id >= LEG_ID_LF )
  {
    thightAngle = 180-thightAngle;
    shankAngle = shankAngle > 120 ? 120 : shankAngle;
    shankAngle = 180-shankAngle;
  }
  else
  {
    shankAngle = shankAngle > 120 ? 120 : shankAngle;
  }
  
  leg->thightAngle = thightAngle;
  leg->shankAngle = shankAngle;  
  pca9685_set_angle(leg->thightServoId, thightAngle);
  pca9685_set_angle(leg->shankServoId, shankAngle);
}

float I1_XZ_to_L7(float X, float Z) {
    return sqrt(pow(X, 2) + pow(Z, 2));
}

float I2_L7_to_R17(float L7) {
    return acos((pow(L1, 2) + pow(L7, 2) - pow(L2, 2)) / (2 * L1 * L7));
}

float I3_L7_to_R12(float L7) {
    float T = (pow(L1, 2) + pow(L2, 2) - pow(L7, 2)) / (2 * L1 * L2);
    return acos(T);
}

float I4_XZ_to_RX7(float X, float Z) {
  if (X == 0) return PI / 2;
  float K = Z / X;
  if (X >= 0 && Z >= 0) return atan(K);
  if (X < 0 && Z >= 0) return PI + atan(K);
  if (X < 0 && Z < 0) return PI + atan(K);
  return 0;
}

float I5_R17R7X_to_RS1(float R17, float R7X) {
  return R7X - R17;
}

float I6_R12_to_R35(float R12) {
    return PI - R12 - R15;
}

float I7_R35_to_L6(float R35) {
  float L6 = sqrt(pow(L3, 2) + pow(L5, 2) - 2 * L3 * L5 * cos(R35));
  if (L6 > L8 + L9) {

  }
  return L6;
}

float I8_L6_to_RS2(float L6) {
    float T = (pow(L6, 2) + pow(L8, 2) - pow(L9, 2)) / (2 * L6 * L8);
    return acos(T);
}

KinematicsData inverse( float X, float Z)
{
  KinematicsData data;

  float L7 = I1_XZ_to_L7(X, Z);
  float R17 = I2_L7_to_R17(L7);
  float R12 = I3_L7_to_R12(L7);
  float R35 = I6_R12_to_R35(R12);
  float L6 = I7_R35_to_L6(R35);
  float RS1 = I5_R17R7X_to_RS1(R17, I4_XZ_to_RX7(X, Z));
  float RS2 = I8_L6_to_RS2(L6);

  data.AS1 = X;
  data.AS2 = Z;
  data.RS1 = RS1;
  data.RS2 = RS2;
  data.L6 = L6;
  data.L7 = L7;
  data.R12 = R12;
  data.R17 = R17;
  data.R35 = R35;
  return data;
}

void leg_set_coord(Leg *leg, float X, float Z)
{ 
  //! Kinematics !//
  leg->ikdata = inverse(X, Z);

  //! Hardware !//
  leg_set_angle(leg, leg->ikdata.RS1, leg->ikdata.RS2);
}