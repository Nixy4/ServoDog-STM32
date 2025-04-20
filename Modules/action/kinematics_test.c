#include "kinematics.h"
#include "elog.h"

#define TAG "kinematics test"

void kinematics_special_points_test()
{
  kinematics_data_t temp = {0};

  temp = inverse_kinematics(x_min.X, x_min.Z);
  dataCompare(&x_min, &temp);
  elog_i(TAG, "x_min: %f %f", x_min.X, x_min.Z);

  temp = inverse_kinematics(x_max.X, x_max.Z);
	dataCompare(&x_max, &temp);
  elog_i(TAG, "x_max: %f %f", x_max.X, x_max.Z);

  temp = inverse_kinematics(z_min.X, z_min.Z);
	dataCompare(&z_min, &temp);
  elog_i(TAG, "z_min: %f %f", z_min.X, z_min.Z);

  temp = inverse_kinematics(z_max.X, z_max.Z);
	dataCompare(&z_max, &temp);
  elog_i(TAG, "z_max: %f %f", z_max.X, z_max.Z);

  temp = inverse_kinematics(start.X, start.Z);
	dataCompare(&start, &temp);
  elog_i(TAG, "start: %f %f", start.X, start.Z);

  temp = inverse_kinematics(end.X, end.Z);
	dataCompare(&end, &temp);
  elog_i(TAG, "end: %f %f", end.X, end.Z);
}

void kinematics_range_test()
{
  kinematics_data_t data = {0};
  for(int AS1=0;AS1<181;AS1++) {
    for(int AS2=0;AS2<121;AS2++){
      data = inverse_kinematics(forward_x_values[AS1][AS2], forward_z_values[AS1][AS2]);
      double x_diff = fabs(data.X - forward_x_values[AS1][AS2]);
      double z_diff = fabs(data.Z - forward_z_values[AS1][AS2]);
      if(x_diff > 0.1 || z_diff > 0.1) {
        elog_e(TAG, "x_diff: %f %f z_diff: %f %f", x_diff, forward_x_values[AS1][AS2], z_diff, forward_z_values[AS1][AS2]);
      }else {
        elog_i(TAG, "pass %d %d", AS1,AS2);
      }
    }
  }
}