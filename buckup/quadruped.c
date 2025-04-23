#include "quadruped.h"

#define TAG "Quadruped"

void quad_standup0(uint16_t frames)
{
  // leg_ecoord_target(LEG_ID_RF, kfsp_x0_z_max.COORD.X, kfsp_x0_z_max.COORD.Z, frames);
  // leg_ecoord_target(LEG_ID_RB, kfsp_x0_z_max.COORD.X, kfsp_x0_z_max.COORD.Z, frames);
  // leg_ecoord_target(LEG_ID_LF, kfsp_x0_z_max.COORD.X, kfsp_x0_z_max.COORD.Z, frames);
  // leg_ecoord_target(LEG_ID_LB, kfsp_x0_z_max.COORD.X, kfsp_x0_z_max.COORD.Z, frames);
  // leg_ecoord_update_all_block();
  leg_eangle_target(LEG_ID_RF, kfsp_x0_z_max.AS1, kfsp_x0_z_max.AS2, frames);
  leg_eangle_target(LEG_ID_RB, kfsp_x0_z_max.AS1, kfsp_x0_z_max.AS2, frames);
  leg_eangle_target(LEG_ID_LF, kfsp_x0_z_max.AS1, kfsp_x0_z_max.AS2, frames);
  leg_eangle_target(LEG_ID_LB, kfsp_x0_z_max.AS1, kfsp_x0_z_max.AS2, frames);
  leg_eangle_update_all_block();
}

void quad_falldown0(uint16_t frames)
{
  // leg_ecoord_target(LEG_ID_RF, kfsp_start.COORD.X, kfsp_start.COORD.Z, frames);
  // leg_ecoord_target(LEG_ID_RB, kfsp_start.COORD.X, kfsp_start.COORD.Z, frames);
  // leg_ecoord_target(LEG_ID_LF, kfsp_start.COORD.X, kfsp_start.COORD.Z, frames);
  // leg_ecoord_target(LEG_ID_LB, kfsp_start.COORD.X, kfsp_start.COORD.Z, frames);
  // leg_ecoord_update_all_block();

  leg_eangle_target(LEG_ID_RF, kfsp_start.AS1, kfsp_start.AS2, frames);
  leg_eangle_target(LEG_ID_RB, kfsp_start.AS1, kfsp_start.AS2, frames);
  leg_eangle_target(LEG_ID_LF, kfsp_start.AS1, kfsp_start.AS2, frames);
  leg_eangle_target(LEG_ID_LB, kfsp_start.AS1, kfsp_start.AS2, frames);
  leg_eangle_update_all_block();
}

void quad_falldown1(uint16_t frames)
{
  leg_eangle_target(LEG_ID_RF, 45, 0, frames);
  leg_eangle_target(LEG_ID_RB, 45, 0, frames);
  leg_eangle_target(LEG_ID_LF, 45, 0, frames);
  leg_eangle_target(LEG_ID_LB, 45, 0, frames);
  leg_ecoord_update_all_block();
}