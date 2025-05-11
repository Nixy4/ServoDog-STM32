#include "quadruped.h"

void quad_fixed_stand0(void)
{
  leg_sync_ccb_target_blocking(cc_stand0, CONFIG_FIXED_FRAME_COUNT);
}

void quad_fixed_fall0(void)
{
  leg_sync_acb_target_blocking(0, CONFIG_FIXED_FRAME_COUNT);
}