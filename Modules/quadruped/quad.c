#include "quad.h"

#include "elog.h"
#include "base.h"
#include "pca9685.h"
#include "servo.h"
#include "leg.h"
#include "kinematics.h"
#include "quad_config.h"

#define TAG "Quad"

leg_t lf,lb,rf,rb;

void quad_init()
{
  pca9685_set_freq(50);
  leg_init(&rf,&rf_cfg);
  leg_init(&rb,&rb_cfg);
  leg_init(&lf,&lf_cfg);
  leg_init(&lb,&lb_cfg); 
  HAL_Delay(1000);
  elog_i(TAG, "Dog Init Success");
}

void quad_reset()
{
  leg_set_angle(&rf,0,0,false);
  leg_set_angle(&rb,0,0,false);
  leg_set_angle(&lf,0,0,false);
  leg_set_angle(&lb,0,0,false);
}

int quad_update()
{
  int flag ;
  flag |= leg_update(&rf);
  flag |= leg_update(&rb);
  flag |= leg_update(&lf);
  flag |= leg_update(&lb);
  return flag;
}

void quad_update_block()
{
  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);
}

void leg_special_point_test(leg_t* leg)
{
  leg_set_coord(leg,x_min.X,x_min.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,x_max.X,x_max.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,z_min.X,z_min.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,z_max.X,z_max.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,start.X,start.Z);
  HAL_Delay(1000);
  leg_set_coord(leg,end.X,end.Z);
  HAL_Delay(1000);
}

void quad_special_point_test()
{
  leg_special_point_test(&rf);
  leg_special_point_test(&rb);
  leg_special_point_test(&lf);
  leg_special_point_test(&lb);
}

void quad_move_block_x_min(quad_float ms)
{
  leg_move_target(&rf, x_min.X, x_min.Z, ms);
  leg_move_target(&rb, x_min.X, x_min.Z, ms);
  leg_move_target(&lf, x_min.X, x_min.Z, ms);
  leg_move_target(&lb, x_min.X, x_min.Z, ms);

  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);

  HAL_Delay(ms);
}

void quad_move_block_x_max(quad_float ms)
{
  leg_move_target(&rf, x_max.X, x_max.Z, ms);
  leg_move_target(&rb, x_max.X, x_max.Z, ms);
  leg_move_target(&lf, x_max.X, x_max.Z, ms);
  leg_move_target(&lb, x_max.X, x_max.Z, ms);

  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);

  HAL_Delay(ms);
}

void quad_move_block_z_min(quad_float ms)
{
  leg_move_target(&rf, z_min.X, z_min.Z, ms);
  leg_move_target(&rb, z_min.X, z_min.Z, ms);
  leg_move_target(&lf, z_min.X, z_min.Z, ms);
  leg_move_target(&lb, z_min.X, z_min.Z, ms);

  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);

  HAL_Delay(ms);
}

void quad_move_block_z_max(quad_float ms)
{
  leg_move_target(&rf, z_max.X, z_max.Z, ms);
  leg_move_target(&rb, z_max.X, z_max.Z, ms);
  leg_move_target(&lf, z_max.X, z_max.Z, ms);
  leg_move_target(&lb, z_max.X, z_max.Z, ms);

  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);

  HAL_Delay(ms);
}

void quad_move_block_start(quad_float ms)
{
  leg_move_target(&rf, start.X, start.Z, ms);
  leg_move_target(&rb, start.X, start.Z, ms);
  leg_move_target(&lf, start.X, start.Z, ms);
  leg_move_target(&lb, start.X, start.Z, ms);

  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);

  HAL_Delay(ms);
}

void quad_move_block_x0_z_max(quad_float ms)
{
  leg_move_target(&rf, x0_z_max.X, x0_z_max.Z, ms);
  leg_move_target(&rb, x0_z_max.X, x0_z_max.Z, ms);
  leg_move_target(&lf, x0_z_max.X, x0_z_max.Z, ms);
  leg_move_target(&lb, x0_z_max.X, x0_z_max.Z, ms);

  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);

  HAL_Delay(ms);
}

void quad_stand0(quad_float ms)
{
  quad_move_block_x0_z_max(ms);
  elog_i(TAG, "Dog Stand !");
}

void quad_stand1(quad_float ms)
{
  quad_float X = -10;
  quad_float Z = 110;
  leg_move_target(&rf, X, Z, ms);
  leg_move_target(&rb, X, Z, ms);
  leg_move_target(&lf, X, Z, ms);
  leg_move_target(&lb, X, Z, ms);
  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);
}

void quad_fall0(quad_float ms)
{
  quad_move_block_start(ms);
  elog_i(TAG, "Dog Fall !");
}

void quad_fall1(quad_float ms)
{
  quad_float AS1 = 50;
  quad_float AS2 = 0;
  leg_turn_target(&rf, AS1, AS2, ms);
  leg_turn_target(&rb, AS1+5, AS2, ms);
  leg_turn_target(&lf, AS1, AS2, ms);
  leg_turn_target(&lb, AS1+5, AS2, ms);
  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);
  HAL_Delay(ms);
}

void quad_squat0(quad_float ms)
{
  leg_move_target(&rf, x0_z_max.X, x0_z_max.Z, ms);
  leg_move_target(&rb, start.X, start.Z, ms);
  leg_move_target(&lf, x0_z_max.X, x0_z_max.Z, ms);
  leg_move_target(&lb, start.X, start.Z, ms);
  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);
  HAL_Delay(ms);  
  elog_i(TAG, "Dog Squat !");
}

void quad_squat1(quad_float ms)
{
  quad_float FAS1 = 30;
  quad_float FAS2 = 115;
  quad_float BAS1 = 40;
  quad_float BAS2 = 0;
  leg_turn_target(&rf, FAS1, FAS2, ms);
  leg_turn_target(&lf, FAS1, FAS2, ms);
  leg_turn_target(&rb, BAS1, BAS2, ms);
  leg_turn_target(&lb, BAS1, BAS2, ms);
  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&rb);
    flag |= leg_update(&lf);
    flag |= leg_update(&lb);
  } while(flag != 0);
  HAL_Delay(ms);
}

void quad_bow0(quad_float ms)
{
  //1.先恢复站立
  quad_stand0(ms);
  //2.前蹲
  quad_float FAS1= 30;
  quad_float FAS2= 30;
  leg_turn_target(&rf, FAS1, FAS2, ms);
  leg_turn_target(&lf, FAS1, FAS2, ms);
  int flag ;
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&lf);
  } while(flag != 0);
  HAL_Delay(ms);
  //3.恢复站立
  quad_stand0(ms);
  //4.前蹲
  leg_turn_target(&rf, FAS1, FAS2, ms);
  leg_turn_target(&lf, FAS1, FAS2, ms);
  do{
    flag = 0;
    flag |= leg_update(&rf);
    flag |= leg_update(&lf);
  } while(flag != 0);
  HAL_Delay(ms);
  //5.恢复站立
  quad_stand0(ms);
}