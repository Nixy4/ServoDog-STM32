// leg_t rf = {0};
// leg_t rb = {0};
// leg_t lf = {0};
// leg_t lb = {0};

// leg_t* legs_ptr[4] = {&rf, &rb, &lf, &lb};

// void legs_init(const leg_config_t* cfgs_ptr[4])
// {
//   pca9685_set_freq(50);
//   for(int i = 0; i < LEG_ID_MAX; i++){
//     leg_init(legs_ptr[i], cfgs_ptr[i]);
//   }
// }

// int legs_update()
// {
//   int flag ;
//   flag |= leg_update(&rf);
//   flag |= leg_update(&rb);
//   flag |= leg_update(&lf);
//   flag |= leg_update(&lb);
//   return flag;
// }

// void legs_update_block()
// {
//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);
// }

// void legs_move_block_x_min(double ms)
// {
//   leg_move_target(&rf, x_min.X, x_min.Z, ms);
//   leg_move_target(&rb, x_min.X, x_min.Z, ms);
//   leg_move_target(&lf, x_min.X, x_min.Z, ms);
//   leg_move_target(&lb, x_min.X, x_min.Z, ms);

//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);

//   HAL_Delay(ms);
// }

// void legs_move_block_x_max(double ms)
// {
//   leg_move_target(&rf, x_max.X, x_max.Z, ms);
//   leg_move_target(&rb, x_max.X, x_max.Z, ms);
//   leg_move_target(&lf, x_max.X, x_max.Z, ms);
//   leg_move_target(&lb, x_max.X, x_max.Z, ms);

//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);

//   HAL_Delay(ms);
// }

// void legs_move_block_z_min(double ms)
// {
//   leg_move_target(&rf, z_min.X, z_min.Z, ms);
//   leg_move_target(&rb, z_min.X, z_min.Z, ms);
//   leg_move_target(&lf, z_min.X, z_min.Z, ms);
//   leg_move_target(&lb, z_min.X, z_min.Z, ms);

//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);

//   HAL_Delay(ms);
// }

// void legs_move_block_z_max(double ms)
// {
//   leg_move_target(&rf, z_max.X, z_max.Z, ms);
//   leg_move_target(&rb, z_max.X, z_max.Z, ms);
//   leg_move_target(&lf, z_max.X, z_max.Z, ms);
//   leg_move_target(&lb, z_max.X, z_max.Z, ms);

//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);

//   HAL_Delay(ms);
// }

// void legs_move_block_start(double ms)
// {
//   leg_move_target(&rf, start.X, start.Z, ms);
//   leg_move_target(&rb, start.X, start.Z, ms);
//   leg_move_target(&lf, start.X, start.Z, ms);
//   leg_move_target(&lb, start.X, start.Z, ms);

//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);

//   HAL_Delay(ms);
// }

// void legs_move_block_x0_z_max(double ms)
// {
//   leg_move_target(&rf, x0_z_max.X, x0_z_max.Z, ms);
//   leg_move_target(&rb, x0_z_max.X, x0_z_max.Z, ms);
//   leg_move_target(&lf, x0_z_max.X, x0_z_max.Z, ms);
//   leg_move_target(&lb, x0_z_max.X, x0_z_max.Z, ms);

//   int flag ;
//   do{
//     flag = 0;
//     flag |= leg_update(&rf);
//     flag |= leg_update(&rb);
//     flag |= leg_update(&lf);
//     flag |= leg_update(&lb);
//   } while(flag != 0);

//   HAL_Delay(ms);
// }