#include "quadruped.h"

#define TAG "LegSync"

void leg_sync_set_angle(quad_fp angle_thigh, quad_fp angle_shank, bool kine_update)
{
  quad_fp angles[SERVO_COUNT] = {0};

  //! Kinematics
  if(kine_update) {
    kine_forward(&_kine[LEG_RF], angle_thigh, angle_shank);
    kine_forward(&_kine[LEG_LF], angle_thigh, angle_shank);
    kine_forward(&_kine[LEG_RB], angle_thigh, angle_shank);
    kine_forward(&_kine[LEG_LB], angle_thigh, angle_shank);
  }

  //! Hardware
  angles[SERVO_RF_T] = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RF_S] = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RF_T] = servo_angle_limit_thigh(angles[SERVO_RF_T]);
  angles[SERVO_RF_S] = servo_angle_limit_shank(angles[SERVO_RF_S]);

  angles[SERVO_LF_T] = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LF_S] = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LF_T] = servo_angle_limit_thigh(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_limit_shank(angles[SERVO_LF_S]);
  angles[SERVO_LF_T] = servo_angle_mirror(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_mirror(angles[SERVO_LF_S]);

  angles[SERVO_RB_T] = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RB_S] = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RB_T] = servo_angle_limit_thigh(angles[SERVO_RB_T]);
  angles[SERVO_RB_S] = servo_angle_limit_shank(angles[SERVO_RB_S]);
  
  angles[SERVO_LB_T] = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LB_S] = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LB_T] = servo_angle_limit_thigh(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_limit_shank(angles[SERVO_LB_S]);
  angles[SERVO_LB_T] = servo_angle_mirror(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_mirror(angles[SERVO_LB_S]);

  servo_set_angle_sync(angles);
}

void leg_sync_set_coord(quad_coord coord)
{
  quad_fp angles[SERVO_COUNT] = {0};

  //! CCB update
  for (leg_index i = LEG_RF; i < LEG_COUNT; i++) {
    _ccb[i].current_x = coord.X;
    _ccb[i].current_z = coord.Z;
  }
  _sync_ccb.current_x = coord.X;
  _sync_ccb.current_z = coord.Z;

  //! Coord offset
  quad_coord coords[LEG_COUNT] = {0};
  coords[LEG_RF] = leg_coord_offset(LEG_RF, coord);
  coords[LEG_LF] = leg_coord_offset(LEG_LF, coord);
  coords[LEG_RB] = leg_coord_offset(LEG_RB, coord);
  coords[LEG_LB] = leg_coord_offset(LEG_LB, coord);

  //! Kinematics
  kine_inverse(&_kine[LEG_RF], coords[LEG_RF].X, coords[LEG_RF].Z);
  kine_inverse(&_kine[LEG_LF], coords[LEG_LF].X, coords[LEG_LF].Z);
  kine_inverse(&_kine[LEG_RB], coords[LEG_RB].X, coords[LEG_RB].Z);
  kine_inverse(&_kine[LEG_LB], coords[LEG_LB].X, coords[LEG_LB].Z);
  
  angles[SERVO_RF_T] = _kine[LEG_RF].AS1;
  angles[SERVO_RF_S] = _kine[LEG_RF].AS2;
  angles[SERVO_LF_T] = _kine[LEG_LF].AS1;
  angles[SERVO_LF_S] = _kine[LEG_LF].AS2;
  angles[SERVO_RB_T] = _kine[LEG_RB].AS1;
  angles[SERVO_RB_S] = _kine[LEG_RB].AS2;
  angles[SERVO_LB_T] = _kine[LEG_LB].AS1;
  angles[SERVO_LB_S] = _kine[LEG_LB].AS2;

  //! ACB update
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].current = angles[i];
  }

  //! Hardware
  angles[SERVO_RF_T] = servo_angle_offset(angles[SERVO_RF_T], CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RF_S] = servo_angle_offset(angles[SERVO_RF_S], CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RF_T] = servo_angle_limit_thigh(angles[SERVO_RF_T]);
  angles[SERVO_RF_S] = servo_angle_limit_shank(angles[SERVO_RF_S]);

  angles[SERVO_LF_T] = servo_angle_offset(angles[SERVO_LF_T], CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LF_S] = servo_angle_offset(angles[SERVO_LF_S], CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LF_T] = servo_angle_limit_thigh(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_limit_shank(angles[SERVO_LF_S]);
  angles[SERVO_LF_T] = servo_angle_mirror(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_mirror(angles[SERVO_LF_S]);

  angles[SERVO_RB_T] = servo_angle_offset(angles[SERVO_RB_T], CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RB_S] = servo_angle_offset(angles[SERVO_RB_S], CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RB_T] = servo_angle_limit_thigh(angles[SERVO_RB_T]);
  angles[SERVO_RB_S] = servo_angle_limit_shank(angles[SERVO_RB_S]);
  
  angles[SERVO_LB_T] = servo_angle_offset(angles[SERVO_LB_T], CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LB_S] = servo_angle_offset(angles[SERVO_LB_S], CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LB_T] = servo_angle_limit_thigh(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_limit_shank(angles[SERVO_LB_S]);
  angles[SERVO_LB_T] = servo_angle_mirror(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_mirror(angles[SERVO_LB_S]);

  servo_set_angle_sync(angles);
}

void leg_sync_set_angle2(quad_fp angles[SERVO_COUNT], bool kine_update)
{
  //! Kinematics
  if(kine_update) {
    kine_forward(&_kine[LEG_RF], angles[SERVO_RF_T], angles[SERVO_RF_S]);
    kine_forward(&_kine[LEG_LF], angles[SERVO_LF_T], angles[SERVO_LF_S]);
    kine_forward(&_kine[LEG_RB], angles[SERVO_RB_T], angles[SERVO_RB_S]);
    kine_forward(&_kine[LEG_LB], angles[SERVO_LB_T], angles[SERVO_LB_S]);

    //! CCB update
    for (leg_index i = LEG_RF; i < LEG_COUNT; i++) {
      _ccb[i].current_x = _kine[i].AS1;
      _ccb[i].current_z = _kine[i].AS2;
    }
  }
  //! ACB update
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].current = angles[i];
  }

  //! Hardware
  angles[SERVO_RF_T] = servo_angle_offset(angles[SERVO_RF_T], CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RF_S] = servo_angle_offset(angles[SERVO_RF_S], CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RF_T] = servo_angle_limit_thigh(angles[SERVO_RF_T]);
  angles[SERVO_RF_S] = servo_angle_limit_shank(angles[SERVO_RF_S]);

  angles[SERVO_LF_T] = servo_angle_offset(angles[SERVO_LF_T], CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LF_S] = servo_angle_offset(angles[SERVO_LF_S], CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LF_T] = servo_angle_limit_thigh(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_limit_shank(angles[SERVO_LF_S]);
  angles[SERVO_LF_T] = servo_angle_mirror(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_mirror(angles[SERVO_LF_S]);

  angles[SERVO_RB_T] = servo_angle_offset(angles[SERVO_RB_T], CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RB_S] = servo_angle_offset(angles[SERVO_RB_S], CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RB_T] = servo_angle_limit_thigh(angles[SERVO_RB_T]);
  angles[SERVO_RB_S] = servo_angle_limit_shank(angles[SERVO_RB_S]);
  
  angles[SERVO_LB_T] = servo_angle_offset(angles[SERVO_LB_T], CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LB_S] = servo_angle_offset(angles[SERVO_LB_S], CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LB_T] = servo_angle_limit_thigh(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_limit_shank(angles[SERVO_LB_S]);
  angles[SERVO_LB_T] = servo_angle_mirror(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_mirror(angles[SERVO_LB_S]);

  servo_set_angle_sync(angles);
}

void leg_sync_set_coord2(quad_coord coords[LEG_COUNT])
{
  quad_fp angles[SERVO_COUNT] = {0};

  //! Coord offset
  coords[LEG_RF] = leg_coord_offset(LEG_RF, coords[LEG_RF]);
  coords[LEG_LF] = leg_coord_offset(LEG_LF, coords[LEG_LF]);
  coords[LEG_RB] = leg_coord_offset(LEG_RB, coords[LEG_RB]);
  coords[LEG_LB] = leg_coord_offset(LEG_LB, coords[LEG_LB]);

  //! Kinematics
  kine_inverse(&_kine[LEG_RF], coords[LEG_RF].X, coords[LEG_RF].Z);
  kine_inverse(&_kine[LEG_LF], coords[LEG_LF].X, coords[LEG_LF].Z);
  kine_inverse(&_kine[LEG_RB], coords[LEG_RB].X, coords[LEG_RB].Z);
  kine_inverse(&_kine[LEG_LB], coords[LEG_LB].X, coords[LEG_LB].Z);

  angles[SERVO_RF_T] = _kine[LEG_RF].AS1;
  angles[SERVO_RF_S] = _kine[LEG_RF].AS2;
  angles[SERVO_LF_T] = _kine[LEG_LF].AS1;
  angles[SERVO_LF_S] = _kine[LEG_LF].AS2;
  angles[SERVO_RB_T] = _kine[LEG_RB].AS1;
  angles[SERVO_RB_S] = _kine[LEG_RB].AS2;
  angles[SERVO_LB_T] = _kine[LEG_LB].AS1;
  angles[SERVO_LB_S] = _kine[LEG_LB].AS2;

  //! ACB update
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].current = angles[i];
  }
  //! CCB update
  for (leg_index i = LEG_RF; i < LEG_COUNT; i++) {
    _ccb[i].current_x = coords[i].X;
    _ccb[i].current_z = coords[i].Z;
  }
  
  //! Hardware
  angles[SERVO_RF_T] = servo_angle_offset(angles[SERVO_RF_T], CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RF_S] = servo_angle_offset(angles[SERVO_RF_S], CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RF_T] = servo_angle_limit_thigh(angles[SERVO_RF_T]);
  angles[SERVO_RF_S] = servo_angle_limit_shank(angles[SERVO_RF_S]);

  angles[SERVO_LF_T] = servo_angle_offset(angles[SERVO_LF_T], CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LF_S] = servo_angle_offset(angles[SERVO_LF_S], CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LF_T] = servo_angle_limit_thigh(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_limit_shank(angles[SERVO_LF_S]);
  angles[SERVO_LF_T] = servo_angle_mirror(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_mirror(angles[SERVO_LF_S]);

  angles[SERVO_RB_T] = servo_angle_offset(angles[SERVO_RB_T], CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RB_S] = servo_angle_offset(angles[SERVO_RB_S], CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RB_T] = servo_angle_limit_thigh(angles[SERVO_RB_T]);
  angles[SERVO_RB_S] = servo_angle_limit_shank(angles[SERVO_RB_S]);
  
  angles[SERVO_LB_T] = servo_angle_offset(angles[SERVO_LB_T], CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LB_S] = servo_angle_offset(angles[SERVO_LB_S], CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LB_T] = servo_angle_limit_thigh(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_limit_shank(angles[SERVO_LB_S]);
  angles[SERVO_LB_T] = servo_angle_mirror(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_mirror(angles[SERVO_LB_S]);

  servo_set_angle_sync(angles);
}

void leg_sync_acb_init(quad_fp (*calc)(quad_fp))
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].calc = calc;
    fcb_init(&_acb_fcb[i], CONFIG_ACB_DEFAULT_FRAME_MODE, CONFIG_ACB_DEFAULT_FRAME_INTERVAL);
  }
  fcb_init(&_sync_acb_fcb, CONFIG_ACB_DEFAULT_FRAME_MODE, CONFIG_ACB_DEFAULT_FRAME_INTERVAL);
}

void leg_sync_acb_init2(quad_fp (*calc[SERVO_COUNT])(quad_fp))
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].calc = calc[i];
    fcb_init(&_acb_fcb[i], CONFIG_ACB_DEFAULT_FRAME_MODE, CONFIG_ACB_DEFAULT_FRAME_INTERVAL);
  }
  fcb_init(&_sync_acb_fcb, CONFIG_ACB_DEFAULT_FRAME_MODE, CONFIG_ACB_DEFAULT_FRAME_INTERVAL);
}

bool leg_sync_acb_update(bool kine_update)
{
  elog_d(TAG, "start");

  bool flag_update = false;
  quad_fp p;
  quad_fp angles[SERVO_COUNT] = {0};

  //! FCB
  if(fcb_complete(&_sync_acb_fcb)) {
    elog_d(TAG, "fbc complete");
    return false;
  }
  if(fcb_skip(&_sync_acb_fcb)) {
    elog_d(TAG, "fbc skip");
    return true;
  }

  //! FCB to ACB
  if(fcb_last(&_sync_acb_fcb)) {
    elog_d(TAG, "fbc last");
    for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
      angles[i] = _acb[i].end;
    }
    flag_update = false;
  } else {
    p = fcb_percentage(&_sync_acb_fcb);
    elog_d(TAG, "fbc percentage: %f", p);
    for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
      angles[i] = _acb[i].start + _acb[i].delta * _acb[i].calc(p);
    }
    flag_update = true;
  }
  fcb_next(&_sync_acb_fcb);
  elog_d(TAG, "fbc next: %d", fcb_current(&_sync_acb_fcb));

  //! ACB
  for (uint32_t i = 0; i < SERVO_COUNT; i++) {
    _acb[i].current = angles[i];
  }

  //! ACB to Kinematics
  if(kine_update) {
    kine_forward(&_kine[LEG_RF], angles[SERVO_RF_T], angles[SERVO_RF_S]);
    kine_forward(&_kine[LEG_LF], angles[SERVO_LF_T], angles[SERVO_LF_S]);
    kine_forward(&_kine[LEG_RB], angles[SERVO_RB_T], angles[SERVO_RB_S]);
    kine_forward(&_kine[LEG_LB], angles[SERVO_LB_T], angles[SERVO_LB_S]);
  }

  //! ACB to Servo
  angles[SERVO_RF_T] = servo_angle_offset(angles[SERVO_RF_T], CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RF_S] = servo_angle_offset(angles[SERVO_RF_S], CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RF_T] = servo_angle_limit_thigh(angles[SERVO_RF_T]);
  angles[SERVO_RF_S] = servo_angle_limit_shank(angles[SERVO_RF_S]);

  angles[SERVO_LF_T] = servo_angle_offset(angles[SERVO_LF_T], CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LF_S] = servo_angle_offset(angles[SERVO_LF_S], CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LF_T] = servo_angle_limit_thigh(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_limit_shank(angles[SERVO_LF_S]);
  angles[SERVO_LF_T] = servo_angle_mirror(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_mirror(angles[SERVO_LF_S]);

  angles[SERVO_RB_T] = servo_angle_offset(angles[SERVO_RB_T], CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RB_S] = servo_angle_offset(angles[SERVO_RB_S], CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RB_T] = servo_angle_limit_thigh(angles[SERVO_RB_T]);
  angles[SERVO_RB_S] = servo_angle_limit_shank(angles[SERVO_RB_S]);
  
  angles[SERVO_LB_T] = servo_angle_offset(angles[SERVO_LB_T], CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LB_S] = servo_angle_offset(angles[SERVO_LB_S], CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LB_T] = servo_angle_limit_thigh(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_limit_shank(angles[SERVO_LB_S]);
  angles[SERVO_LB_T] = servo_angle_mirror(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_mirror(angles[SERVO_LB_S]);

  servo_set_angle_sync(angles);

  return flag_update;
}

void leg_sync_acb_absolute(quad_fp start, quad_fp end, uint32_t fcb_count)
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].start   = start;
    _acb[i].end     = end;
    _acb[i].delta   = end - start;
    _acb[i].current = start;
  }
  fcb_start(&_sync_acb_fcb, fcb_count);
}

void leg_sync_acb_absolute2(quad_fp start[SERVO_COUNT], quad_fp end[SERVO_COUNT], uint32_t fcb_count)
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].start   = start[i];
    _acb[i].end     = end[i];
    _acb[i].delta   = end[i] - start[i];
    _acb[i].current = start[i];
  }
  fcb_start(&_sync_acb_fcb, fcb_count);
}

void leg_sync_acb_relative(quad_fp delta, uint32_t fcb_count)
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].start = _acb[i].current;
    _acb[i].end   = _acb[i].start + delta;
    _acb[i].delta = delta;
  }
  fcb_start(&_sync_acb_fcb, fcb_count);
}

void leg_sync_acb_relative2(quad_fp delta[SERVO_COUNT], uint32_t fcb_count)
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].start = _acb[i].current;
    _acb[i].end   = _acb[i].start + delta[i];
    _acb[i].delta = delta[i];
  }
  fcb_start(&_sync_acb_fcb, fcb_count);
}

void leg_sync_acb_target(quad_fp target, uint32_t fcb_count)
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].start = _acb[i].current;
    _acb[i].end   = target;
    _acb[i].delta = _acb[i].end - _acb[i].current;
  }
  fcb_start(&_sync_acb_fcb, fcb_count);
}

void leg_sync_acb_target2(quad_fp target[SERVO_COUNT], uint32_t fcb_count)
{
  for (servo_index i = SERVO_START; i < SERVO_COUNT; i++) {
    _acb[i].start = _acb[i].current;
    _acb[i].end   = target[i];
    _acb[i].delta = _acb[i].end - _acb[i].current;
  }
  fcb_start(&_sync_acb_fcb, fcb_count);
}

void leg_sync_acb_update_blocking(bool kine_update)
{
  while(leg_sync_acb_update(kine_update));
}

void leg_sync_acb_absolute_blocking(quad_fp start, quad_fp end, uint32_t fcb_count)
{
  leg_sync_acb_absolute(start, end, fcb_count);
  leg_sync_acb_update_blocking(CONFIG_ACB_UPDATE_KINE);
}

void leg_sync_acb_absolute2_blocking(quad_fp start[SERVO_COUNT], quad_fp end[SERVO_COUNT], uint32_t fcb_count)
{
  leg_sync_acb_absolute2(start, end, fcb_count);
  leg_sync_acb_update_blocking(CONFIG_ACB_UPDATE_KINE);
}

void leg_sync_acb_relative_blocking(quad_fp delta, uint32_t fcb_count)
{
  leg_sync_acb_relative(delta, fcb_count);
  leg_sync_acb_update_blocking(CONFIG_ACB_UPDATE_KINE);
}

void leg_sync_acb_relative2_blocking(quad_fp delta[SERVO_COUNT], uint32_t fcb_count)
{
  leg_sync_acb_relative2(delta, fcb_count);
  leg_sync_acb_update_blocking(CONFIG_ACB_UPDATE_KINE);
}

void leg_sync_acb_target_blocking(quad_fp target, uint32_t fcb_count)
{
  leg_sync_acb_target(target, fcb_count);
  leg_sync_acb_update_blocking(CONFIG_ACB_UPDATE_KINE);
}

void leg_sync_acb_target2_blocking(quad_fp target[SERVO_COUNT], uint32_t fcb_count)
{
  leg_sync_acb_target2(target, fcb_count);
  leg_sync_acb_update_blocking(true);
}

void leg_sync_ccb_init(quad_fp (*calc_x)(quad_fp), quad_fp (*calc_z)(quad_fp))
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    memset(&_ccb[i], 0, sizeof(quad_ccb));
    _ccb[i].calc_x = calc_x;
    _ccb[i].calc_z = calc_z;
    fcb_init(&_ccb_fcb[i], CONFIG_CCB_DEFAULT_FRAME_MODE, CONFIG_CCB_DEFAULT_FRAME_INTERVAL);
  }
  fcb_init(&_sync_ccb_fcb, CONFIG_CCB_DEFAULT_FRAME_MODE, CONFIG_CCB_DEFAULT_FRAME_INTERVAL);
}

void leg_sync_ccb_init2(quad_fp (*calc_x[LEG_COUNT])(quad_fp), quad_fp (*calc_z[LEG_COUNT])(quad_fp))
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    memset(&_ccb[i], 0, sizeof(quad_ccb));
    _ccb[i].calc_x = calc_x[i];
    _ccb[i].calc_z = calc_z[i];
    fcb_init(&_ccb_fcb[i], CONFIG_CCB_DEFAULT_FRAME_MODE, CONFIG_CCB_DEFAULT_FRAME_INTERVAL);
  }
  fcb_init(&_sync_ccb_fcb, CONFIG_CCB_DEFAULT_FRAME_MODE, CONFIG_CCB_DEFAULT_FRAME_INTERVAL);
}

bool leg_sync_ccb_update(void)
{
  bool flag_update = false;
  quad_fp p;
  quad_coord coords[LEG_COUNT] = {0};
  quad_fp angles[SERVO_COUNT] = {0};

  //! FCB
  if(fcb_complete(&_sync_ccb_fcb)) {
    elog_d(TAG, "fbc complete");
    return false;
  }
  if(fcb_skip(&_sync_ccb_fcb)) {
    elog_d(TAG, "fbc skip");
    return true;
  }

  //! FCB to CCB
  if(fcb_last(&_sync_ccb_fcb)) {
    elog_d(TAG, "fbc last");
    for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
      coords[i].X = _ccb[i].end_x;
      coords[i].Z = _ccb[i].end_z;
      elog_d(TAG, "ccb end: %f, %f", coords[i].X, coords[i].Z);
    }
    flag_update = false;
  } else {
    p = fcb_percentage(&_sync_ccb_fcb);
    elog_d(TAG, "fbc percentage: %f", p);
    for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
      coords[i].X = _ccb[i].start_x + _ccb[i].delta_x * _ccb[i].calc_x(p);
      coords[i].Z = _ccb[i].start_z + _ccb[i].delta_z * _ccb[i].calc_z(p);
      elog_d(TAG, "ccb calc: %f, %f", coords[i].X, coords[i].Z);
    }
    flag_update = true;
  }
  elog_d(TAG, "fbc next: %d", fcb_current(&_sync_ccb_fcb));
  fcb_next(&_sync_ccb_fcb);

  //! CCB
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].current_x = coords[i].X;
    _ccb[i].current_z = coords[i].Z;
    elog_d(TAG, "ccb current: %f, %f", _ccb[i].current_x, _ccb[i].current_z);
  }

  //! Coord Offset
  coords[LEG_RF] = leg_coord_offset(LEG_RF, coords[LEG_RF]);
  coords[LEG_LF] = leg_coord_offset(LEG_LF, coords[LEG_LF]);
  coords[LEG_RB] = leg_coord_offset(LEG_RB, coords[LEG_RB]);
  coords[LEG_LB] = leg_coord_offset(LEG_LB, coords[LEG_LB]);

  //! CCB to Kinematics
  kine_inverse(&_kine[LEG_RF], coords[LEG_RF].X, coords[LEG_RF].Z);
  kine_inverse(&_kine[LEG_LF], coords[LEG_LF].X, coords[LEG_LF].Z);
  kine_inverse(&_kine[LEG_RB], coords[LEG_RB].X, coords[LEG_RB].Z);
  kine_inverse(&_kine[LEG_LB], coords[LEG_LB].X, coords[LEG_LB].Z);

  angles[SERVO_RF_T] = _kine[LEG_RF].AS1;
  angles[SERVO_RF_S] = _kine[LEG_RF].AS2;
  angles[SERVO_LF_T] = _kine[LEG_LF].AS1;
  angles[SERVO_LF_S] = _kine[LEG_LF].AS2;
  angles[SERVO_RB_T] = _kine[LEG_RB].AS1;
  angles[SERVO_RB_S] = _kine[LEG_RB].AS2;
  angles[SERVO_LB_T] = _kine[LEG_LB].AS1;
  angles[SERVO_LB_S] = _kine[LEG_LB].AS2;

  //! CCB to Servo
  angles[SERVO_RF_T] = servo_angle_offset(angles[SERVO_RF_T], CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RF_S] = servo_angle_offset(angles[SERVO_RF_S], CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RF_T] = servo_angle_limit_thigh(angles[SERVO_RF_T]);
  angles[SERVO_RF_S] = servo_angle_limit_shank(angles[SERVO_RF_S]);

  angles[SERVO_LF_T] = servo_angle_offset(angles[SERVO_LF_T], CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LF_S] = servo_angle_offset(angles[SERVO_LF_S], CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LF_T] = servo_angle_limit_thigh(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_limit_shank(angles[SERVO_LF_S]);
  angles[SERVO_LF_T] = servo_angle_mirror(angles[SERVO_LF_T]);
  angles[SERVO_LF_S] = servo_angle_mirror(angles[SERVO_LF_S]);

  angles[SERVO_RB_T] = servo_angle_offset(angles[SERVO_RB_T], CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_RB_S] = servo_angle_offset(angles[SERVO_RB_S], CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_RB_T] = servo_angle_limit_thigh(angles[SERVO_RB_T]);
  angles[SERVO_RB_S] = servo_angle_limit_shank(angles[SERVO_RB_S]);

  angles[SERVO_LB_T] = servo_angle_offset(angles[SERVO_LB_T], CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
  angles[SERVO_LB_S] = servo_angle_offset(angles[SERVO_LB_S], CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
  angles[SERVO_LB_T] = servo_angle_limit_thigh(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_limit_shank(angles[SERVO_LB_S]);
  angles[SERVO_LB_T] = servo_angle_mirror(angles[SERVO_LB_T]);
  angles[SERVO_LB_S] = servo_angle_mirror(angles[SERVO_LB_S]);

  servo_set_angle_sync(angles);
  return flag_update;
}

void leg_sync_ccb_update_blocking(void)
{
  while(leg_sync_ccb_update());
}

void leg_sync_ccb_absolute(quad_coord start, quad_coord end, uint32_t fcb_count)
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].start_x = start.X;
    _ccb[i].start_z = start.Z;
    _ccb[i].end_x   = end.X;
    _ccb[i].end_z   = end.Z;
    _ccb[i].delta_x = end.X - start.X;
    _ccb[i].delta_z = end.Z - start.Z;
  }
  fcb_start(&_sync_ccb_fcb, fcb_count);
}

void leg_sync_ccb_absolute2(quad_coord start[LEG_COUNT], quad_coord end[LEG_COUNT], uint32_t fcb_count)
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].start_x = start[i].X;
    _ccb[i].start_z = start[i].Z;
    _ccb[i].end_x   = end[i].X;
    _ccb[i].end_z   = end[i].Z;
    _ccb[i].delta_x = end[i].X - start[i].X;
    _ccb[i].delta_z = end[i].Z - start[i].Z;
  }
  fcb_start(&_sync_ccb_fcb, fcb_count);
}

void leg_sync_ccb_relative(quad_coord delta, uint32_t fcb_count)
{
  elog_d(TAG, "delta: %f, %f", delta.X, delta.Z);
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].start_x = _ccb[i].current_x;
    _ccb[i].start_z = _ccb[i].current_z;
    _ccb[i].end_x   = _ccb[i].start_x + delta.X;
    _ccb[i].end_z   = _ccb[i].start_z + delta.Z;
    _ccb[i].delta_x = delta.X;
    _ccb[i].delta_z = delta.Z;
    elog_d(TAG, "current: %f, %f", _ccb[i].current_x, _ccb[i].current_z);
    elog_d(TAG, "end: %f, %f", _ccb[i].end_x, _ccb[i].end_z);
    elog_d(TAG, "delta: %f, %f", _ccb[i].delta_x, _ccb[i].delta_z);
  }
  fcb_start(&_sync_ccb_fcb, fcb_count);
}

void leg_sync_ccb_relative2(quad_coord delta[LEG_COUNT], uint32_t fcb_count)
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].start_x = _ccb[i].current_x;
    _ccb[i].start_z = _ccb[i].current_z;
    _ccb[i].end_x   = _ccb[i].start_x + delta[i].X;
    _ccb[i].end_z   = _ccb[i].start_z + delta[i].Z;
    _ccb[i].delta_x = delta[i].X;
    _ccb[i].delta_z = delta[i].Z;
  }
  fcb_start(&_sync_ccb_fcb, fcb_count);
}

void leg_sync_ccb_target(quad_coord target, uint32_t fcb_count)
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].start_x = _ccb[i].current_x;
    _ccb[i].start_z = _ccb[i].current_z;
    _ccb[i].end_x   = target.X;
    _ccb[i].end_z   = target.Z;
    _ccb[i].delta_x = target.X - _ccb[i].current_x;
    _ccb[i].delta_z = target.Z - _ccb[i].current_z;
  }
  fcb_start(&_sync_ccb_fcb, fcb_count);
}

void leg_sync_ccb_target2(quad_coord target[LEG_COUNT], uint32_t fcb_count)
{
  for (leg_index i = LEG_START; i < LEG_COUNT; i++) {
    _ccb[i].start_x = _ccb[i].current_x;
    _ccb[i].start_z = _ccb[i].current_z;
    _ccb[i].end_x   = target[i].X;
    _ccb[i].end_z   = target[i].Z;
    _ccb[i].delta_x = target[i].X - _ccb[i].current_x;
    _ccb[i].delta_z = target[i].Z - _ccb[i].current_z;
  }
  fcb_start(&_sync_ccb_fcb, fcb_count);
}

void leg_sync_ccb_absolute_blocking(quad_coord start, quad_coord end, uint32_t fcb_count)
{
  leg_sync_ccb_absolute(start, end, fcb_count);
  leg_sync_ccb_update_blocking();
}

void leg_sync_ccb_absolute2_blocking(quad_coord start[LEG_COUNT], quad_coord end[LEG_COUNT], uint32_t fcb_count)
{
  leg_sync_ccb_absolute2(start, end, fcb_count);
  leg_sync_ccb_update_blocking();
}

void leg_sync_ccb_relative_blocking(quad_coord delta, uint32_t fcb_count)
{
  leg_sync_ccb_relative(delta, fcb_count);
  leg_sync_ccb_update_blocking();
}

void leg_sync_ccb_relative2_blocking(quad_coord delta[LEG_COUNT], uint32_t fcb_count)
{
  leg_sync_ccb_relative2(delta, fcb_count);
  leg_sync_ccb_update_blocking();
}

void leg_sync_ccb_target_blocking(quad_coord target, uint32_t fcb_count)
{
  leg_sync_ccb_target(target, fcb_count);
  leg_sync_ccb_update_blocking();
}

void leg_sync_ccb_target2_blocking(quad_coord target[LEG_COUNT], uint32_t fcb_count)
{
  leg_sync_ccb_target2(target, fcb_count);
  leg_sync_ccb_update_blocking();
}

