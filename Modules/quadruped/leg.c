#include "quadruped.h"

#define TAG "Leg"

inline quad_coord leg_coord_offset(leg_index index ,quad_coord c)
{
  switch(index) {
    case LEG_RF:
      c.X += (CONFIG_LEG_COORD_OFFSET_X_RF+CONFIG_LEG_COORD_OFFSET_X_GLOBAL);
      c.Z += (CONFIG_LEG_COORD_OFFSET_Z_RF+CONFIG_LEG_COORD_OFFSET_Z_GLOBAL);
      break;
    case LEG_LF:
      c.X += (CONFIG_LEG_COORD_OFFSET_X_LF+CONFIG_LEG_COORD_OFFSET_X_GLOBAL);
      c.Z += (CONFIG_LEG_COORD_OFFSET_Z_LF+CONFIG_LEG_COORD_OFFSET_Z_GLOBAL);
      break;
    case LEG_RB:
      c.X += (CONFIG_LEG_COORD_OFFSET_X_RB+CONFIG_LEG_COORD_OFFSET_X_GLOBAL);
      c.Z += (CONFIG_LEG_COORD_OFFSET_Z_RB+CONFIG_LEG_COORD_OFFSET_Z_GLOBAL);
      break;
    case LEG_LB:
      c.X += (CONFIG_LEG_COORD_OFFSET_X_LB+CONFIG_LEG_COORD_OFFSET_X_GLOBAL);
      c.Z += (CONFIG_LEG_COORD_OFFSET_Z_LB+CONFIG_LEG_COORD_OFFSET_Z_GLOBAL);
      break;
  }
  return c;
}

void leg_set_angle(leg_index leg, quad_fp angle_thigh, quad_fp angle_shank, bool kine_update)
{
  //! Parameter Check
  if(leg > LEG_END) {
    elog_e(TAG, "Invalid leg index");
    return;
  }

  //! Kinematics
  if(kine_update) {
    kine_forward(&_kine[leg], angle_thigh, angle_shank);
  }

  //! Hardware
  switch(leg) {
    case LEG_RF:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      servo_set_angle(SERVO_RF_T, angle_thigh);
      servo_set_angle(SERVO_RF_S, angle_shank);
      break;
    case LEG_LF:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      angle_thigh = servo_angle_mirror(angle_thigh);
      angle_shank = servo_angle_mirror(angle_shank);
      servo_set_angle(SERVO_LF_T, angle_thigh);
      servo_set_angle(SERVO_LF_S, angle_shank);
      break;
    case LEG_RB:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      servo_set_angle(SERVO_RB_T, angle_thigh);
      servo_set_angle(SERVO_RB_S, angle_shank);
      break;
    case LEG_LB:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      angle_thigh = servo_angle_mirror(angle_thigh);
      angle_shank = servo_angle_mirror(angle_shank);
      servo_set_angle(SERVO_LB_T, angle_thigh);
      servo_set_angle(SERVO_LB_S, angle_shank);
      break;
  }
}

void leg_set_coord(leg_index leg, quad_coord coord)
{
  quad_fp angle_thigh, angle_shank;

  //! Parameter Check
  if(leg > LEG_END) {
    elog_e(TAG, "Invalid leg index");
    return;
  }

  //! Coord offset
  coord = leg_coord_offset(leg, coord);

  //! Kinematics
  kine_inverse(&_kine[leg], coord.X, coord.Z);
  angle_thigh = _kine[leg].AS1;
  angle_shank = _kine[leg].AS2;

  //! Hardware
  switch(leg) {
    case LEG_RF:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      servo_set_angle(SERVO_RF_T, angle_thigh);
      servo_set_angle(SERVO_RF_S, angle_shank);
      break;
    case LEG_LF:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      angle_thigh = servo_angle_mirror(angle_thigh);
      angle_shank = servo_angle_mirror(angle_shank);
      servo_set_angle(SERVO_LF_T, angle_thigh);
      servo_set_angle(SERVO_LF_S, angle_shank);
      break;
    case LEG_RB:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      servo_set_angle(SERVO_RB_T, angle_thigh);
      servo_set_angle(SERVO_RB_S, angle_shank);
      break;
    case LEG_LB:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      angle_thigh = servo_angle_mirror(angle_thigh);
      angle_shank = servo_angle_mirror(angle_shank);
      servo_set_angle(SERVO_LB_T, angle_thigh);
      servo_set_angle(SERVO_LB_S, angle_shank);
      break;
  }
}

void leg_acb_init(servo_index index, quad_fp (*calc)(quad_fp))
{
  _acb[index].calc = calc;
  fcb_init(&_acb_fcb[index], CONFIG_ACB_DEFAULT_FRAME_MODE, CONFIG_ACB_DEFAULT_FRAME_INTERVAL);
}

bool leg_acb_update(servo_index index, bool kine_update)
{
  bool flag = false;
  quad_fp p;
  quad_fp angle;
  
  //! FCB
  if (fcb_complete(&_acb_fcb[index])) {
    return false;
  }
  if( fcb_skip(&_acb_fcb[index]) ) {
    return true;
  }
  //! FCB to ACB
  if(fcb_last(&_acb_fcb[index])) {
    angle = _acb[index].end;
    flag = false;
  } else {
    p = fcb_percentage(&_acb_fcb[index]);
    angle = _acb[index].start + _acb[index].delta * _acb->calc(p);
    flag = true;
  }
  fcb_next(&_acb_fcb[index]);
  //! ACB
  _acb[index].current = angle;
  //! ACB to Kinematics
  if(kine_update) {
    switch(index) {
      case SERVO_RF_T:
        kine_forward(&_kine[LEG_RF], angle, _kine[LEG_RF].AS2);
        break;
      case SERVO_RF_S:
        kine_forward(&_kine[LEG_RF], _kine[LEG_RF].AS1, angle);
        break;
      case SERVO_LF_T:
        kine_forward(&_kine[LEG_LF], angle, _kine[LEG_LF].AS2);
        break;
      case SERVO_LF_S:
        kine_forward(&_kine[LEG_LF], _kine[LEG_LF].AS1, angle);
        break;
      case SERVO_RB_T:
        kine_forward(&_kine[LEG_RB], angle, _kine[LEG_RB].AS2);
        break;
      case SERVO_RB_S:
        kine_forward(&_kine[LEG_RB], _kine[LEG_RB].AS1, angle);
        break;
      case SERVO_LB_T:
        kine_forward(&_kine[LEG_LB], angle, _kine[LEG_LB].AS2);
        break;
      case SERVO_LB_S:
        kine_forward(&_kine[LEG_LB], _kine[LEG_LB].AS1, angle);
        break;
    }
  }
  //! ACB to Servo
  switch(index) {
    case SERVO_RF_T:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle = servo_angle_limit_thigh(angle);
      servo_set_angle(SERVO_RF_T, angle);
      break;
    case SERVO_RF_S:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle = servo_angle_limit_shank(angle);
      servo_set_angle(SERVO_RF_S, angle);
      break;
    case SERVO_LF_T:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle = servo_angle_limit_thigh(angle);
      angle = servo_angle_mirror(angle);
      servo_set_angle(SERVO_LF_T, angle);
      break;
    case SERVO_LF_S:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle = servo_angle_limit_shank(angle);
      angle = servo_angle_mirror(angle);
      servo_set_angle(SERVO_LF_S, angle);
      break;
    case SERVO_RB_T:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle = servo_angle_limit_thigh(angle);
      servo_set_angle(SERVO_RB_T, angle);
      break;
    case SERVO_RB_S:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle = servo_angle_limit_shank(angle);
      servo_set_angle(SERVO_RB_S, angle);
      break;
    case SERVO_LB_T:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle = servo_angle_limit_thigh(angle);
      angle = servo_angle_mirror(angle);
      servo_set_angle(SERVO_LB_T, angle);
      break;
    case SERVO_LB_S:
      angle = servo_angle_offset(angle, CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle = servo_angle_limit_shank(angle);
      angle = servo_angle_mirror(angle);
      servo_set_angle(SERVO_LB_S, angle);
      break;
  }
  return flag;
}

void leg_acb_absolute(servo_index index, quad_fp start, quad_fp end, uint32_t fcb_count)
{
  _acb[index].start   = start;
  _acb[index].end     = end;
  _acb[index].delta   = end - start;
  _acb[index].current = start;
  fcb_start(&_acb_fcb[index], fcb_count);
}

void leg_acb_relative(servo_index index, quad_fp delta, uint32_t fcb_count)
{
  _acb[index].start = _acb[index].current;
  _acb[index].end   = _acb[index].start + delta;
  _acb[index].delta = delta;
  fcb_start(&_acb_fcb[index], fcb_count);
}

void leg_acb_target(servo_index index, quad_fp target, uint32_t fcb_count)
{
  _acb[index].start = _acb[index].current;
  _acb[index].end   = target;
  _acb[index].delta = _acb[index].end - _acb[index].current;
  fcb_start(&_acb_fcb[index], fcb_count);
}

void leg_acb_update_blocking(servo_index index, bool kine_update)
{
  while(leg_acb_update(index, kine_update));
}

void leg_acb_absolute_blocking(servo_index index, quad_fp start, quad_fp end, uint32_t fcb_count)
{
  leg_acb_absolute(index, start, end, fcb_count);
  leg_acb_update_blocking(index, CONFIG_ACB_UPDATE_KINE);
}

void leg_acb_relative_blocking(servo_index index, quad_fp delta, uint32_t fcb_count)
{
  leg_acb_relative(index, delta, fcb_count);
  leg_acb_update_blocking(index, CONFIG_ACB_UPDATE_KINE);
}

void leg_acb_target_blocking(servo_index index, quad_fp target, uint32_t fcb_count)
{
  leg_acb_target(index, target, fcb_count);
  leg_acb_update_blocking(index, CONFIG_ACB_UPDATE_KINE);
}

void leg_ccb_init(leg_index index, quad_fp (*calc_x)(quad_fp), quad_fp (*calc_z)(quad_fp))
{
  memset(&_ccb[index], 0, sizeof(quad_ccb));
  _ccb[index].calc_x = calc_x;
  _ccb[index].calc_z = calc_z;
  fcb_init(&_ccb_fcb[index], CONFIG_CCB_DEFAULT_FRAME_MODE, CONFIG_CCB_DEFAULT_FRAME_INTERVAL);
}

bool leg_ccb_update(leg_index index)
{
  bool flag = false;
  quad_fp p;
  quad_coord coord;
  quad_fp angle_thigh, angle_shank;

  //! FCB
  if (fcb_complete(&_ccb_fcb[index])) {
    return false;
  }
  if( fcb_skip(&_ccb_fcb[index]) ) {
    return true;
  }
  //! FCB to CCB
  if(fcb_last(&_ccb_fcb[index])) {
    coord.X = _ccb[index].end_x;
    coord.Z = _ccb[index].end_z;
    flag = false;
  } else {
    p = fcb_percentage(&_ccb_fcb[index]);
    coord.X = _ccb[index].start_x + _ccb[index].delta_x * _ccb->calc_x(p);
    coord.Z = _ccb[index].start_z + _ccb[index].delta_z * _ccb->calc_z(p);
    flag = true;
  }
  fcb_next(&_ccb_fcb[index]);
  
  //! Coord Offset
  coord = leg_coord_offset(index, coord);

  //! CCB
  _ccb[index].current_x = coord.X;
  _ccb[index].current_z = coord.Z;

  //! CCB to Kinematics
  kine_inverse(&_kine[index], coord.X, coord.Z);
  angle_thigh = _kine[index].AS1;
  angle_shank = _kine[index].AS2;

  //! Kinematics to Servo
  switch(index) {
    case LEG_RF:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      servo_set_angle(SERVO_RF_T, angle_thigh);
      servo_set_angle(SERVO_RF_S, angle_shank);
      break;
    case LEG_LF:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LF_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LF_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      angle_thigh = servo_angle_mirror(angle_thigh);
      angle_shank = servo_angle_mirror(angle_shank);
      servo_set_angle(SERVO_LF_T, angle_thigh);
      servo_set_angle(SERVO_LF_S, angle_shank);
      break;
    case LEG_RB:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_RB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_RB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      servo_set_angle(SERVO_RB_T, angle_thigh);
      servo_set_angle(SERVO_RB_S, angle_shank);
      break;
    case LEG_LB:
      angle_thigh = servo_angle_offset(angle_thigh, CONFIG_SERVO_OFFSET_LB_T+CONFIG_SERVO_OFFSET_GLOBAL_T);
      angle_shank = servo_angle_offset(angle_shank, CONFIG_SERVO_OFFSET_LB_S+CONFIG_SERVO_OFFSET_GLOBAL_S);
      angle_thigh = servo_angle_limit_thigh(angle_thigh);
      angle_shank = servo_angle_limit_shank(angle_shank);
      angle_thigh = servo_angle_mirror(angle_thigh);
      angle_shank = servo_angle_mirror(angle_shank);
      servo_set_angle(SERVO_LB_T, angle_thigh);
      servo_set_angle(SERVO_LB_S, angle_shank);
      break;
  }
  return flag;
}

void leg_ccb_absolute(leg_index index, quad_coord start, quad_coord end, uint32_t fcb_count)
{
  _ccb[index].start_x = start.X;
  _ccb[index].start_z = start.Z;
  _ccb[index].end_x   = end.X;
  _ccb[index].end_z   = end.Z;
  _ccb[index].delta_x = end.X - start.X;
  _ccb[index].delta_z = end.Z - start.Z;
  fcb_start(&_ccb_fcb[index], fcb_count);
}

void leg_ccb_relative(leg_index index, quad_coord delta, uint32_t fcb_count)
{
  _ccb[index].start_x = _ccb[index].current_x;
  _ccb[index].start_z = _ccb[index].current_z;
  _ccb[index].end_x   = _ccb[index].start_x + delta.X;
  _ccb[index].end_z   = _ccb[index].start_z + delta.Z;
  _ccb[index].delta_x = delta.X;
  _ccb[index].delta_z = delta.Z;
  fcb_start(&_ccb_fcb[index], fcb_count);
}

void leg_ccb_target(leg_index index, quad_coord target, uint32_t fcb_count)
{
  _ccb[index].start_x = _ccb[index].current_x;
  _ccb[index].start_z = _ccb[index].current_z;
  _ccb[index].end_x   = target.X;
  _ccb[index].end_z   = target.Z;
  _ccb[index].delta_x = target.X - _ccb[index].current_x;
  _ccb[index].delta_z = target.Z - _ccb[index].current_z;
  fcb_start(&_ccb_fcb[index], fcb_count);
}

void leg_ccb_update_blocking(leg_index index)
{
  while (leg_ccb_update(index));
}

void leg_ccb_absolute_blocking(leg_index index, quad_coord start, quad_coord end, uint32_t fcb_count)
{
  leg_ccb_absolute(index, start, end, fcb_count);
  leg_ccb_update_blocking(index);
}

void leg_ccb_relative_blocking(leg_index index, quad_coord delta, uint32_t fcb_count)
{
  leg_ccb_relative(index, delta, fcb_count);
  leg_ccb_update_blocking(index);
}

void leg_ccb_target_blocking(leg_index index, quad_coord target, uint32_t fcb_count)
{
  leg_ccb_target(index, target, fcb_count);
  leg_ccb_update_blocking(index);
}