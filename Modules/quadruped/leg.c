#include "quadruped.h"

#define TAG "Leg"

quad_leg leg_rf = {.type=LEG_RF};
quad_leg leg_lf  = {.type=LEG_LF};
quad_leg leg_rb  = {.type=LEG_RB};
quad_leg leg_lb   = {.type=LEG_LB};

static inline const char* get_leg_str(quad_types type)
{
  switch(type) {
    case LEG_RF : return "RF";
    case LEG_LF : return "LF";
    case LEG_RB : return "RB";
    case LEG_LB : return "LB";
         default: return "Unknown";
  }
}

static inline quad_fp angle_mirror(quad_fp angle)
{
  return 180.f - angle;
}

static inline quad_fp angle_offset(quad_fp angle, quad_fp offset)
{
  return angle + offset;
}

static inline quad_fp angle_limit_thigh(quad_fp angle)
{
  if(angle > CONFIG_THIGH_SERVO_ANGLE_MAX) {
    return CONFIG_THIGH_SERVO_ANGLE_MAX;
  } else if(angle < CONFIG_THIGH_SERVO_ANGLE_MIN) {
    return CONFIG_THIGH_SERVO_ANGLE_MIN;
  }
  return angle;
}

static inline quad_fp angle_limit_shank(quad_fp angle)
{
  if(angle > CONFIG_SHANK_SERVO_ANGLE_MAX) {
    return CONFIG_SHANK_SERVO_ANGLE_MAX;
  } else if(angle < CONFIG_SHANK_SERVO_ANGLE_MIN) {
    return CONFIG_SHANK_SERVO_ANGLE_MIN;
  }
  return angle;
}

void leg_init(quad_leg* leg, quad_leg_cfg cfg)
{
  leg->is_init = 1;

  leg->type                = cfg.type;

  leg->thigh_servo.channel = cfg.thigh_servo_cfg.channel;
  leg->thigh_servo.offset  = cfg.thigh_servo_cfg.offset;

  leg->shank_servo.channel = cfg.shank_servo_cfg.channel;
  leg->shank_servo.offset  = cfg.shank_servo_cfg.offset;

  leg->tacb.type           = ACB_THIGH;
  leg->tacb.calc           = cfg.tacb_cfg->calc;
  leg->tacb.frame.interval = cfg.tacb_cfg->frame_cfg.interval;

  leg->sacb.type           = ACB_SHANK;
  leg->sacb.calc           = cfg.sacb_cfg->calc;
  leg->sacb.frame.interval = cfg.sacb_cfg->frame_cfg.interval;

  leg->ccb.calc_x          = cfg.ccb_cfg->calc_x;
  leg->ccb.calc_z          = cfg.ccb_cfg->calc_z;
  leg->ccb.frame.interval  = cfg.ccb_cfg->frame_cfg.interval;
}

void led_set_angle(quad_leg* leg, quad_fp thigh_angle, quad_fp shank_angle, bool fowward)
{
  if(fowward) 
  {
    kine_forward(&leg->kine, thigh_angle, shank_angle);
  }
  else
  {
    leg->kine.AS1 = thigh_angle;
    leg->kine.AS2 = shank_angle;
  }

  thigh_angle = angle_offset(thigh_angle, leg->thigh_servo.offset);
  shank_angle = angle_offset(shank_angle, leg->shank_servo.offset);

  elog_v(TAG, "leg_set_angle: %s after offset, thigh_angle: %5.3f shank_angle: %5.3f", 
    get_leg_str(leg->type), thigh_angle, shank_angle);

  thigh_angle = angle_limit_thigh(thigh_angle);
  shank_angle = angle_limit_shank(shank_angle);

  if(leg->type == LEG_LF || leg->type == LEG_LB)
  {
    thigh_angle = angle_mirror(thigh_angle);
    shank_angle = angle_mirror(shank_angle);
  }

  servo_set_angle(&leg->thigh_servo, thigh_angle);
  servo_set_angle(&leg->shank_servo, shank_angle);
}

void leg_set_coord(quad_leg* leg, quad_fp X, quad_fp Z)
{
  switch(leg->type) {
    case LEG_RF:
      X += CONFIG_X_OFFSET_RF;
      Z += CONFIG_Z_OFFSET_RF;
      break;
    case LEG_LF:
      X += CONFIG_X_OFFSET_LF;
      Z += CONFIG_Z_OFFSET_LF;
      break;
    case LEG_RB:
      X += CONFIG_X_OFFSET_RB;
      Z += CONFIG_Z_OFFSET_RB;
      break;
    case LEG_LB:
      X += CONFIG_X_OFFSET_LB;
      Z += CONFIG_Z_OFFSET_LB;
      break;
    default:
      elog_e(TAG, "unknown leg type");
      return;
  }

  kine_inverse(&leg->kine, X, Z);
  
  quad_fp thigh_angle = leg->kine.AS1;
  quad_fp shank_angle = leg->kine.AS2;

  thigh_angle = angle_offset(thigh_angle, leg->thigh_servo.offset);
  shank_angle = angle_offset(shank_angle, leg->shank_servo.offset);

  elog_v(TAG, "leg_set_coord: %s after offset, thigh_angle: %5.3f shank_angle: %5.3f", 
    get_leg_str(leg->type), thigh_angle, shank_angle);

  thigh_angle = angle_limit_thigh(thigh_angle);
  shank_angle = angle_limit_shank(shank_angle);

  if(leg->type == LEG_LF || leg->type == LEG_LB)
  {
    thigh_angle = angle_mirror(thigh_angle);
    shank_angle = angle_mirror(shank_angle);
  }

  servo_set_angle(&leg->thigh_servo, thigh_angle);
  servo_set_angle(&leg->shank_servo, shank_angle);
}



void leg_acb_abslute(quad_leg* leg, quad_acb* acb, quad_fp start, quad_fp end, uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }

  if(start == end)
  {
    acb->frame.count = 0;
    elog_w(TAG, "start == end");
    return;
  }

  volatile quad_fp* current_ptr;

  if(acb->type == ACB_THIGH)
  {
    current_ptr = &leg->kine.AS1;
  }
  else if(acb->type == ACB_SHANK)
  {
    current_ptr = &leg->kine.AS2;
  }
  else
  {
    elog_e(TAG, "unknown acb type");
    return;
  }
  
  *current_ptr = start;

  acb->start = start;
  acb->end   = end;
  acb->delta = end - start;

  acb->frame.count        = frame_count;
  acb->frame.index        = 0;
  acb->frame.elapsed_tick = 0;
  acb->frame.last_tick    = 0;
}

void leg_acb_relative(quad_leg* leg, quad_acb* acb, quad_fp delta, uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }

  if(delta == 0)
  {
    acb->frame.count = 0;
    elog_w(TAG, "delta == 0");
    return;
  }

  volatile quad_fp* current_ptr;

  if(acb->type == ACB_THIGH)
  {
    current_ptr = &leg->kine.AS1;
  }
  else if(acb->type == ACB_SHANK)
  {
    current_ptr = &leg->kine.AS2;
  }
  else
  {
    elog_e(TAG, "unknown acb type");
    return;
  }

  acb->start = *current_ptr;
  acb->end   = *current_ptr + delta;
  acb->delta = delta;

  acb->frame.count        = frame_count;
  acb->frame.index        = 0;
  acb->frame.elapsed_tick = 0;
  acb->frame.last_tick    = 0;
}

void leg_acb_target(quad_leg* leg, quad_acb* acb, quad_fp target, uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }

  volatile quad_fp* current_ptr;

  if(acb->type == ACB_THIGH)
  {
    current_ptr = &leg->kine.AS1;
  }
  else if(acb->type == ACB_SHANK)
  {
    current_ptr = &leg->kine.AS2;
  }
  else
  {
    elog_e(TAG, "unknown acb type");
    return;
  }

  if(target == *current_ptr)
  {
    acb->frame.count = 0;
    elog_w(TAG, "target == current");
    return;
  }

  acb->start = *current_ptr;
  acb->end   = target;
  acb->delta = target - *current_ptr;

  acb->frame.count        = frame_count;
  acb->frame.index        = 0;
  acb->frame.elapsed_tick = 0;
  acb->frame.last_tick    = 0;
}

bool leg_acb_update(quad_leg* leg, quad_acb* acb)
{
  bool flag = false;
  quad_fp t = 0;
  volatile quad_fp* ASX_ptr;
  quad_servo* servo_ptr;
  quad_fp angle;

  //! Angle Control Block
  if(acb->frame.count == 0) { return false; }
  if(acb->type == ACB_THIGH) {
    //控制大腿舵机
    ASX_ptr = &leg->kine.AS1;
    servo_ptr = &leg->thigh_servo;
  } else if(acb->type == ACB_SHANK) {
    //控制小腿舵机
    ASX_ptr = &leg->kine.AS2;
    servo_ptr = &leg->shank_servo;
  } else {
    elog_e(TAG, "unknown acb type");
    return false;
  }
  if(acb->frame.index == acb->frame.count) {
    //最后一帧
    t = 1.0f;
    angle = acb->end;
    flag = false;
  } else {
    //非最后一帧
    t = ((quad_fp)(acb->frame.index))/((quad_fp)(acb->frame.count));
    angle = acb->start + acb->delta * acb->calc(t);
    flag = true;
  }
  //更新kine数据
  *ASX_ptr = angle;

  //! Hardware
  angle = angle_offset(angle, servo_ptr->offset);
  if(acb->type==ACB_THIGH) {
    angle = angle_limit_thigh(angle);
  } else {
    angle = angle_limit_shank(angle);
  }
  if(leg->type==LEG_LF || leg->type==LEG_LB) {
    angle = angle_mirror(angle);
  }
  servo_set_angle(servo_ptr, angle);

  //! Angle Control Block
  ++acb->frame.index;
  return flag;
}

void leg_acb_update_block(quad_leg* leg, quad_acb* acb)
{
  bool flag = false;
  volatile uint32_t last_tick = 0;
  do {
    if( HAL_GetTick()-last_tick > CONFIG_ACB_FRAME_INTERVAL0) {
      flag = leg_acb_update(leg, acb);
      last_tick = HAL_GetTick();
    } else {
      flag = true;
    }
  } while(flag);
}

bool leg_acb_update_all()
{
  bool flag = false;

  flag |= leg_acb_update(&leg_rf,&leg_rf.tacb);
  flag |= leg_acb_update(&leg_rf,&leg_rf.sacb);

  flag |= leg_acb_update(&leg_lf,&leg_lf.tacb);
  flag |= leg_acb_update(&leg_lf,&leg_lf.sacb);

  flag |= leg_acb_update(&leg_rb,&leg_rb.tacb);
  flag |= leg_acb_update(&leg_rb,&leg_rb.sacb);

  flag |= leg_acb_update(&leg_lb,&leg_lb.tacb);
  flag |= leg_acb_update(&leg_lb,&leg_lb.sacb);

  return flag;
}

void leg_acb_update_all_block()
{
  bool flag = false;
  volatile uint32_t last_tick = 0;
  do {
    if( HAL_GetTick()-last_tick > CONFIG_ACB_FRAME_INTERVAL0) {
      flag = leg_acb_update_all();
      last_tick = HAL_GetTick();
    } else {
      flag = true;
    }
  } while(flag);
}

void leg_ccb_abslute(quad_leg* leg, quad_ccb* ccb, 
  quad_coord start, quad_coord end, uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }

  if(start.X == end.X && start.Z == end.Z)
  {
    ccb->frame.count = 0;
    elog_w(TAG, "start == end");
    return;
  }

  ccb->start_x = start.X;
  ccb->start_z = start.Z;
  ccb->end_x   = end.X;
  ccb->end_z   = end.Z;

  ccb->delta_x = end.X - start.X;
  ccb->delta_z = end.Z - start.Z;

  ccb->frame.count        = frame_count;
  ccb->frame.index        = 0;
  ccb->frame.elapsed_tick = 0;
  ccb->frame.last_tick    = 0;
}

void leg_ccb_relative(quad_leg* leg, quad_ccb* ccb,
  quad_coord delta, uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }

  if(delta.X == 0 && delta.Z == 0)
  {
    ccb->frame.count = 0;
    elog_w(TAG, "delta == 0");
    return;
  }

  ccb->start_x = leg->kine.X;
  ccb->start_z = leg->kine.Z;
  ccb->end_x   = leg->kine.X + delta.X;
  ccb->end_z   = leg->kine.Z + delta.Z;

  ccb->delta_x = delta.X;
  ccb->delta_z = delta.Z;

  ccb->frame.count        = frame_count;
  ccb->frame.index        = 0;
  ccb->frame.elapsed_tick = 0;
  ccb->frame.last_tick    = 0;
}

void leg_ccb_target(quad_leg* leg, quad_ccb* ccb,
  quad_coord target, uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }

  if(target.X == leg->kine.X && target.Z == leg->kine.Z)
  {
    ccb->frame.count = 0;
    elog_w(TAG, "target == current");
    return;
  }

  ccb->start_x = leg->kine.X;
  ccb->start_z = leg->kine.Z;
  ccb->end_x   = target.X;
  ccb->end_z   = target.Z;

  ccb->delta_x = target.X - leg->kine.X;
  ccb->delta_z = target.Z - leg->kine.Z;

  ccb->frame.count        = frame_count;
  ccb->frame.index        = 0;
  ccb->frame.elapsed_tick = 0;
  ccb->frame.last_tick    = 0;
}

bool leg_ccb_update(quad_leg* leg, quad_ccb* ccb)
{
  bool flag = false;
  quad_fp t = 0;

  //! Coord Control Block
  if(ccb->frame.count == 0) {
    return false;
  }
  quad_coord coord = (quad_coord) {leg->kine.X, leg->kine.Z};
  ++ccb->frame.index;
  if(ccb->frame.index == ccb->frame.count) {
    //最后一帧 
    t = 1.0f;
    coord.X = ccb->end_x;
    coord.Z = ccb->end_z;
    flag = false;
  } else {
    //非最后一帧
    t = ((quad_fp)(ccb->frame.index-1))/((quad_fp)(ccb->frame.count-1));
    coord.X = ccb->start_x + ccb->delta_x * ccb->calc_x(t);
    coord.Z = ccb->start_z + ccb->delta_z * ccb->calc_z(t);
    flag = true;
  }

  //! Hardware
  leg_set_coord(leg, coord.X, coord.Z);
  return flag;
}

void leg_ccb_update_block(quad_leg* leg, quad_ccb* ccb)
{
  while(leg_ccb_update(leg, ccb));
}

bool leg_ccb_update_all()
{
  bool flag = false;

  flag |= leg_ccb_update(&leg_rf,&leg_rf.ccb);
  flag |= leg_ccb_update(&leg_lf,&leg_lf.ccb);
  flag |= leg_ccb_update(&leg_rb,&leg_rb.ccb);
  flag |= leg_ccb_update(&leg_lb,&leg_lb.ccb);

  return flag;
}

void leg_ccb_update_all_block()
{
  while(leg_ccb_update_all());
}

//!sync section

static uint32_t sync_frame_count = 0;
static uint32_t sync_frame_index = 0;

void sync_leg_set_angle(quad_agnle rfa, quad_agnle lfa, quad_agnle rba, quad_agnle lba)
{
  quad_fp angles[8];
  angles[leg_rf.thigh_servo.channel] = rfa.AS1;
  angles[leg_rf.shank_servo.channel] = rfa.AS2;
  angles[leg_lf.thigh_servo.channel] = lfa.AS1;
  angles[leg_lf.shank_servo.channel] = lfa.AS2;
  angles[leg_rb.thigh_servo.channel] = rba.AS1;
  angles[leg_rb.shank_servo.channel] = rba.AS2;
  angles[leg_lb.thigh_servo.channel] = lba.AS1;
  angles[leg_lb.shank_servo.channel] = lba.AS2;
  sync_servo_set_angle(angles);
}

void sync_leg_set_coord(quad_coord rfc, quad_coord lfc, quad_coord rbc, quad_coord lbc)
{
  rfc.X += CONFIG_X_OFFSET_RF;
  rfc.Z += CONFIG_Z_OFFSET_RF;
  lfc.X += CONFIG_X_OFFSET_LF;
  lfc.Z += CONFIG_Z_OFFSET_LF;
  rbc.X += CONFIG_X_OFFSET_RB;
  rbc.Z += CONFIG_Z_OFFSET_RB;
  lbc.X += CONFIG_X_OFFSET_LB;
  lbc.Z += CONFIG_Z_OFFSET_LB;

  kine_inverse(&leg_rf.kine, rfc.X, rfc.Z);
  kine_inverse(&leg_lf.kine, lfc.X, lfc.Z);
  kine_inverse(&leg_rb.kine, rbc.X, rbc.Z);
  kine_inverse(&leg_lb.kine, lbc.X, lbc.Z);

  quad_fp thigh_angle_rf = leg_rf.kine.AS1;
  quad_fp shank_angle_rf = leg_rf.kine.AS2;
  quad_fp thigh_angle_lf = leg_lf.kine.AS1;
  quad_fp shank_angle_lf = leg_lf.kine.AS2;
  quad_fp thigh_angle_rb = leg_rb.kine.AS1;
  quad_fp shank_angle_rb = leg_rb.kine.AS2;
  quad_fp thigh_angle_lb = leg_lb.kine.AS1;
  quad_fp shank_angle_lb = leg_lb.kine.AS2;

  thigh_angle_rf = angle_offset(thigh_angle_rf, leg_rf.thigh_servo.offset);
  shank_angle_rf = angle_offset(shank_angle_rf, leg_rf.shank_servo.offset);
  thigh_angle_lf = angle_offset(thigh_angle_lf, leg_lf.thigh_servo.offset);
  shank_angle_lf = angle_offset(shank_angle_lf, leg_lf.shank_servo.offset);
  thigh_angle_rb = angle_offset(thigh_angle_rb, leg_rb.thigh_servo.offset);
  shank_angle_rb = angle_offset(shank_angle_rb, leg_rb.shank_servo.offset);
  thigh_angle_lb = angle_offset(thigh_angle_lb, leg_lb.thigh_servo.offset);
  shank_angle_lb = angle_offset(shank_angle_lb, leg_lb.shank_servo.offset);
  
  thigh_angle_rf = angle_limit_thigh(thigh_angle_rf);
  shank_angle_rf = angle_limit_shank(shank_angle_rf);
  thigh_angle_lf = angle_limit_thigh(thigh_angle_lf);
  shank_angle_lf = angle_limit_shank(shank_angle_lf);
  thigh_angle_rb = angle_limit_thigh(thigh_angle_rb);
  shank_angle_rb = angle_limit_shank(shank_angle_rb);
  thigh_angle_lb = angle_limit_thigh(thigh_angle_lb);
  shank_angle_lb = angle_limit_shank(shank_angle_lb);

  thigh_angle_lf = angle_mirror(thigh_angle_lf);
  shank_angle_lf = angle_mirror(shank_angle_lf);
  thigh_angle_lb = angle_mirror(thigh_angle_lb);
  shank_angle_lb = angle_mirror(shank_angle_lb);

  quad_fp angles[8];

  angles[leg_rf.thigh_servo.channel] = thigh_angle_rf;
  angles[leg_rf.shank_servo.channel] = shank_angle_rf;
  angles[leg_lf.thigh_servo.channel] = thigh_angle_lf;
  angles[leg_lf.shank_servo.channel] = shank_angle_lf;
  angles[leg_rb.thigh_servo.channel] = thigh_angle_rb;
  angles[leg_rb.shank_servo.channel] = shank_angle_rb;
  angles[leg_lb.thigh_servo.channel] = thigh_angle_lb;
  angles[leg_lb.shank_servo.channel] = shank_angle_lb;

  sync_servo_set_angle(angles);
}

void sync_leg_acb_start(uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }
  sync_frame_count = frame_count;
  sync_frame_index = 0;
}

bool sync_leg_acb_update()
{
  bool flag = false;
  quad_fp t = 0;
  quad_fp angles[8];

  if(sync_frame_count == 0) { return false; }

  if(sync_frame_index == sync_frame_count) {
    //最后一帧
    t = 1.0f;
    angles[leg_rf.thigh_servo.channel] = leg_rf.tacb.end;
    angles[leg_rf.shank_servo.channel] = leg_rf.sacb.end;
    angles[leg_lf.thigh_servo.channel] = leg_lf.tacb.end;
    angles[leg_lf.shank_servo.channel] = leg_lf.sacb.end;
    angles[leg_rb.thigh_servo.channel] = leg_rb.tacb.end;
    angles[leg_rb.shank_servo.channel] = leg_rb.sacb.end;
    angles[leg_lb.thigh_servo.channel] = leg_lb.tacb.end;
    angles[leg_lb.shank_servo.channel] = leg_lb.sacb.end;
    flag = false;
  } else {
    //非最后一帧
    t = ((quad_fp)(sync_frame_index))/((quad_fp)(sync_frame_count));
    angles[leg_rf.thigh_servo.channel] = leg_rf.tacb.start + leg_rf.tacb.delta * leg_rf.tacb.calc(t);
    angles[leg_rf.shank_servo.channel] = leg_rf.sacb.start + leg_rf.sacb.delta * leg_rf.sacb.calc(t);
    angles[leg_lf.thigh_servo.channel] = leg_lf.tacb.start + leg_lf.tacb.delta * leg_lf.tacb.calc(t);
    angles[leg_lf.shank_servo.channel] = leg_lf.sacb.start + leg_lf.sacb.delta * leg_lf.sacb.calc(t);
    angles[leg_rb.thigh_servo.channel] = leg_rb.tacb.start + leg_rb.tacb.delta * leg_rb.tacb.calc(t);
    angles[leg_rb.shank_servo.channel] = leg_rb.sacb.start + leg_rb.sacb.delta * leg_rb.sacb.calc(t);
    angles[leg_lb.thigh_servo.channel] = leg_lb.tacb.start + leg_lb.tacb.delta * leg_lb.tacb.calc(t);
    angles[leg_lb.shank_servo.channel] = leg_lb.sacb.start + leg_lb.sacb.delta * leg_lb.sacb.calc(t);
    flag = true;
  }

  //! Hardware
  angles[leg_rf.thigh_servo.channel] = angle_offset(angles[leg_rf.thigh_servo.channel], leg_rf.thigh_servo.offset);
  angles[leg_rf.shank_servo.channel] = angle_offset(angles[leg_rf.shank_servo.channel], leg_rf.shank_servo.offset);
  angles[leg_lf.thigh_servo.channel] = angle_offset(angles[leg_lf.thigh_servo.channel], leg_lf.thigh_servo.offset);
  angles[leg_lf.shank_servo.channel] = angle_offset(angles[leg_lf.shank_servo.channel], leg_lf.shank_servo.offset);
  angles[leg_rb.thigh_servo.channel] = angle_offset(angles[leg_rb.thigh_servo.channel], leg_rb.thigh_servo.offset);
  angles[leg_rb.shank_servo.channel] = angle_offset(angles[leg_rb.shank_servo.channel], leg_rb.shank_servo.offset);
  angles[leg_lb.thigh_servo.channel] = angle_offset(angles[leg_lb.thigh_servo.channel], leg_lb.thigh_servo.offset);
  angles[leg_lb.shank_servo.channel] = angle_offset(angles[leg_lb.shank_servo.channel], leg_lb.shank_servo.offset);
  
  angles[leg_rf.thigh_servo.channel] = angle_limit_thigh(angles[leg_rf.thigh_servo.channel]);
  angles[leg_rf.shank_servo.channel] = angle_limit_shank(angles[leg_rf.shank_servo.channel]);
  angles[leg_lf.thigh_servo.channel] = angle_limit_thigh(angles[leg_lf.thigh_servo.channel]);
  angles[leg_lf.shank_servo.channel] = angle_limit_shank(angles[leg_lf.shank_servo.channel]);
  angles[leg_rb.thigh_servo.channel] = angle_limit_thigh(angles[leg_rb.thigh_servo.channel]);
  angles[leg_rb.shank_servo.channel] = angle_limit_shank(angles[leg_rb.shank_servo.channel]);
  angles[leg_lb.thigh_servo.channel] = angle_limit_thigh(angles[leg_lb.thigh_servo.channel]);
  angles[leg_lb.shank_servo.channel] = angle_limit_shank(angles[leg_lb.shank_servo.channel]);

  angles[leg_lf.thigh_servo.channel] = angle_mirror(angles[leg_lf.thigh_servo.channel]);
  angles[leg_lf.shank_servo.channel] = angle_mirror(angles[leg_lf.shank_servo.channel]);
  angles[leg_lb.thigh_servo.channel] = angle_mirror(angles[leg_lb.thigh_servo.channel]);
  angles[leg_lb.shank_servo.channel] = angle_mirror(angles[leg_lb.shank_servo.channel]);
  sync_servo_set_angle(angles);
  ++sync_frame_index;
  return flag;
}

void sync_leg_acb_updata_block()
{
  bool flag = false;
#if CONFIG_ACB_FRAME_INTERVAL_MODE==0
  volatile uint32_t last_tick = 0;
#endif
  do {
  #if CONFIG_ACB_FRAME_INTERVAL_MODE==0
    if( HAL_GetTick()-last_tick > CONFIG_ACB_FRAME_INTERVAL0) {
      flag = sync_leg_acb_update();
      last_tick = HAL_GetTick();
  #elif CONFIG_ACB_FRAME_INTERVAL_MODE==1
    flag = sync_leg_acb_update();
    delay_ms(CONFIG_ACB_FRAME_INTERVAL1);
  #elif CONFIG_ACB_FRAME_INTERVAL_MODE==2
    flag = sync_leg_acb_update();
    delay_us(CONFIG_ACB_FRAME_INTERVAL2);
  #else
    flag = sync_leg_acb_update();
  #endif
    } else {
      flag = true;
    }
  } while(flag);
}

void sync_leg_acb_stop()
{
  sync_frame_count = 0;
  sync_frame_index = 0;
}

void sync_leg_ccb_start(uint32_t frame_count)
{
  if(frame_count == 0)
  {
    elog_w(TAG, "frame_count is 0");
    return;
  }
  sync_frame_count = frame_count;
  sync_frame_index = 0;
}

bool sync_leg_ccb_update()
{
  bool flag = false;
  quad_fp t = 0;
  quad_coord rfc, lfc, rbc, lbc;

  if(sync_frame_count == 0) { return false; }

  if(sync_frame_index == sync_frame_count) {
    //最后一帧
    t = 1.0f;
    rfc.X = leg_rf.ccb.end_x;
    rfc.Z = leg_rf.ccb.end_z;
    lfc.X = leg_lf.ccb.end_x;
    lfc.Z = leg_lf.ccb.end_z;
    rbc.X = leg_rb.ccb.end_x;
    rbc.Z = leg_rb.ccb.end_z;
    lbc.X = leg_lb.ccb.end_x;
    lbc.Z = leg_lb.ccb.end_z;
    flag = false;
  } else {
    //非最后一帧
    t = ((quad_fp)(sync_frame_index))/((quad_fp)(sync_frame_count));
    rfc.X = leg_rf.ccb.start_x + leg_rf.ccb.delta_x * leg_rf.ccb.calc_x(t);
    rfc.Z = leg_rf.ccb.start_z + leg_rf.ccb.delta_z * leg_rf.ccb.calc_z(t);
    lfc.X = leg_lf.ccb.start_x + leg_lf.ccb.delta_x * leg_lf.ccb.calc_x(t);
    lfc.Z = leg_lf.ccb.start_z + leg_lf.ccb.delta_z * leg_lf.ccb.calc_z(t);
    rbc.X = leg_rb.ccb.start_x + leg_rb.ccb.delta_x * leg_rb.ccb.calc_x(t);
    rbc.Z = leg_rb.ccb.start_z + leg_rb.ccb.delta_z * leg_rb.ccb.calc_z(t);
    lbc.X = leg_lb.ccb.start_x + leg_lb.ccb.delta_x * leg_lb.ccb.calc_x(t);
    lbc.Z = leg_lb.ccb.start_z + leg_lb.ccb.delta_z * leg_lb.ccb.calc_z(t);
    flag = true;
  }
  ++sync_frame_index;
  //! Hardware
  sync_leg_set_coord(rfc, lfc, rbc, lbc);

  elog_e(TAG, "fc:%5u fi:%5u t:%5.3f [rf:%5.3f %5.3f] [lf:%5.3f %5.3f] [rb:%5.3f %5.3f] [lb:%5.3f %5.3f]", 
    sync_frame_count, sync_frame_index, t,
    rfc.X, rfc.Z, lfc.X, lfc.Z, rbc.X, rbc.Z, lbc.X, lbc.Z);
  return flag;
}

void sync_leg_ccb_update_block()
{
  elog_d(TAG,"start");
  bool flag = false;
#if CONFIG_ACB_FRAME_INTERVAL_MODE==0
  volatile uint32_t last_tick = 0;
#endif
  do {
  #if CONFIG_ACB_FRAME_INTERVAL_MODE==0
    if( HAL_GetTick()-last_tick > CONFIG_ACB_FRAME_INTERVAL0) {
      flag = sync_leg_ccb_update();
      last_tick = HAL_GetTick();
    }
  #elif CONFIG_ACB_FRAME_INTERVAL_MODE==1
    flag = sync_leg_ccb_update();
    delay_ms(CONFIG_ACB_FRAME_INTERVAL1);
  #elif CONFIG_ACB_FRAME_INTERVAL_MODE==2
    flag = sync_leg_ccb_update();
    delay_us(CONFIG_ACB_FRAME_INTERVAL2);
  #else
    flag = sync_leg_ccb_update();
  #endif
  } while(flag);
  elog_d(TAG,"end");
}