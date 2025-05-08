#include "./define.h"

#define TAG "Leg"

quad_leg leg_rf = {.type=LEG_RF};
quad_leg leg_lf  = {.type=LEG_LF};
quad_leg leg_rb  = {.type=LEG_RB};
quad_leg leg_lb   = {.type=LEG_LB};

static const char* get_leg_str(quad_types type)
{
  switch(type)
  {
    case LEG_RF: return "RF";
    case LEG_LF: return "LF";
    case LEG_RB: return "RB";
    case LEG_LB: return "LB";
    default:       return "Unknown";
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

  elog_v(TAG, "leg_set_angle: %s after offset, thigh_angle: %.2f shank_angle: %.2f", 
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
  kine_inverse(&leg->kine, X, Z);
  
  quad_fp thigh_angle = leg->kine.AS1;
  quad_fp shank_angle = leg->kine.AS2;

  thigh_angle = angle_offset(thigh_angle, leg->thigh_servo.offset);
  shank_angle = angle_offset(shank_angle, leg->shank_servo.offset);

  elog_v(TAG, "leg_set_coord: %s after offset, thigh_angle: %.2f shank_angle: %.2f", 
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
    elog_w(TAG, "lstart == end");
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
  volatile quad_fp* current_ptr;
  quad_servo* servo_ptr;
  quad_fp angle;

  if(acb->type == ACB_THIGH)
  {
    current_ptr = &leg->kine.AS1;
    servo_ptr = &leg->thigh_servo;
  }
  else if(acb->type == ACB_SHANK)
  {
    current_ptr = &leg->kine.AS2;
    servo_ptr = &leg->shank_servo;
  }
  else
  {
    elog_e(TAG, "unknown acb type");
    return false;
  }

  if(acb->frame.interval > 0)
  {
    if(HAL_GetTick()-acb->frame.last_tick < acb->frame.interval) {
      return true;
    }
  }

  ++acb->frame.index;

  if(acb->frame.index == acb->frame.count) //最后一帧
  {
    t = 1.0f;
    *current_ptr = acb->end;
    flag = false;
  }
  else
  {
    t = ((quad_fp)(acb->frame.index-1))/((quad_fp)(acb->frame.count-1));
    *current_ptr = acb->start + acb->delta * acb->calc(t);
    flag = true;
  }

  angle = angle_offset(*current_ptr,servo_ptr->offset);

  if(acb->type==ACB_THIGH)
  {
    angle = angle_limit_thigh(angle);
  }
  else 
  {
    angle = angle_limit_shank(angle);
  }

  if(leg->type==LEG_LF || leg->type==LEG_LB)
  {
    angle = angle_mirror(angle);
  }

  servo_set_angle(servo_ptr, angle);

  if(acb->frame.interval > 0) 
  {
    acb->frame.last_tick = HAL_GetTick();
  }

  return flag;
}

void leg_acb_update_block(quad_leg* leg, quad_acb* acb)
{
  while(leg_acb_update(leg,acb));
}

bool legs_acb_update()
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

void legs_acb_update_block()
{
  while(legs_acb_update());
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
  
  if(ccb->frame.interval > 0)
  {
    if(HAL_GetTick()-ccb->frame.last_tick < ccb->frame.interval) {
      return true;
    }
  }

  quad_coord current = (quad_coord) {leg->kine.X, leg->kine.Z};
  ++ccb->frame.index;

  if(ccb->frame.index == ccb->frame.count) //最后一帧
  {
    t = 1.0f;
    current.X = ccb->end_x;
    current.Z = ccb->end_z;
    flag = false;
  }
  else
  {
    t = ((quad_fp)(ccb->frame.index-1))/((quad_fp)(ccb->frame.count-1));
    current.X = ccb->start_x + ccb->delta_x * ccb->calc_x(t);
    current.Z = ccb->start_z + ccb->delta_z * ccb->calc_z(t);
    flag = true;
  }

  leg_set_coord(leg, current.X, current.Z);

  if(ccb->frame.interval > 0) 
  {
    ccb->frame.last_tick = HAL_GetTick();
  }

  return flag;
}

void leg_ccb_update_block(quad_leg* leg, quad_ccb* ccb)
{
  while(leg_ccb_update(leg, ccb));
}

bool legs_ccb_update()
{
  bool flag = false;

  flag |= leg_ccb_update(&leg_rf,&leg_rf.ccb);
  flag |= leg_ccb_update(&leg_lf,&leg_lf.ccb);
  flag |= leg_ccb_update(&leg_rb,&leg_rb.ccb);
  flag |= leg_ccb_update(&leg_lb,&leg_lb.ccb);

  return flag;
}

void legs_ccb_update_block()
{
  while(legs_ccb_update());
}