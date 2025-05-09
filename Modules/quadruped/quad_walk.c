#include "quadruped.h"

#include "stdint.h"

#define TAG "walk"

static quad_fp _swing_width  = 0;
static quad_fp _swing_height = 0;
static quad_fp _swing_duty   = 0;
static quad_coord _base      = {0};

static uint32_t _frame_count    = 0;
static uint32_t _swing_count    = 0;
static uint32_t _frame_index    = 0;

static quad_fp T, Tx, Tz;

static quad_coord _swing , _support;

void walk_init(quad_fp swing_width, quad_fp swing_height, quad_fp swing_duty)
{
  _swing_width  = swing_width;
  _swing_height = swing_height;
  _swing_duty   = swing_duty;
  _base.X       = swing_width * swing_duty;
  _base.Z       = CONFIG_WALK_BASE_Z;
}

void walk0(void)
{
  elog_i(TAG, "walk0");
  elog_d(TAG, "walk0: %10f %10f", _base.X, _base.Z);
  leg_ccb_target(&leg_rf, &leg_rf.ccb, _base, CONFIG_WALK0_FRAME_COUNT);
  leg_ccb_target(&leg_lf, &leg_lf.ccb, _base, CONFIG_WALK0_FRAME_COUNT);
  leg_ccb_target(&leg_rb, &leg_rb.ccb, _base, CONFIG_WALK0_FRAME_COUNT);
  leg_ccb_target(&leg_lb, &leg_lb.ccb, _base, CONFIG_WALK0_FRAME_COUNT);
  leg_ccb_update_all_block();
}

void walk1(uint32_t frame_count)
{
  elog_d(TAG, "walk1");

  _frame_count = frame_count;
  _swing_count = (uint32_t)(_swing_duty * _frame_count);
  _frame_index = 0;
  elog_d(TAG, "walk1: %10d %10d", _frame_count, _swing_count);

  do {
    T = DPI * _frame_index / _swing_count;
    Tx = (T-sin(T)) / DPI;
    Tz = (1-cos(T)) / 2.f;
    _swing.X = _swing_width  * Tx;
    _swing.Z = _swing_height * Tz;
    _swing.X = -_swing.X + _base.X;
    _swing.Z = -_swing.Z + _base.Z;
    elog_d(TAG, "walk1: %10d %10f %10f", _frame_index, _swing.X, _swing.Z);

    //右前腿和左后腿静止,左前腿和右后腿动
    leg_set_coord(&leg_rf, _base.X, _base.Z);
    leg_set_coord(&leg_lf, _swing.X, _swing.Z);
    leg_set_coord(&leg_rb, _swing.X, _swing.Z);
    leg_set_coord(&leg_lb, _base.X, _base.Z);
    ++_frame_index;
    // HAL_Delay(1);
  } while(_frame_index < _swing_count);
}

void walk2(uint32_t frame_count, uint32_t steps)
{
  elog_d(TAG, "walk2");

  _frame_count = frame_count;
  _swing_count = (uint32_t)(_swing_duty * _frame_count);
  elog_d(TAG, "walk2: %10d %10d", _frame_count, _swing_count);

  for(uint32_t i = 0; i< steps; ++i)
  {
    _frame_index = 0;
    do {
      T = DPI * _frame_index / _swing_count;
      Tx = (T-sin(T)) / DPI;
      Tz = (1-cos(T)) / 2.f;
      _swing.X   = _swing_width  * Tx;
      _swing.Z   = _swing_height * Tz;
      _support.X = _swing_width  * (1.f-Tx);
      _support.Z = 0;
      _swing.X   = -_swing.X + _base.X;
      _swing.Z   = -_swing.Z + _base.Z;
      _support.X = -_support.X + _base.X;
      _support.Z = -_support.Z + _base.Z;
      elog_d(TAG, "walk2: %10d %10f %10f %10f %10f", _frame_index, _swing.X, _swing.Z, _support.X, _support.Z);

      //右前腿和左后退摆动,右后腿和左前腿支撑
      leg_set_coord(&leg_rf, _swing.X, _swing.Z);
      leg_set_coord(&leg_lf, _support.X, _support.Z);
      leg_set_coord(&leg_rb, _support.X, _support.Z);
      leg_set_coord(&leg_lb, _swing.X, _swing.Z);
      ++_frame_index;
      // HAL_Delay(1);
    } while(_frame_index < _swing_count);
    do {
      T = (DPI * _frame_index - DPI * _swing_count) / _swing_count;
      Tx = (T-sin(T)) / DPI;
      Tz = (1-cos(T)) / 2.f;
      _swing.X   = _swing_width  * Tx;
      _swing.Z   = _swing_height * Tz;
      _support.X = _swing_width  * (1.f-Tx);
      _support.Z = 0 ;
      _swing.X   = -_swing.X + _base.X;
      _swing.Z   = -_swing.Z + _base.Z;
      _support.X = -_support.X + _base.X;
      _support.Z = -_support.Z + _base.Z;
      //右前腿和左后腿支撑,右后腿和左前腿摆动
      leg_set_coord(&leg_rf, _support.X, _support.Z);
      leg_set_coord(&leg_lf, _swing.X, _swing.Z);
      leg_set_coord(&leg_rb, _swing.X, _swing.Z); 
      leg_set_coord(&leg_lb, _support.X, _support.Z);
      ++_frame_index;
      // HAL_Delay(1);
    } while(_frame_index < _frame_count);
  }
}

void walk(uint32_t frame_count, uint32_t steps)
{
  walk0();
  walk1(frame_count);
  walk2(frame_count, steps);
  walk0();
}

