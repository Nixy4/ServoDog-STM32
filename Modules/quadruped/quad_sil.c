#include "quadruped.h"

#define TAG "sil"

static quad_fp _swing_height  = 0;
static quad_fp _swing_duty   = 0;
static quad_coord _base      = {0};

static uint32_t _frame_count = 0;
static uint32_t _swing_count = 0;
static uint32_t _frame_index = 0;
static volatile uint32_t _frame_last_tick = 0;

static quad_fp T, Tz;
static quad_coord _swing , _support;

void sil_init(quad_fp swing_height, quad_fp swing_duty)
{
  _swing_height = swing_height;
  _swing_duty  = swing_duty;
  _base.X      = CONFIG_SIL_BASE_X;
  _base.Z      = CONFIG_SIL_BASE_Z;
  _swing.X     = CONFIG_SIL_BASE_X;
  _swing.Z     = CONFIG_SIL_BASE_Z;
}

void sil0()
{
  elog_i(TAG, "sil0");
  elog_d(TAG, "sil0: %10f %10f", _base.X, _base.Z);
  leg_ccb_target(&leg_rf, &leg_rf.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_target(&leg_lf, &leg_lf.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_target(&leg_rb, &leg_rb.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_target(&leg_lb, &leg_lb.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_update_all_block();
}

void sil1(uint32_t frame_count, uint32_t times)
{
  elog_d(TAG, "start");

  _frame_count = frame_count;
  _swing_count = (uint32_t)(_swing_duty * frame_count);
  elog_d(TAG, "sil1: %10d %10d", _frame_count, _swing_count);

  for(uint32_t i = 0; i< times; ++i)
  {
    _frame_index = 0;
    _frame_last_tick = 0;
    do {
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      if(HAL_GetTick()-_frame_last_tick <= CONFIG_SIL1_FRAME_INTERVAL0) {
        continue;
      }
    #endif
      T = DPI * _frame_index / _swing_count;
      Tz = (1-cos(T)) / 2.f;
      _swing.Z   = _swing_height * Tz;
      _support.Z = 0;
      _swing.Z   = -_swing.Z + _base.Z;
      _support.Z = -_support.Z + _base.Z;
      elog_d(TAG, "walk2: %10d %10f %10f %10f %10f", _frame_index, _swing.X, _swing.Z, _support.X, _support.Z);
      //右前腿和左后退摆动,右后腿和左前腿支撑
      leg_set_coord(&leg_rf, CONFIG_SIL_BASE_X, _swing.Z);
      leg_set_coord(&leg_lf, CONFIG_SIL_BASE_X, _support.Z);
      leg_set_coord(&leg_rb, CONFIG_SIL_BASE_X, _support.Z);
      leg_set_coord(&leg_lb, CONFIG_SIL_BASE_X, _swing.Z);
      ++_frame_index;
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      _frame_last_tick = HAL_GetTick();
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==1
      delay_ms(CONFIG_SIL1_FRAME_INTERVAL1);
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==2
      delay_us(CONFIG_SIL1_FRAME_INTERVAL2);
    #endif
    } while(_frame_index < _swing_count);

    do {
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      if(HAL_GetTick()-_frame_last_tick <= CONFIG_SIL1_FRAME_INTERVAL0) {
        continue;
      }
    #endif
      T = (DPI * _frame_index - DPI * _swing_count) / _swing_count;
      Tz = (1-cos(T)) / 2.f;
      _swing.Z   = _swing_height * Tz;
      _support.Z = 0 ;
      _swing.Z   = -_swing.Z + _base.Z;
      _support.Z = -_support.Z + _base.Z;
      //右前腿和左后腿支撑,右后腿和左前腿摆动
      leg_set_coord(&leg_rf, CONFIG_SIL_BASE_X, _support.Z);
      leg_set_coord(&leg_lf, CONFIG_SIL_BASE_X, _swing.Z);
      leg_set_coord(&leg_rb, CONFIG_SIL_BASE_X, _swing.Z); 
      leg_set_coord(&leg_lb, CONFIG_SIL_BASE_X, _support.Z);
      ++_frame_index;
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      _frame_last_tick = HAL_GetTick();
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==1
      delay_ms(CONFIG_SIL1_FRAME_INTERVAL1);
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==2
      delay_us(CONFIG_SIL1_FRAME_INTERVAL2);
    #endif

    } while(_frame_index < _frame_count);
  }
}

void sil(uint32_t frame_count, uint32_t times)
{
  sil0();
  sil1(frame_count, times);
  sil0();
}

void sync_sil0()
{
  elog_d(TAG, "start");
  elog_d(TAG, "_base.X: %10f | _base.Z: %10f", _base.X, _base.Z);
  
  leg_ccb_target(&leg_rf, &leg_rf.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_target(&leg_lf, &leg_lf.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_target(&leg_rb, &leg_rb.ccb, _base, CONFIG_SIL0_FRAME_COUNT);
  leg_ccb_target(&leg_lb, &leg_lb.ccb, _base, CONFIG_SIL0_FRAME_COUNT);

  sync_leg_ccb_start(CONFIG_SIL0_FRAME_COUNT);
  sync_leg_ccb_update_block();
  elog_d(TAG, "end");
}

void sync_sil1(uint32_t frame_count, uint32_t times)
{
  elog_d(TAG, "start");
  _frame_count = CONFIG_SIL1_FRAME_INTERVAL0;
  _swing_count = (uint32_t)(_swing_duty * CONFIG_SIL1_FRAME_INTERVAL0);
  elog_d(TAG, "frame_count: %10d | swing_count: %10d", _frame_count, _swing_count);
  for(uint32_t i = 0; i< times; ++i)
  {
    _frame_index = 0;
    _frame_last_tick = 0;
    do {
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      if(HAL_GetTick()-_frame_last_tick <= CONFIG_SIL1_FRAME_INTERVAL0) {
        continue;
      }
    #endif
      T = DPI * _frame_index / _swing_count;
      Tz = (1-cos(T)) / 2.f;
      _swing.Z   = _swing_height * Tz;
      _support.Z = 0;
      _swing.Z   = -_swing.Z + _base.Z;
      _support.Z = -_support.Z + _base.Z;
      //右前腿和左后退摆动,右后腿和左前腿支撑
      quad_coord rfc, lfc, rbc, lbc;
      rfc.X = CONFIG_SIL_BASE_X;
      rfc.Z = _swing.Z;
      lfc.X = CONFIG_SIL_BASE_X;
      lfc.Z = _support.Z;
      rbc.X = CONFIG_SIL_BASE_X;
      rbc.Z = _support.Z;
      lbc.X = CONFIG_SIL_BASE_X;
      lbc.Z = _swing.Z;
      //! Hardware
      sync_leg_set_coord(rfc, lfc, rbc, lbc);
      ++_frame_index;
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      _frame_last_tick = HAL_GetTick();
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==1
      delay_ms(CONFIG_SIL1_FRAME_INTERVAL1);
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==2
      delay_us(CONFIG_SIL1_FRAME_INTERVAL2);
    #endif
    } while(_frame_index < _swing_count);

    do {
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      if(HAL_GetTick()-_frame_last_tick <= CONFIG_SIL1_FRAME_INTERVAL0) {
        continue;
      }
    #endif
      T = (DPI * _frame_index - DPI * _swing_count) / _swing_count;
      Tz = (1-cos(T)) / 2.f;
      _swing.Z   = _swing_height * Tz;
      _support.Z = 0 ;
      _swing.Z   = -_swing.Z + _base.Z;
      _support.Z = -_support.Z + _base.Z;
      //右前腿和左后腿支撑,右后腿和左前腿摆动
      quad_coord rfc, lfc, rbc, lbc;
      rfc.X = CONFIG_SIL_BASE_X;
      rfc.Z = _support.Z;
      lfc.X = CONFIG_SIL_BASE_X;
      lfc.Z = _swing.Z;
      rbc.X = CONFIG_SIL_BASE_X;
      rbc.Z = _swing.Z; 
      lbc.X = CONFIG_SIL_BASE_X;
      lbc.Z = _support.Z;
      //! Hardware
      sync_leg_set_coord(rfc, lfc, rbc, lbc);
      ++_frame_index;
    #if CONFIG_SIL1_FRAME_INTERVAL_MODE==0
      _frame_last_tick = HAL_GetTick();
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==1
      delay_ms(CONFIG_SIL1_FRAME_INTERVAL1);
    #elif CONFIG_SIL1_FRAME_INTERVAL_MODE==2
      delay_us(CONFIG_SIL1_FRAME_INTERVAL2);
    #endif
    } while(_frame_index < _frame_count);
  }
}

void sync_sil(uint32_t frame_count, uint32_t times)
{
  sync_sil0();
  sync_sil1(frame_count, times);
  sync_sil0();
}
