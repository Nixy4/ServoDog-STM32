#include "quadruped.h"

#define TAG "Trot"
#include "elog.h" 
//! Init Variables
static quad_fp _swing_width;
static quad_fp _swing_height;
static quad_fp _cuf_off_percentage;
static uint32_t _cuf_off_index;
static quad_coord _base;
static quad_fcb _fcb;
static uint32_t _frame_count;

void quad_tort_init(
    quad_fp swing_width, quad_fp swing_height, quad_fp cuf_off_percentage, quad_coord base, fcb_mode mode, uint32_t frame_count, uint32_t frame_interval)
{

  _swing_width        = swing_width;
  _swing_height       = swing_height;
  _cuf_off_percentage = cuf_off_percentage;
  _base.X             = base.X + swing_width/2.f;
  _base.Z             = base.Z;
  _frame_count        = frame_count;

  _cuf_off_index      = (uint32_t)(_frame_count*_cuf_off_percentage+0.5f);
  _cuf_off_index      = (_cuf_off_index > _frame_count) ? _frame_count : _cuf_off_index;
  fcb_init(&_fcb, mode, frame_interval);
  elog_i(TAG, "sw = %f, sh = %f, cop = %f, base = (%f,%f), frame_count = %d, cuf_off_index = %d, interval = %d", 
    swing_width, swing_height, cuf_off_percentage, base.X, base.Z, frame_count, _cuf_off_index, frame_interval);
}

void quad_tort(uint32_t step_count)
{
  quad_fp T, Tx, Tz;
  quad_coord swing, support;
  quad_coord coords[LEG_COUNT];

  //准备动作, 移动至_base
  leg_sync_ccb_target_blocking(_base, CONFIG_TORT_READY_FRAME_COUNT);

  //小跑起步 [右前|左后]不动 [左前|右后]起步
  fcb_start(&_fcb, _frame_count);
  while (1)
  {
    //! Frame Control Block
    if(fcb_current(&_fcb) == _cuf_off_index) {
      break;
    } else if(fcb_skip(&_fcb)) {
      continue;
    } else {
      fcb_next(&_fcb);
    }
    //! Gait Calculation
    T = DPI * fcb_current(&_fcb) / _cuf_off_index;
    Tx = (T-sin(T)) / DPI;
    Tz = (1-cos(T)) / 2.f;
    swing.X = _swing_width  * Tx;
    swing.Z = _swing_height * Tz;
    //! Coord Mapping
    swing = coord_mapping(swing,_base);
    //! Leg Control
    coords[LEG_RF] = _base;
    coords[LEG_LF] = swing;
    coords[LEG_RB] = swing;
    coords[LEG_LB] = _base;
    leg_sync_set_coord2(coords);
  }

  //小跑循环 [右前|左后]先动 [左前|右后]后动
  for(uint32_t step = 0; step < step_count; step++) 
  {
    fcb_start(&_fcb, _frame_count);
    while (1)
    {
      if(fcb_complete(&_fcb)) {
        break;
      } else if( fcb_skip(&_fcb) ) {
        continue;
      } else {
        fcb_next(&_fcb);
      }
      if( fcb_current(&_fcb) <= _cuf_off_index ) 
      {
        T = DPI * (quad_fp)fcb_current(&_fcb) / (quad_fp)_cuf_off_index;
        Tx = (T-sin(T)) / DPI;
        Tz = (1-cos(T)) / 2.f;
        swing.X        = _swing_width  * Tx;
        swing.Z        = _swing_height * Tz;
        support.X      = _swing_width  * (1.f-Tx);
        support.Z      = 0;
        swing          = coord_mapping(swing,_base);
        support        = coord_mapping(support,_base);
        coords[LEG_RF] = swing;
        coords[LEG_LF] = support;
        coords[LEG_RB] = support;
        coords[LEG_LB] = swing;
      } else {
        T = (DPI * (quad_fp)fcb_current(&_fcb) - DPI * (quad_fp)_cuf_off_index) / (quad_fp)(_frame_count-_cuf_off_index);
        Tx = (T-sin(T)) / DPI;
        Tz = (1-cos(T)) / 2.f;
        swing.X        = _swing_width  * Tx;
        swing.Z        = _swing_height * Tz;
        support.X      = _swing_width  * (1.f-Tx);
        support.Z      = 0;
        swing          = coord_mapping(swing,_base);
        support        = coord_mapping(support,_base);
        coords[LEG_RF] = support;
        coords[LEG_LF] = swing;
        coords[LEG_RB] = swing;
        coords[LEG_LB] = support;
      }
      //! Leg Control
      leg_sync_set_coord2(coords);
    }
  }
  //小跑结束 移动至_base
  leg_sync_ccb_target(_base, CONFIG_TORT_READY_FRAME_COUNT);
  leg_sync_ccb_update_blocking();

  //回复stand0
  quad_fixed_stand0();
}