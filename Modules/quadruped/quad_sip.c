#include "quadruped.h"

#define TAG "StepInPlace"

static quad_fp _swing_height;
static quad_fp _cuf_off_percentage;
static uint32_t _cuf_off_index;
static quad_coord _base;
static quad_fcb _fcb;
static uint32_t _frame_count;

void quad_sip_init(
  quad_fp swing_height, quad_fp cuf_off_percentage, quad_coord base, fcb_mode mode, uint32_t frame_count, uint32_t frame_interval)
{
  //! Parameter Check
  if (swing_height <= 0) {
    elog_w(TAG, "swing_height is %f, set to 1.f", swing_height);
    swing_height = 1.f;
  }
  if (cuf_off_percentage <= 0) {
    elog_w(TAG, "cuf_off_percentage is %f, set to 0.5f", cuf_off_percentage);
    cuf_off_percentage = 0.5f;
  } else if (cuf_off_percentage > 1) {
    elog_w(TAG, "cuf_off_percentage is %f, set to 1.f", cuf_off_percentage);
    cuf_off_percentage = 1.f;
  }
  _swing_height       = swing_height;
  _cuf_off_percentage = cuf_off_percentage;
  _base               = base;
  _frame_count        = frame_count;
  fcb_init(&_fcb, mode, frame_interval );
  elog_i(TAG, "swing_height = %f, cuf_off_percentage = %f, base = (%f,%f), frame_count = %d, frame_interval = %d", 
    swing_height, cuf_off_percentage, base.X, base.Z, frame_count, frame_interval);
}

void quad_sip(uint32_t step_count)
{
  //! Parameter Check
  if (step_count == 0) {
    elog_w(TAG, "step_count is 0, set to 1");
    step_count = 1;
  }

  quad_fp T, Tx, Tz;
  quad_coord swing, support;
  quad_coord coords[LEG_COUNT];

  _cuf_off_index  = (uint32_t)(fcb_count(&_fcb)*_cuf_off_percentage+0.5f);
  _cuf_off_index = (_cuf_off_index > fcb_count(&_fcb)) ? fcb_count(&_fcb) : _cuf_off_index;

  //准备动作,移动至_base
  leg_sync_ccb_target_blocking(_base, CONFIG_SIP_READY_FRAME_COUNT);

  //踏步循环 [右前|左后]先动 [左前|右后]后动
  for(uint32_t step=0;step<step_count;step++)
  {
    fcb_start(&_fcb, _frame_count);
    while (1)
    {
      //! FCB
      if( fcb_complete(&_fcb) ) {
        break;
      } else if( fcb_skip(&_fcb) ) {
        continue;
      } else {
        fcb_next(&_fcb);
      }
      //! Calculation
      if( fcb_current(&_fcb) <= _cuf_off_index ) 
      {
        T = DPI * (quad_fp)fcb_current(&_fcb) / (quad_fp)_cuf_off_index;
        // elog_t(TAG, "1 fcb_percentage = %f, T = %f", fcb_percentage(&_fcb), T);
        UNUSED(Tx);
        Tx = (T-sin(T)) / DPI;
        Tz = (1-cos(T)) / 2.f;
        swing.X        = 0;
        swing.Z        = _swing_height * Tz;
        support.X      = 0;
        support.Z      = 0;
        swing          = coord_mapping(swing,_base);
        support        = coord_mapping(support,_base);
        coords[LEG_RF] = swing;
        coords[LEG_LF] = support;
        coords[LEG_RB] = support;
        coords[LEG_LB] = swing;
      } else {
        T = DPI * ( (quad_fp)fcb_current(&_fcb) - (quad_fp)_cuf_off_index) / (quad_fp)_cuf_off_index;
        // elog_t(TAG, "2 fcb_percentage = %f, T = %f", fcb_percentage(&_fcb), T);
        UNUSED(Tx);
        Tx = (T-sin(T)) / DPI;
        Tz = (1-cos(T)) / 2.f;
        swing.X        = 0;
        swing.Z        = _swing_height * Tz;
        support.X      = 0;
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

  //踏步循环结束,移动至_base
  leg_sync_ccb_target_blocking(_base, CONFIG_SIP_READY_FRAME_COUNT);
}

void quad_sip2(uint32_t step_count,  uint32_t frame_count)
{
  quad_coord deltas[LEG_COUNT] = {0};

  //踏步准备 先抬起一组腿 [右前|左后]抬起 [左前|右后]不动
  deltas[LEG_RF].Z = -_swing_height;
  deltas[LEG_LF].Z = 0;
  deltas[LEG_RB].Z = 0;
  deltas[LEG_LB].Z = -_swing_height;
  leg_sync_ccb_relative2_blocking(deltas, frame_count);

  //踏步循环 两组腿交替抬起和放下
  for(uint32_t step=0;step<step_count;step++)
  {
    //[右前|左后]放下 [左前|右后]抬起
    deltas[LEG_RF].Z = _swing_height;
    deltas[LEG_LF].Z = 0;
    deltas[LEG_RB].Z = 0;
    deltas[LEG_LB].Z = _swing_height;
    leg_sync_ccb_relative2_blocking(deltas, frame_count);
    //[右前|左后]放下 [左前|右后]抬起
    deltas[LEG_RF].Z = 0;
    deltas[LEG_LF].Z = -_swing_height;
    deltas[LEG_RB].Z = -_swing_height;
    deltas[LEG_LB].Z = 0;
    leg_sync_ccb_relative2_blocking(deltas, frame_count); 
    deltas[LEG_RF].Z = 0;
    deltas[LEG_LF].Z = _swing_height;
    deltas[LEG_RB].Z = _swing_height;
    deltas[LEG_LB].Z = 0;
    leg_sync_ccb_relative2_blocking(deltas, frame_count);
    deltas[LEG_RF].Z = -_swing_height;
    deltas[LEG_LF].Z = 0;
    deltas[LEG_RB].Z = 0;
    deltas[LEG_LB].Z = -_swing_height;
    leg_sync_ccb_relative2_blocking(deltas, frame_count);
  }
  //踏步结束 放下所有腿
  leg_sync_ccb_target_blocking(_base, frame_count);
}

void quad_sip3(uint32_t step_count)
{
  //! Parameter Check
  if (step_count == 0) {
    elog_w(TAG, "step_count is 0, set to 1");
    step_count = 1;
  }
  if (_frame_count == 0) {
    elog_w(TAG, "_frame_count is 0, set to 1");
    _frame_count = 1;
  }

  quad_fp T, Tx, Tz;
  quad_coord swing, support;
  quad_coord coords[LEG_COUNT];

  uint32_t _cuf_off_index  = (uint32_t)(_frame_count*_cuf_off_percentage+0.5f);
  _cuf_off_index = (_cuf_off_index > _frame_count) ? _frame_count : _cuf_off_index;
  uint32_t support_count = _frame_count - _cuf_off_index;
  support_count = (support_count > 0) ? support_count : 1;

  elog_i(TAG, "step_count = %d, _frame_count = %d, _cuf_off_index = %d", 
    step_count, _frame_count, _cuf_off_index);

  //准备动作,移动至_base
  leg_sync_ccb_target_blocking(_base, CONFIG_SIP_READY_FRAME_COUNT);

  //踏步循环 [右前|左后]先动 [左前|右后]后动
  for(uint32_t step=0;step<step_count;step++)
  {
    fcb_start(&_fcb, _frame_count);
    while (1)
    {
      //! FCB
      if( fcb_complete(&_fcb) ) {
        break;
      } else if( fcb_skip(&_fcb) ) {
        continue;
      } else {
        fcb_next(&_fcb);
      }
      //! Calculation
      if( fcb_current(&_fcb) <= _cuf_off_index ) 
      {
        T = DPI * ( (quad_fp)fcb_current(&_fcb) / (quad_fp)_cuf_off_index );
        // elog_t(TAG, "1 fcb_percentage = %f, T = %f", fcb_percentage(&_fcb), T);
        UNUSED(Tx);
        Tx = (T-sin(T)) / DPI; // x period = DPI
        Tz = (1-cos(T)) / 2.f; // z period = 2(sin(0~DPI))
        swing.X        = 0;
        swing.Z        = _swing_height * Tz;
        support.X      = 0;
        support.Z      = 0;
        swing          = coord_mapping(swing,_base);
        support        = coord_mapping(support,_base);
        coords[LEG_RF] = swing;
        coords[LEG_LF] = support;
        coords[LEG_RB] = support;
        coords[LEG_LB] = swing;
      } else {
        T = DPI * ( ( (quad_fp)fcb_current(&_fcb) - (quad_fp)_cuf_off_index) / (quad_fp)support_count );
        // elog_t(TAG, "2 fcb_percentage = %f, T = %f", fcb_percentage(&_fcb), T);
        UNUSED(Tx);
        Tx = (T-sin(T)) / DPI;
        Tz = (1-cos(T)) / 2.f;
        swing.X        = 0;
        swing.Z        = _swing_height * Tz;
        support.X      = 0;
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

  //踏步循环结束,移动至_base
  leg_sync_ccb_target_blocking(_base, CONFIG_SIP_READY_FRAME_COUNT);
}