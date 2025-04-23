#include "quadruped_def.h"

QuadCoord _quad_gait_start_walk(Gait* qg)
{
  QuadCoord QC = {
    .orientation = {
      .rf = qg->offset,
      .rb = qg->offset,
      .lf = qg->offset,
      .lb = qg->offset
    }
  };
  leg_ecoord_target(LEG_ID_RF, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_target(LEG_ID_RB, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_target(LEG_ID_LF, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_target(LEG_ID_LB, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_update_all_block();
  return QC;
}

inline QuadCoord _quad_gait_period_walk(Gait* qg)
{
  QuadCoord QC;

  float T, Kx, Kz, x0, z0, x1, z1;
  float swingWidth = qg->swingWidth;
  float swingHeight = qg->swingHeight;
  float frameIndex = (float)qg->frameIndex;
  float frameCount = (float)qg->frameCount;
  float frameSwingCount = frameCount * qg->swingDuty;

  if( frameIndex <= frameSwingCount )
  {
    T  = DPI*frameIndex/frameSwingCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    x0 = (Kx)   * swingWidth;
    z0 = (Kz)   * swingHeight;
    x1 = (1-Kx) * swingHeight;
    z1 = 0;
  }
  else if( frameIndex > frameSwingCount && frameIndex < frameCount)
  {
    T  = DPI*(frameIndex-frameSwingCount)/frameSwingCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    x0 = (1-Kx) * swingWidth;
    z0 = 0;
    x1 = (Kx)   * swingWidth;
    z1 = (Kz)   * swingHeight;
  }
  else
  {
    QC = qg->current;
    return QC;
  }
  
  x0 = -x0 + qg->offset.X;
  z0 = -z0 + qg->offset.Z;
  x1 = -x1 + qg->offset.X;
  z1 = -z1 + qg->offset.Z;

  QC.orientation.rf = (Coord) { x0, z0 };
  QC.orientation.rb = (Coord) { x1, z1 };
  QC.orientation.lf = (Coord) { x1, z1 };
  QC.orientation.lb = (Coord) { x0, z0 };

  return QC;
}

QuadCoord _quad_gait_stop_walk(Gait* qg)
{
  QuadCoord QC = {
    .orientation = {
      .rf = qg->offset,
      .rb = qg->offset,
      .lf = qg->offset,
      .lb = qg->offset
    }
  };
  leg_ecoord_target(LEG_ID_RF, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_target(LEG_ID_RB, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_target(LEG_ID_LF, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_target(LEG_ID_LB, qg->offset.X, qg->offset.Z, qg->frameCount);
  leg_ecoord_update_all_block();
  return QC;
}

void quadruped_gait_init(Gait* qg, const GaitConfig* cfg)
{
  qg->startFunc = cfg->startFunc;
  qg->periodFunc = cfg->periodFunc;
  qg->stopFunc = cfg->stopFunc;

  qg->swingWidth = cfg->swingWidth;
  qg->swingHeight = cfg->swingHeight;
  qg->swingDuty = cfg->swingDuty;

  qg->offset = cfg->offset;

  qg->frameCount = cfg->frameCount;
  qg->frameInverval = cfg->frameInverval;
}

static inline bool _quadruped_gait_update(Gait* qg)
{
  QuadCoord QC;
  
  if(qg->times == 0) {
    qg->stopFunc(qg);
    return false;
  }

  // if(qg->frameIndex == 0) {
  //   QC = qg->startFunc(qg);
  // } else if(qg->frameIndex < qg->frameCount) {
  //  QC = qg->periodFunc(qg);
  // } 
  // else if (qg->frameIndex == qg->frameCount) {
  //   QC = qg->stopFunc(qg);
  // }
  // qg->current = QC;

  QC =  _quad_gait_period_walk(qg);

  qg->frameIndex++;

  if(qg->frameIndex > qg->frameCount) {
    qg->frameIndex = 0;
    qg->times--;
  }
  leg_set_coord(LEG_ID_RF, QC.orientation.rf.X, QC.orientation.rf.Z);
  leg_set_coord(LEG_ID_RB, QC.orientation.rb.X, QC.orientation.rb.Z);
  leg_set_coord(LEG_ID_LF, QC.orientation.lf.X, QC.orientation.lf.Z);
  leg_set_coord(LEG_ID_LB, QC.orientation.lb.X, QC.orientation.lb.Z);
  return true;
}

inline void quadruped_gait_operation0(Gait* qg, uint16_t frames)
{
  qg->frameIndex = 0;
  qg->times = 1;
  qg->frameCount = frames;
  qg->frameInverval = 0;
}

inline void quadruped_gait_operation1(Gait* qg, uint16_t frames, int16_t times)
{
  qg->frameIndex = 0;
  qg->times = times;
  qg->frameCount = frames;
  qg->frameInverval = 0;
  qg->startFunc(qg);
}

void quadruped_gait_operation0_block(Gait* qg, uint16_t frames)
{
  quadruped_gait_operation0(qg, frames);
  while(_quadruped_gait_update(qg) != false);
}

void quadruped_gait_operation1_block(Gait* qg, uint16_t frames, int16_t times)
{
  quadruped_gait_operation1(qg, frames, times);
  while(_quadruped_gait_update(qg) != false);
}