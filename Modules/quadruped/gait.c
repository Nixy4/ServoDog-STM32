#include "quadruped_def.h"

inline QuadCoord _gait_walk(Gait* gait)
{
  QuadCoord QC;
  float T, Kx, Kz, x0, z0, x1, z1;
  float swingWidth      = gait->swingWidth;
  float swingHeight     = gait->swingHeight;
  float swingFrameCount = gait->swingFrameCount;
  float frameIndex      = gait->frameIndex;
  float frameCount      = gait->frameCount;
  
  if( frameIndex <= swingFrameCount )
  {
    T  = DPI*frameIndex/swingFrameCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    x0 = (Kx)   * swingWidth;
    z0 = (Kz)   * swingHeight;
    x1 = (1-Kx) * swingHeight;
    z1 = 0;
  }
  else if( frameIndex > swingFrameCount && frameIndex < frameCount)
  {
    T  = DPI*(frameIndex-swingFrameCount)/swingFrameCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    x0 = (1-Kx) * swingWidth;
    z0 = 0;
    x1 = (Kx)   * swingWidth;
    z1 = (Kz)   * swingHeight;
  }
  else
  {
    return (gait->current);
  }
  
  if(gait->timesIndex==gait->times)
  {
    //第一个周期 0组 腿不动
    x0 = gait->originalPoint.X;
    z0 = gait->originalPoint.Z;
    x1 = -x1 + gait->originalPoint.X;
    z1 = -z1 + gait->originalPoint.Z;      
  }
  else if(gait->timesIndex==1)
  {
    //最后一个周期 1组 腿不动
    x0 = -x0 + gait->originalPoint.X;
    z0 = -z0 + gait->originalPoint.Z;
    x1 = gait->originalPoint.X;
    z1 = gait->originalPoint.Z;  
  }
  else
  {
    //中间的周期
    x0 = -x0 + gait->originalPoint.X;
    z0 = -z0 + gait->originalPoint.Z;
    x1 = -x1 + gait->originalPoint.X;
    z1 = -z1 + gait->originalPoint.Z;
  }
  QC.orientation.rf = (Coord) { x0, z0 };
  QC.orientation.rb = (Coord) { x1, z1 };
  QC.orientation.lf = (Coord) { x1, z1 };
  QC.orientation.lb = (Coord) { x0, z0 };  
  return QC;
}

void gait_init(Gait* gait, const GaitConfig* cfg)
{
  gait->function      = cfg->function;
  gait->swingWidth    = cfg->swingWidth;
  gait->swingHeight   = cfg->swingHeight;
  gait->swingDuty     = cfg->swingDuty;
  gait->originalPoint = cfg->originalPoint;
  gait->frameCount    = cfg->frameCount;
  gait->frameInverval = cfg->frameInverval;
}

void gait_start(Gait* gait, uint16_t frames, int16_t times)
{
  gait->frameIndex   = 0;
  gait->frameCount   = frames;
  gait->frameInverval= 0;
  gait->times        = times;
  gait->timesIndex   = times;
  gait->current.orientation.rf = gait->originalPoint;
  gait->current.orientation.rb = gait->originalPoint;
  gait->current.orientation.lf = gait->originalPoint;
  gait->current.orientation.lb = gait->originalPoint;
}

bool gait_update(Gait* gait)
{
  if(gait->timesIndex==0) {
    return false;
  }
  gait->current = gait->function(gait);
  gait->frameIndex++;
  if(gait->frameIndex > gait->frameCount) {
    gait->frameIndex = 0;
    gait->timesIndex--;
  }
  return true;
}