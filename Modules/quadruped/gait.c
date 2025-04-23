#include "quadruped_def.h"

#define TAG "Gait"

void _gait_walk(Gait* gait)
{
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
    x1 = swingWidth - swingWidth * Kx;
    z1 = 0;
  }
  else if( frameIndex > swingFrameCount && frameIndex < frameCount)
  {
    T  = DPI*(frameIndex-swingFrameCount)/swingFrameCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    x0 = swingWidth - swingWidth * Kx; 
    z0 = 0;
    x1 = (Kx)   * swingWidth;
    z1 = (Kz)   * swingHeight;
  }
  else
  {
    return;
  }
  
  // if(gait->timesIndex==gait->times)
  // {
  //   //第一个周期 0组 腿不动
  //   x0 = gait->originalPoint.X;
  //   z0 = gait->originalPoint.Z;
  //   x1 = -x1 + gait->originalPoint.X;
  //   z1 = -z1 + gait->originalPoint.Z;
  // }
  // else if(gait->timesIndex==1)
  // {
  //   //最后一个周期 1组 腿不动
  //   x0 = -x0 + gait->originalPoint.X;
  //   z0 = -z0 + gait->originalPoint.Z;
  //   x1 = gait->originalPoint.X;
  //   z1 = gait->originalPoint.Z;  
  // }
  // else
  // {
    //中间的周期
    x0 = -x0 + gait->originalPoint.X;
    z0 = -z0 + gait->originalPoint.Z;
    x1 = -x1 + gait->originalPoint.X;
    z1 = -z1 + gait->originalPoint.Z;
  // }

  elog_v(TAG,"| %15s : %15s | %15s : %15s |", "----------", "----------", "----------", "----------");
  elog_v(TAG,"| %15s : %10.5f | %15s : %10.5f |", "swingWidth", swingWidth, "swingHeight", swingHeight);
  elog_v(TAG,"| %15s : %10.5f | %15s : %15s |", "swingDuty", gait->swingDuty, "", "");
  elog_v(TAG,"| %15s : %10.5f | %15s : %10.5f |", "frameIndex", frameIndex, "frameCount", frameCount);
  elog_v(TAG,"| %15s : %10d | %15s : %10d |", "timesIndex", gait->timesIndex, "times", gait->times);
  elog_v(TAG,"| %15s : %10.5f | %15s : %10.5f |", "x0", x0, "z0", z0);
  elog_v(TAG,"| %15s : %10.5f | %15s : %10.5f |", "x1", x1, "z1", z1);
  elog_v(TAG,"| %15s : %10.5f | %15s : %10.5f |", "Kx", Kx, "Kz", Kz);

  gait->current.orientation.rf = (Coord){.X = x0, .Z = z0};
  gait->current.orientation.rb = (Coord){.X = x1, .Z = z1};
  gait->current.orientation.lf = (Coord){.X = x1, .Z = z1};
  gait->current.orientation.lb = (Coord){.X = x0, .Z = z0};
}

void gait_init(Gait* gait, const GaitConfig* cfg)
{
  gait->function        = cfg->function;

  gait->swingWidth      = cfg->swingWidth;
  gait->swingHeight     = cfg->swingHeight;
  gait->swingDuty       = cfg->swingDuty;
  // gait->swingFrameCount = cfg->frameCount * cfg->swingDuty;

  gait->originalPoint   = cfg->originalPoint;
  
  gait->frameCount      = cfg->frameCount;
  gait->frameInverval   = cfg->frameInverval;

  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "swingWidth", gait->swingWidth,             "swingHeight", gait->swingHeight);
  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "swingDuty", gait->swingDuty,               "swingFrameCount", gait->swingFrameCount);
  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "frameCount", gait->frameCount,             "frameInverval", gait->frameInverval);
  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "originalPoint.X", gait->originalPoint.X,   "originalPoint.Z", gait->originalPoint.Z);
  elog_i(TAG,"| %15s : %10d | %15s : %10d |",     "timesIndex", gait->timesIndex,             "times", cfg->times);
  elog_i(TAG,"| %15s : %10d | %15s : %10.5f |",     "frameIndex", gait->frameIndex,             "frameCount", gait->frameCount);
  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "Kx", 0.f, "Kz", 0.f);
  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "x0", 0.f, "z0", 0.f);
  elog_i(TAG,"| %15s : %10.5f | %15s : %10.5f |", "x1", 0.f, "z1", 0.f);
}

void gait_start(Gait* gait, uint16_t frames, int16_t times)
{
  gait->frameIndex   = 0;
  gait->frameCount   = frames;
  gait->frameInverval= 1;
  gait->times        = times;
  gait->timesIndex   = times;
  gait->lastTick     = 0;
  gait->swingFrameCount = frames * gait->swingDuty;
  gait->current.orientation.rf = gait->originalPoint;
  gait->current.orientation.rb = gait->originalPoint;
  gait->current.orientation.lf = gait->originalPoint;
  gait->current.orientation.lb = gait->originalPoint;

}

bool gait_update(Gait* gait)
{
  if(gait->frameInverval > 0) {
    if(HAL_GetTick() - gait->lastTick < gait->frameInverval) {
      return true;
    }
  }

  if(gait->timesIndex==0) {
    return false;
  }
  _gait_walk(gait);
  gait->frameIndex++;
  if(gait->frameIndex > gait->frameCount) {
    gait->frameIndex = 0;
    gait->timesIndex--;
  }

  if(gait->frameInverval > 0) {
    gait->lastTick = HAL_GetTick();
  }
  return true;
}