#include "EasingCoordWalk.h"
#include "math.h"

EasingCoordWalkCustomData gEasingCoordWalkCustomData = {0.5f};

// EasingCoordConfig walk_first = 
// {
//   .mode = EASING_MODE_DEFAULT|EASING_MODE_NTIMES(10),
//   .function = _easing_coord_walk_first,
//   .frameCount = 500,
//   .interval = 0,
//   .customData = &gEasingCoordWalkCustomData
// };

// EasingCoordConfig walk_second = 
// {
//   .mode = EASING_MODE_DEFAULT|EASING_MODE_NTIMES(10),
//   .function = _easing_coord_walk_second,
//   .frameCount = 500,
//   .interval = 0,
//   .customData = &gEasingCoordWalkCustomData
// };

Coord _easing_coord_walk_first(EasingCoord* ec)
{
  EasingCoordWalkCustomData* customData = ec->customData;
  float T, Kx, Kz, X, Z;
  float frameIndex = (float)ec->frameIndex;
  float frameCount = (float)ec->frameCount;
  float frameSwingCount = frameCount * customData->swingDuty;
  // float deltaX = ec->stop.X - ec->start.X;
  // float deltaZ = ec->stop.Z - ec->start.Z;
  float deltaX = 1.0f;
  float deltaZ = 1.0f;
  if( frameIndex <= frameSwingCount)
  {
    T = DPI*frameIndex/frameSwingCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    X = deltaX * Kx;
    Z = deltaZ * Kz;
  }
  else if( frameIndex > frameSwingCount && frameIndex < frameCount)
  {
    T = DPI*(frameIndex-frameSwingCount)/frameSwingCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    X = deltaX - deltaX * Kx;
    Z = 0;
  }
  else
  {
    X = ec->stop.X;
    Z = ec->stop.Z;
  }
  return (Coord) {X, Z};
}

Coord _easing_coord_walk_second(EasingCoord* ec)
{
  EasingCoordWalkCustomData* customData = ec->customData;
  float T, Kx, Kz, X, Z;
  float frameIndex = (float)ec->frameIndex;
  float frameCount = (float)ec->frameCount;
  float frameSwingCount = frameCount * customData->swingDuty;
  // float deltaX = ec->stop.X - ec->start.X;
  // float deltaZ = ec->stop.Z - ec->start.Z;
  float deltaX = 1.0f;
  float deltaZ = 1.0f;
  if( frameIndex <= frameSwingCount)
  {
    T = DPI*frameIndex/frameSwingCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    X = deltaX - deltaX * Kx;
    Z = deltaZ * Kz;
  }
  else if( frameIndex > frameSwingCount && frameIndex < frameCount)
  {
    T = DPI*(frameIndex-frameSwingCount)/frameSwingCount;
    Kx = (T-sin(T)) / DPI;
    Kz = (1-cos(T)) / 2.f;
    X = deltaX * Kx;
    Z = 0;
  }
  else
  {
    X = ec->stop.X;
    Z = ec->stop.Z;
  }
  return (Coord) {X, Z};
}