#include "quadruped_def.h"

// Coords _easing_gait_walk_first(EasingGait* eg)
// {
//   float T, Kx, Kz, X, Z;
//   float frameIndex = (float)eg->frameIndex;
//   float frameCount = (float)eg->frameCount;
//   float frameSwingCount = frameCount * eg->swingDuty;
//   if( frameIndex <= frameSwingCount)
//   {
    
//     T = DPI*frameIndex/frameSwingCount;
//     Kx = (T-sin(T)) / DPI;
//     Kz = (1-cos(T)) / 2.f;
//     X = eg->swingWidth * Kx;
//     Z = eg->swingHeight * Kz;
//   }
//   else if( frameIndex > frameSwingCount && frameIndex < frameCount)
//   {
//     T = DPI*(frameIndex-frameSwingCount)/frameSwingCount;
//     Kx = (T-sin(T)) / DPI;
//     Kz = (1-cos(T)) / 2.f;
//     X = eg->swingWidth - eg->swingWidth * Kx;
//     Z = 0;
//   }
//   else
//   {
//     X = eg->current.X;
//     Z = eg->current.Z;
//   }
  
//   X = -X + eg->offset.X;
//   Z = -Z + eg->offset.Z;

//   return (Coord) {X, Z};
// }

// Coords _easing_gait_walk_second(EasingGait* eg)
// {
//   float T, Kx, Kz, X, Z;
//   float frameIndex = (float)eg->frameIndex;
//   float frameCount = (float)eg->frameCount;
//   float frameSwingCount = frameCount * eg->swingDuty;
//   if( frameIndex <= frameSwingCount)
//   {
//     T = DPI*frameIndex/frameSwingCount;
//     Kx = (T-sin(T)) / DPI;
//     Kz = (1-cos(T)) / 2.f;
//     X = eg->swingWidth - eg->swingWidth * Kx;
//     Z = eg->swingHeight * Kz;
//   }
//   else if( frameIndex > frameSwingCount && frameIndex < frameCount)
//   {
//     T = DPI*(frameIndex-frameSwingCount)/frameSwingCount;
//     Kx = (T-sin(T)) / DPI;
//     Kz = (1-cos(T)) / 2.f;
//     X = eg->swingWidth * Kx;
//     Z = 0;
//   }
//   else
//   {
//     X = eg->current.X;
//     Z = eg->current.Z;
//   }

//   X = -X + eg->offset.X;
//   Z = -Z + eg->offset.Z;

//   return (Coord) {X, Z};
// }