#pragma once

#define QUAD_TYPE double

typedef struct
{
  QUAD_TYPE thigh;
  QUAD_TYPE shank;
} angle_t;

typedef struct
{
  QUAD_TYPE x;
  QUAD_TYPE z;
} coord_t;

#define PI   3.141592653589793f
#define _2PI 6.283185307179586f
#define PI2_ 1.570796326794897f

#define DEG_TO_RAD(deg) ((deg) * 0.01745329251994329547f)
#define RAD_TO_DEG(rad) ((rad) * 57.29577951308232286465f)