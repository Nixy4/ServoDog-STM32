#pragma once

typedef double quad_float;

typedef struct
{
  quad_float thigh;
  quad_float shank;
} angle_t;

typedef struct
{
  quad_float x;
  quad_float z;
} coord_t;

#define PI        3.141592653589793f
#define DOUBLE_PI 6.283185307179586f
#define HALF_PI   1.570796326794897f

#define DEG_TO_RAD(deg) ((deg) * 0.01745329251994329547f)
#define RAD_TO_DEG(rad) ((rad) * 57.29577951308232286465f)