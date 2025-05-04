#include "quad.h"

#define WALK_PERIOD       150.f
#define WALK_SWING_DUTY   0.5f
#define WALK_SWING_TIME   (WALK_PERIOD*WALK_SWING_DUTY)
#define WALK_SWING_WIDTH  35.f
#define WALK_SWING_HEIGHT 35.f
#define WALK_X_BASE       WALK_SWING_WIDTH/2.f
#define WALK_Z_BASE       110.f

typedef struct
{
  coord_t swingCoord;
  coord_t supportCoord;
}walk_coord_t;

typedef struct
{
  quad_float peried;
  quad_float swingDuty;
  quad_float swingTime;
  quad_float swingWidth;
  quad_float swingHeight;
  quad_float x_base;
  quad_float z_base;
  quad_float t_delta;

  leg_t* rfPtr;
  leg_t* rbPtr;
  leg_t* lfPtr;
  leg_t* lbPtr;

  coord_t rfCoord;
  coord_t rbCoord;
  coord_t lfCoord;
  coord_t lbCoord;

  volatile quad_float t;
  volatile walk_coord_t data;
} walk_t;

void walk_config(quad_float period, quad_float swing_duty, quad_float swing_width, quad_float swing_height, quad_float z_base);
void walk(int steps);
