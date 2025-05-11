#include "quadruped.h"

quad_kine _kine[LEG_COUNT];

quad_fcb _acb_fcb[SERVO_COUNT];
quad_fcb _ccb_fcb[LEG_COUNT];
quad_acb _acb[SERVO_COUNT];
quad_ccb _ccb[LEG_COUNT];

quad_fcb _sync_acb_fcb;
quad_fcb _sync_ccb_fcb;
quad_acb _sync_acb;
quad_ccb _sync_ccb;

