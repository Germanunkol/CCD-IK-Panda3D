#ifndef VEC_UTILS_H
#define VEC_UTILS_H

#include "lquaternion.h"
#include "lvector3.h"

void swing_twist_decomposition(
    LQuaternionf rotation,
    LVector3f twist_axis,
    LQuaternionf &swing,
    LQuaternionf &twist );
#endif
