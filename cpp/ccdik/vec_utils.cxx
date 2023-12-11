#include "vec_utils.h"


void swing_twist_decomposition(
    LQuaternionf rotation,
    LVector3f twist_axis,
    LQuaternionf &swing,
    LQuaternionf &twist )
{
  LVector3f ra( rotation.get_i(), rotation.get_j(), rotation.get_k() );
  LVector3f p = ra.project( twist_axis );
  twist.set( rotation.get_r(), p.get_x(), p.get_y(), p.get_z() );
  twist.normalize();
  swing = rotation * twist.conjugate();
}

LVector3f get_perpendicular_vec( LVector3f vec )
{
    vec = vec.normalized();
    LVector3f vec2 = LVector3f::unit_y();

    float ang = vec.angle_deg( vec2 );
    if( ang < 0.001 or ang > 179.999 )       // Parallel?
    {
      // Choose a different vector:
      vec2 = LVector3f::unit_x();
    }

    return vec.cross(vec2).normalized();
}


