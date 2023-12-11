#ifndef DEBUG_UTILS_H
#define DEBUG_UTILS_H

// dtoolbase.h defines the PUBLISHED macro if the CPPPARSER macro is defined
#include "dtoolbase.h"
#include "nodePath.h"

GeomNode* create_axes( float size = 0.5, bool bothways = false, float thickness = 2 );

void x_ray_node( NodePath node );

#endif
