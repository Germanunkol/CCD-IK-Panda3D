#include "lineSegs.h"
GeomNode* create_axes( float size, bool bothways, float thickness )
{
  LineSegs lines;
  lines.set_thickness( thickness );

  lines.set_color( 1,0.1,0.1,0.1 );
  if( bothways ){
    lines.move_to( -size, 0, 0 );
  } else {
    lines.move_to( 0, 0, 0 );
  }
  lines.draw_to( size, 0, 0 );

  lines.set_color( 0.1,1,0.1,0.1 );
  if( bothways ){
    lines.move_to( 0, -size, 0 );
  } else {
    lines.move_to( 0, 0, 0 );
  }
  lines.draw_to( 0, size, 0 );

  lines.set_color( 0.1,0.1,1,0.1 );
  if( bothways ){
    lines.move_to( 0, 0, -size );
  } else {
    lines.move_to( 0, 0, 0 );
  }
  lines.draw_to( 0, 0, size );

  return lines.create();
}

void x_ray_node( NodePath node )
{
  node.set_bin("fixed", 0);
  node.set_depth_test(false);
  node.set_depth_write(false);
}


