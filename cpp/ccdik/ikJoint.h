
#ifndef IKJOINT_H
#define IKJOINT_H

// dtoolbase.h defines the PUBLISHED macro if the CPPPARSER macro is defined
#include "dtoolbase.h"

#include "nodePath.h"
#include "characterJoint.h"
#include "lvector4.h"

#include <list>
#include <math.h>

class EXPORT_CLASS IKJoint {
PUBLISHED:
  // These methods are publicly accessible to Python and C++

  IKJoint( CharacterJoint* joint, IKJoint* parent=nullptr, bool is_static = false );
  ~IKJoint();

  void set_static( bool is_static = true ) { this->is_static = is_static; }
  bool get_static() { return this->is_static; }
  void set_control_node( NodePath control_node ) { this->control_node = control_node; }
  std::string get_name();

  void set_hinge_constraint( LVector3f axis, float min_ang = -M_PI, float max_ang = M_PI );
  void set_ball_constraint( float min_ang = -M_PI, float max_ang = M_PI );

  bool get_has_rotation_axis() { return this->has_rotation_axis; }
  LVector3f get_axis() { return this->axis; }

  float get_min_ang() { return this->min_ang; }
  float get_max_ang() { return this->max_ang; }

  LVector4f get_col() { return this->col; }

  NodePath get_control_node() { return this->control_node; }
  NodePath get_debug_node();

  IKJoint* get_parent() { return this->parent; }

public:
  // C++-only methods:
  
  void add_child( IKJoint* bone );
  void remove_child( IKJoint* bone );
  void remove_parent();

private:
  // The joint this ik_joint controls/represents:
  CharacterJoint* joint;
  IKJoint* parent;
  LVector3f axis;
  float min_ang;
  float max_ang;
  bool is_static;
  bool has_rotation_axis;

  NodePath control_node;
  NodePath debug_node;

  std::list<IKJoint*> children;

  LVector4f col;
};

#endif
