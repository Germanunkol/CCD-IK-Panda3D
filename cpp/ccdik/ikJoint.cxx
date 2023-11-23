#include "ikJoint.h"
#include <math.h>
#include <algorithm>

IKJoint::IKJoint( CharacterJoint* joint, IKJoint* parent, bool is_static )
{
  this->joint = joint;
  this->parent = parent;

  this->is_static = is_static;
  this->axis = LVector3f( 0, 0, 0 );
  this->has_rotation_axis = false;
  this->min_ang = 0;
  this->max_ang = M_PI*2;

  this->col = LVector4f( 0.2, 0.2, 0.7, 1 );

  if( parent != nullptr )
    parent->add_child( this );

}

IKJoint::~IKJoint()
{
  if( this->parent != nullptr )
    this->parent->remove_child( this );

  for( auto it = this->children.begin(); it != this->children.end(); it++ )
    (*it)->remove_parent();
}

std::string IKJoint::get_name()
{
  return this->joint->get_name();
}

void IKJoint::add_child( IKJoint* child )
{
  bool found = (std::find(this->children.begin(), this->children.end(), child) != this->children.end());
  if( !found )
    this->children.push_back( child );
}

void IKJoint::remove_child( IKJoint* child )
{
  this->children.remove( child );
}

void IKJoint::remove_parent()
{
  this->parent = nullptr;
}

void IKJoint::set_hinge_constraint( LVector3f axis, float min_ang, float max_ang )
{
  this->axis = axis.normalized();
  this->min_ang = min_ang;
  this->max_ang = max_ang;
  this->has_rotation_axis = true;
}

void IKJoint::set_ball_constraint( float min_ang, float max_ang )
{
  this->min_ang = min_ang;
  this->max_ang = max_ang;
  this->has_rotation_axis = false;
}

NodePath IKJoint::get_debug_node()
{
  if( !this->debug_node )
    this->debug_node = this->control_node.attach_new_node("Debug_Display");

  return this->debug_node;
}
