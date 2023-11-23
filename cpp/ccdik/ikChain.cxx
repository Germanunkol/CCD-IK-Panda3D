#include "ikChain.h"
#include "debug_utils.h"
#include <stdio.h>
#include "lquaternion.h"
#include "lvector3.h"
#include "lpoint3.h"
#include "vec_utils.h"
#include "lineSegs.h"

IKChain::IKChain()
{
  this->annealing_exponent = 0;
  this->debug_display_enabled = false;
  this->target_reached = false;
}

IKChain::~IKChain()
{
  for( size_t i = 0; i < this->ik_joints.size(); i++ )
  {
    delete this->ik_joints[i];
  }
}

IKJoint* IKChain::add_joint( CharacterJoint* joint, NodePath control_node, IKJoint* parent_ik_joint,
    bool is_static )
{
  std::string name = joint->get_name();

  IKJoint* ik_joint = new IKJoint( joint, parent_ik_joint, is_static );
  ik_joint->set_control_node( control_node );

  if( parent_ik_joint != nullptr )
  {
    control_node.reparent_to( parent_ik_joint->get_control_node() );
  }

  this->ik_joints.push_back( ik_joint );

  if( !this->root )
  {
    assert( control_node.get_parent() && "Please connect node to scene graph before adding to joint!" );
    this->root = control_node.get_parent();
  }

  if( this->debug_display_enabled )
    this->debug_display();

  return ik_joint;
}

IKJoint* IKChain::get_ik_joint( std::string joint_name )
{
  for( size_t i = 0; i < this->ik_joints.size(); i++ )
  {
    if( this->ik_joints[i]->get_name() == joint_name )
      return this->ik_joints[i];
  }
  return nullptr;
}

void IKChain::update_ik( float threshold, int min_iterations, int max_iterations )
{
  // Solve IK chain for current target:
  this->inverse_kinematics_ccd( threshold, min_iterations, max_iterations );

  // Apply result (TODO: Still necessary?)
  for( auto it = this->ik_joints.begin(); it != this->ik_joints.end(); it ++ )
  {
    NodePath node = (*it)->get_control_node();
    node.set_quat( node.get_quat() );
  }
}

void IKChain::inverse_kinematics_ccd( float threshold, int min_iterations, int max_iterations )
{

  assert( this->ik_joints.size() > 0 && "IK only works when chain has at least one joint!" );

  assert( this->target && "IK target must be set!" );

  if( ! this->end_effector )
    this->end_effector = this->ik_joints.back()->get_control_node().attach_new_node( "End_effector" );

  this->target_reached = false;
  for( int i = 0; i < max_iterations; i++ )
  {
    if( i > min_iterations )
    {
      float err = (this->target.get_pos( this->root ) - this->end_effector.get_pos( this->root )).length();
      if( err < threshold )
      {
        this->target_reached = true;
        break;
      }
    }

    for( size_t j = 0; j < this->ik_joints.size() -1; j++ )
    {
      // Iterate backwards:
      IKJoint* ik_joint = this->ik_joints[this->ik_joints.size() - j - 1];


      // Skip over static joints:
      if( ik_joint->get_static() )
        continue;

      NodePath ik_joint_node = ik_joint->get_control_node();
      NodePath parent_node = (ik_joint->get_parent() != nullptr) ? \
                             ik_joint->get_parent()->get_control_node() : \
                             this->root;


      // The following is computed in local space.
      // First, get the target's position in the local space of this joint
      LPoint3 target = this->target.get_pos( ik_joint_node );

      // Then get the position of this node in local space always (0,0,0) and the 
      // current position of the end effector in current space:
      //LPoint3 pos = LPoint3( 0,0,0 );
      LPoint3 ee = this->end_effector.get_pos( ik_joint_node );

      // Get the direction to the target and the direction to the end effector
      // (all still in local space). These are the two vectors we want to align,
      // i.e. we want to rotate the joint so that the direction to the end effector
      // is the direction to the target.
      LVector3f d1 = target;
      LVector3f d2 = ee;

      // 
      LVector3f cross = d1.cross( d2 ).normalized();
      if( cross.length_squared() < 1e-9 )
        continue;

      float ang = d2.normalized().signed_angle_rad( d1.normalized(), cross );
      LQuaternionf q(0,0,0,0);
      q.set_from_axis_angle_rad( ang, cross );
      // Add this rotation to the current rotation:
      LQuaternionf q_old = ik_joint_node.get_quat();
      LQuaternionf q_new = q*q_old;
      q_new.normalize();

      // Correct rotation for hinge:
      if( ik_joint->get_has_rotation_axis() )
      {
        LVector3f my_axis_in_parent_space = ik_joint->get_axis();
        LQuaternionf swing, twist;
        swing_twist_decomposition( q_new, -my_axis_in_parent_space, swing, twist );
        // Only keep the part of the rotation over the hinge axis:
        q_new = twist;
      }

      LVector3f rot_axis = q_new.get_axis_normalized();

      float rot_ang = q_new.get_angle_rad();

      // Valid rotation?
      if( rot_axis.length_squared() > 1e-3 && !std::isnan( rot_ang ) and abs( rot_ang ) > 0 )
      {
        // reduce the angle
        //rot_ang = rot_ang % float(M_PI*2);
        // Force into the correct range, so that -180 < angle < 180:
        while(rot_ang > M_PI)
        {
          rot_ang -= 2*M_PI;
        }
          
        if( abs(rot_ang) > 1e-6 and abs(rot_ang) < M_PI*2 )   // Still necessary?
        {
          // Clamp rotation angle:
          if( ik_joint->get_has_rotation_axis() && (rot_axis - ik_joint->get_axis()).length_squared() > 0.5 )
            rot_ang = std::max( -ik_joint->get_max_ang(), std::min( -ik_joint->get_min_ang(), rot_ang ) );
          else
            rot_ang = std::max( ik_joint->get_min_ang(), std::min( ik_joint->get_max_ang(), rot_ang ) );
        }

        q_new.set_from_axis_angle_rad( rot_ang, rot_axis );

        float ik_joint_factor = float(j+1)/float(this->ik_joints.size()-1);
        float annealing = pow( ik_joint_factor, this->annealing_exponent );
        q_new = q_old + (q_new-q_old)*annealing;

        ik_joint_node.set_quat( q_new );

      }
    }
  }
}

void IKChain::debug_display( float line_length, float thickness )
{
  this->remove_debug_display();
  this->debug_display_enabled = true;

  if( !this->root )
    return;

  GeomNode* axes_geom = create_axes( 0.15 );


  for( auto it = this->ik_joints.begin(); it != this->ik_joints.end(); it ++ )
  {
    IKJoint* ik_joint = (*it);
    /* Attach a new node to the ik_node. All debug info will be attached to this node.
    This is only for cleaner removal of the node later on - by remuving the debug node,
    all debug info will be cleared. Otherwise this node has no significance and we could
    just as well attach everything to the ik_node itself
    */
    NodePath debug_node = ik_joint->get_debug_node();
    debug_node.set_light_off(1);

    //PandaNode* p_node = new PandaNode( axes_geom );
    debug_node.attach_new_node( (PandaNode*)axes_geom );

    NodePath parent_node = (ik_joint->get_parent() != nullptr) ? \
                             ik_joint->get_parent()->get_control_node() : \
                             this->root;

    // Again, use the parent's debug node rather than attaching stuff to the ik_node directly,
    // so we can remove the debug info easily later on by removing the debug node.
    assert( parent_node );
    NodePath parent_debug_node = parent_node.find("Debug_display");
    if( !parent_debug_node )
    {
      parent_debug_node = parent_node.attach_new_node("Debug_display");
      parent_debug_node.set_light_off(1);
    }

    // Draw my offset in parent space
    LineSegs lines;
    lines.set_thickness( thickness );
    lines.set_color( ik_joint->get_col() );
    lines.move_to( 0, 0, 0 );
    LPoint3 my_pos = ik_joint->get_control_node().get_pos( parent_node );
    lines.draw_to( my_pos );
    GeomNode* geom = lines.create();
    parent_debug_node.attach_new_node( (PandaNode*)geom );
  }
}

void IKChain::remove_debug_display()
{
  if( !this->root )
    return;

  for( auto it = this->ik_joints.begin(); it != this->ik_joints.end(); it ++ )
  {
    IKJoint* ik_joint = (*it);

    NodePath parent_node = (ik_joint->get_parent() != nullptr) ? \
                             ik_joint->get_parent()->get_control_node() : \
                             this->root;
    NodePath parent_debug_node = parent_node.find("Debug_display");
    if( parent_debug_node )
      parent_debug_node.remove_node();
  }
}


