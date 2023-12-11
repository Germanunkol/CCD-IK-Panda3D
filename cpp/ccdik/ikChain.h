
#ifndef IKCHAIN_H
#define IKCHAIN_H

// dtoolbase.h defines the PUBLISHED macro if the CPPPARSER macro is defined
#include "dtoolbase.h"
#include <string>
#include <list>
#include <algorithm>
#include "ikJoint.h"
#include "nodePath.h"
#include "characterJoint.h"

class EXPORT_CLASS IKChain {
  /* Class to handle an IK chain.
   *
   * To set up manually
   * 1. Create the IKChain
   * 2. Add all joints using add_joint()
   * 3. Optionally add constraints on the created IKJoints.
   *
   * Alternatively, use the armature utils (Python only!) to set up joints.
   *
   * Finally, remember to set a target for the chain using set_target().
   */
PUBLISHED:
  // These methods are publicly accessible to Python and C++

  IKChain();
  ~IKChain();

  IKJoint* add_joint( CharacterJoint* joint, NodePath control_node, IKJoint* parent_ik_joint = nullptr,
      bool is_static = false );

  IKJoint* get_ik_joint( std::string joint_name );
  IKJoint* get_ik_joint( unsigned int id );

  size_t get_num_ik_joints() { return this->ik_joints.size(); }

  void set_annealing_exponent( int exp ) { this->annealing_exponent = std::max( exp, 0 ); }
  int get_annealing_exponent() { return this->annealing_exponent; }
  void set_target( NodePath target ) { this->target = target; }

  void update_ik( float threshold = 1e-2, int min_iterations = 0, int max_iterations = 10 );

  void debug_display( float line_length=0.2, float thickness=2, bool draw_constraints = true, bool x_ray = true );

  void remove_debug_display();

  float calc_length();

  NodePath get_end_effector();

public:
  // C++-only methods:
  
private:

  int annealing_exponent;
  NodePath target;
  NodePath root;
  NodePath end_effector;

  bool target_reached;
  bool debug_display_enabled;

  void inverse_kinematics_ccd( float threshold = 1e-2, int min_iterations = 1, int max_iterations = 10 );

  std::vector<IKJoint*> ik_joints;
};

#endif
