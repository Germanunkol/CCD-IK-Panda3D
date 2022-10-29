from panda3d.core import *
from direct.actor.Actor import Actor
import random, math
from .bone import Bone
from .utils import *
from .vec_utils import *

class IKChain():

    def __init__( self, actor ):

        # We need an actor to be able to control (and expose) joints. If it already exists,
        # likely because the model was loaded from a file - great then just use that.
        # However, if it doesn't exist, this means we first need to set up all the joints before
        # we create the actor, so keep it empty for now!
        self.actor = actor

        self.bones = []
        self.target = None
        self.target_reached = False

        self.debug_display_enabled = False

        self.end_effector = None


    def add_joint( self, joint, control_node, parent_bone=None, static=False ):

        #if parent_bone:
        #    parent_ik_node = parent_bone.control_node
        #else:
        #    parent_ik_node = self.actor
        
        name = joint.get_name()

        bone = Bone( joint, parent=parent_bone, static=static )
        bone.control_node = control_node

        if parent_bone:
            control_node.reparent_to( parent_bone.control_node )

        self.bones.append(bone)

        return bone

    def get_bone( self, joint_name ):
        for b in self.bones:
            if b.joint.get_name() == joint_name:
                return b
        raise Exception(f"Cannot find joint {joint_name}!")

    def set_static( self, joint_name, static=True ):
        b = self.get_bone( joint_name )
        b.static = static

    def set_hinge_constraint( self, joint_name, axis, min_ang=-math.pi, max_ang=math.pi ):
        b = self.get_bone( joint_name )
        b.axis = axis.normalized()
        b.min_ang = min_ang
        b.max_ang = max_ang

        if self.debug_display_enabled:
            self.debug_display()

    def set_ball_constraint( self, joint_name, min_ang=-math.pi, max_ang=math.pi ):
        b = self.get_bone( joint_name )
        b.axis = None
        b.min_ang = min_ang
        b.max_ang = max_ang

        if self.debug_display_enabled:
            self.debug_display()

    def update_ik( self, threshold = 1e-2, min_iterations=1, max_iterations=10 ):

        assert len(self.bones) > 0, "IKChain requires at least one bone for update_i_k() to work!"

        # Solve the IK chain for the IK nodes:
        if self.target:
            self.inverse_kinematics_cCD( threshold, min_iterations, max_iterations )

        # Copy the data from the IK chain to the actual bones.
        # This will end up affecting the actual mesh.
        for bone in self.bones:
            bone.control_node.set_quat( bone.control_node.get_quat() )

    def inverse_kinematics_cCD( self, threshold = 1e-2, min_iterations=1, max_iterations=10 ):

        if not self.end_effector:
            self.end_effector = self.bones[-1].control_node.attach_new_node( "End_effector" )

        self.target_reached = False
        for i in range(max_iterations):

            if i >= min_iterations:
                err = (self.target.get_pos(render)-self.end_effector.get_pos(render)).length()
                if err < threshold:
                    self.target_reached = True
                    break

            for j in range(len(self.bones)-1):
                bone = self.bones[-j-2]

                if bone.static:
                    continue

                bone_node = bone.control_node
                if bone.parent:
                    parent_node = bone.parent.control_node
                else:
                    parent_node = self.actor

                target = self.target.get_pos( bone_node )

                pos = LPoint3.zero()
                ee = self.end_effector.get_pos( bone_node )

                d1 = target-pos
                d2 = ee-pos

                cross = d1.cross(d2).normalized()
                if cross.length_squared() < 1e-9:
                    continue

                ang = d2.normalized().signed_angle_rad( d1.normalized(), cross )
                q = Quat()
                q.set_from_axis_angle_rad( ang, cross )
                # Add this rotation to the current rotation:
                q_old = bone_node.get_quat()
                q_new = q*q_old
                q_new.normalize()
                #bone_node.set_quat( q_new )

                # Correct rotation for hinge:
                if bone.axis:
                    #q_inv = bone_node.get_quat()
                    #q_inv.invert_in_place()
                    #my_axis_in_parent_space = q_inv.xform( bone.axis )
                    my_axis_in_parent_space = bone.axis
                    swing, twist = swing_twist_decomposition( q_new, -my_axis_in_parent_space )
                    q_new = twist

                rot_axis = q_new.get_axis()
                rot_axis.normalize()
                ang = q_new.get_angle_rad()
                if rot_axis.length_squared() > 1e-3 and not math.isnan(ang) and abs(ang) > 0: # valid rotation axis?
                    # reduce the angle  
                    ang = ang % (math.pi*2)
                    # force into the minimum absolute value residue class, so that -180 < angle <= 180  
                    if ang > math.pi:
                        ang -= 2*math.pi

                    if abs(ang) > 1e-6 and abs(ang) < math.pi*2:
                        if bone.axis and (rot_axis - bone.axis).length_squared() > 0.5:
                            # Clamp the rotation value:
                            ang = max( -bone.max_ang, min( -bone.min_ang, ang ) )
                        else:
                            # Clamp the rotation value:
                            ang = max( bone.min_ang, min( bone.max_ang, ang ) )
                            #ang = -ang
                            #rot_axis = -rot_axis

                    #annealing = (j+1)/len(self.bones)
                    #print("annealing", annealing)
                    #q = q_old + (q_new-q_old)*annealing

                    q_new.set_from_axis_angle_rad( ang, rot_axis )

                    bone_node.set_quat( q_new )


    def set_target( self, node ):
        self.target = node

    def debug_display( self, line_length=0.2, x_ray=True, draw_constraints=True ):

        self.remove_debug_display()

        self.debug_display_enabled = True

        axes_geom = create_axes( line_length )

        for i in range(len(self.bones)):
            bone = self.bones[i]

            # Attach a new node to the ik_node. All debug info will be attached to this node.
            # This is only for cleaner removal of the node later on - by remuving the debug node,
            # all debug info will be cleared. Otherwise this node has no significance and we could
            # just as well attach everything to the ik_node itself
            bone.debug_node = bone.control_node.attach_new_node("Debug_display")
            bone.debug_node.set_light_off(1)

            # Draw axes at my location and rotation (i.e. after applying my transform to my parent):
            axes = bone.debug_node.attach_new_node( axes_geom )
            #point = bone.ik_node.attach_new_node( create_point( col=bone.col ) )

            # Retrieve parent space:
            if bone.parent:
                # If we have a parent, then this parent is a bone.
                parent_node = bone.parent.control_node
            else:
                # Otherwise the parent is the actor itself, i.e. the root of the skeleton
                parent_node = self.actor

            # Again, use the parent's debug node rather than attaching stuff to the ik_node directly,
            # so we can remove the debug info easily later on by removing the debug node.
            parent_debug_node = parent_node.find("Debug_display")
            if not parent_debug_node:
                parent_debug_node = parent_node.attach_new_node("Debug_display")
                parent_debug_node.set_light_off(1)

            # Draw my offset in parent space
            lines = LineSegs()
            lines.set_thickness( 3 )
            lines.set_color( bone.col[0], bone.col[1], bone.col[2], 1 )
            lines.move_to( 0, 0, 0 )
            my_pos = bone.control_node.get_pos( parent_node )
            lines.draw_to( my_pos )
            geom = lines.create()
            parent_debug_node.attach_new_node( geom )

            # Draw my constraints:
            # These need to be drawn in parent space (since my rotation is done in parent space)
            if draw_constraints:
                if bone.axis:
                    l = get_perpendicular_vec( bone.axis )*line_length

                    lines = LineSegs()
                    lines.set_color( 0.6, 0.3, 0.3 )
                    lines.set_thickness( 5 )
                    lines.move_to( 0,0,0 )
                    lines.draw_to( l )
                    bone.debug_node.attach_new_node(lines.create())
        
                    lines = LineSegs()
                    lines.set_color( 0.8, 0.1, 0.2 )
                    lines.set_thickness( 3 )
                    q_min = Quat()
                    q_min.set_from_axis_angle_rad( bone.min_ang, bone.axis )
                    q_max = Quat()
                    q_max.set_from_axis_angle_rad( bone.max_ang, bone.axis )
                    lines.move_to( my_pos )
                    lines.draw_to( my_pos + q_min.xform( l ) )
                    lines.move_to( my_pos )
                    lines.draw_to( my_pos + q_max.xform( l ) )
                    parent_debug_node.attach_new_node(lines.create())

                    # Draw arc:
                    lines = LineSegs()
                    lines.set_color( 0.6, 0.3, 0.3 )
                    lines.set_thickness( 2 )
                    lines.move_to( my_pos + q_min.xform( l*0.9 ) )
                    ang = bone.min_ang
                    while ang < bone.max_ang:
                        ang += math.pi*0.1
                        if ang > bone.max_ang:
                            ang = bone.max_ang
                        q = Quat()
                        q.set_from_axis_angle_rad( ang, bone.axis )
                        lines.draw_to( my_pos + q.xform( l*0.9 ) )
                    parent_debug_node.attach_new_node(lines.create())


                if bone.axis:
                    lines = LineSegs()
                    lines.set_color( 0.8, 0.8, 0.8 )
                    lines.set_thickness( 4 )
                    my_pos = bone.control_node.get_pos( parent_node )
                    lines.move_to( my_pos )
                    lines.draw_to( my_pos + bone.axis*0.1 )
                    geom = lines.create()
                    constraints_axis = parent_node.attach_new_node( geom )
                    print("drawing axis", parent_node)

            if x_ray:
                bone.debug_node.set_bin("fixed", 0)
                bone.debug_node.set_depth_test(False)
                bone.debug_node.set_depth_write(False)
                parent_debug_node.set_bin("fixed", 0)
                parent_debug_node.set_depth_test(False)
                parent_debug_node.set_depth_write(False)

    def remove_debug_display( self ):
        for i in range(len(self.bones)):
            bone = self.bones[i]
            if hasattr( bone, "debug_node" ):
                bone.debug_node.remove_node()
                bone.debug_node = None

        root_debug_node = self.actor.find("Debug_display")
        if root_debug_node:
            root_debug_node.remove_node()

    def calc_length( self ):
        length = 0
        for i in range(1,len(self.bones)):
            b1 = self.bones[i]
            b0 = self.bones[i-1]
            diff = b1.control_node.get_pos( render ) - b0.control_node.get_pos( render )
            length += diff.length()
        return length


