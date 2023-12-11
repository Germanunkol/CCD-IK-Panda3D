from panda3d.core import *
from direct.actor.Actor import Actor
import random, math
from .ik_joint import IKJoint
from .utils import *
from .vec_utils import *

class IKChain():

    def __init__( self ):

        # We need an actor to be able to control (and expose) joints. If it already exists,
        # likely because the model was loaded from a file - great then just use that.
        # However, if it doesn't exist, this means we first need to set up all the joints before
        # we create the actor, so keep it empty for now!
        #self.actor = actor
        self.root = None

        self.ik_joints = []
        self.target = None
        self.target_reached = False

        self.debug_display_enabled = False

        self.end_effector = None

        # Default: exponent is zero (no annealing)
        self.annealing_exponent = 0


    def add_joint( self, joint, control_node, parent_ik_joint=None, static=False ):

        name = joint.get_name()

        ik_joint = IKJoint( joint, parent=parent_ik_joint, static=static )
        ik_joint.control_node = control_node

        if parent_ik_joint:
            control_node.reparent_to( parent_ik_joint.control_node )

        self.ik_joints.append(ik_joint)

        if self.root == None:
            self.root = control_node.get_parent()

        return ik_joint

    def get_ik_joint( self, joint_id ):
        if type(joint_id) == int:
            return self.ik_joints[joint_id]
        for b in self.ik_joints:
            if b.joint.get_name() == joint_id:
                return b
        raise Exception(f"Cannot find joint {joint_id}!")

    def get_num_ik_joints( self ):
        return len(self.ik_joints)

    def update_ik( self, threshold = 1e-2, min_iterations=1, max_iterations=10 ):

        # Solve the IK chain for the IK nodes:
        self.inverse_kinematics_cCD( threshold, min_iterations, max_iterations )

        # Copy the data from the IK chain to the actual bones.
        # This will end up affecting the actual mesh.
        for ik_joint in self.ik_joints:
            ik_joint.control_node.set_quat( ik_joint.control_node.get_quat() )

    def get_annealing_exponent( self ):
        return self.annealing_exponent

    def set_annealing_exponent( self, exponent ):
        """ Set the annealing strength

        Annealing is the process of reducing rotation of joints further away from the root.
        This attempts to counter an effect inherent to CCD-IK which lets joints further
        down the chain rotate much more than those closer to the root.

        Valid values are all positive numbers >= 0. (Usually use number between 1 and 4).
        Note: This is not tested well and has less of an effect than I had hoped for.

        Passing 0 disables annealing.
        """
        self.annealing_exponent = max( exponent, 0 )

    def inverse_kinematics_cCD( self, threshold = 1e-2, min_iterations=0, max_iterations=10 ):

        assert self.root is not None and len(self.ik_joints) > 0, \
                "IK only works when chain has at least one joint!"

        assert self.target is not None, "IK target must be set!"

        if not self.end_effector:
            self.end_effector = self.ik_joints[-1].control_node.attach_new_node( "End_effector" )

        self.target_reached = False
        for i in range(max_iterations):

            if i > min_iterations:
                err = (self.target.get_pos(self.root)-self.end_effector.get_pos(self.root)).length()
                if err < threshold:
                    self.target_reached = True
                    break

            for j in range(len(self.ik_joints)-1):
                ik_joint = self.ik_joints[-j-2]

                if ik_joint.static:
                    continue

                ik_joint_node = ik_joint.control_node
                if ik_joint.parent:
                    parent_node = ik_joint.parent.control_node
                else:
                    #parent_node = self.actor
                    #parent_node = ik_joint.control_node.get_parent()
                    parent_node = self.root

                target = self.target.get_pos( ik_joint_node )

                pos = LPoint3.zero()
                ee = self.end_effector.get_pos( ik_joint_node )

                d1 = target-pos
                d2 = ee-pos

                cross = d1.cross(d2).normalized()
                if cross.length_squared() < 1e-9:
                    continue

                ang = d2.normalized().signed_angle_rad( d1.normalized(), cross )
                q = Quat()
                q.set_from_axis_angle_rad( ang, cross )
                # Add this rotation to the current rotation:
                q_old = ik_joint_node.get_quat()
                q_new = q*q_old
                q_new.normalize()
                #ik_joint_node.set_quat( q_new )

                # Correct rotation for hinge:
                if ik_joint.axis:
                    #q_inv = ik_joint_node.get_quat()
                    #q_inv.invert_in_place()
                    #my_axis_in_parent_space = q_inv.xform( ik_joint.axis )
                    my_axis_in_parent_space = ik_joint.axis
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
                        if ik_joint.axis and (rot_axis - ik_joint.axis).length_squared() > 0.5:
                            # Clamp the rotation value:
                            ang = max( -ik_joint.max_ang, min( -ik_joint.min_ang, ang ) )
                        else:
                            # Clamp the rotation value:
                            ang = max( ik_joint.min_ang, min( ik_joint.max_ang, ang ) )
                            #ang = -ang
                            #rot_axis = -rot_axis


                    q_new.set_from_axis_angle_rad( ang, rot_axis )

                    ik_joint_factor = (j+1)/(len(self.ik_joints)-1)
                    annealing = ik_joint_factor**self.annealing_exponent
                    q_new = q_old + (q_new-q_old)*annealing

                    ik_joint_node.set_quat( q_new )


    def set_target( self, node ):
        self.target = node

    def debug_display( self, line_length=0.2, thickness=2, x_ray=True, draw_constraints=True ):

        self.remove_debug_display()

        self.debug_display_enabled = True

        axes_geom = create_axes( line_length )

        for i in range(len(self.ik_joints)):
            ik_joint = self.ik_joints[i]

            # Attach a new node to the ik_node. All debug info will be attached to this node.
            # This is only for cleaner removal of the node later on - by remuving the debug node,
            # all debug info will be cleared. Otherwise this node has no significance and we could
            # just as well attach everything to the ik_node itself
            ik_joint.debug_node = ik_joint.control_node.attach_new_node("Debug_display")
            ik_joint.debug_node.set_light_off(1)

            # Draw axes at my location and rotation (i.e. after applying my transform to my parent):
            axes = ik_joint.debug_node.attach_new_node( axes_geom )
            #point = ik_joint.ik_node.attach_new_node( create_point( col=ik_joint.col ) )

            # Retrieve parent space:
            if ik_joint.parent:
                # If we have a parent, then this parent is a ik_joint.
                parent_node = ik_joint.parent.control_node
            else:
                # Otherwise the parent is the actor itself, i.e. the root of the skeleton
                #parent_node = self.actor
                # Use control node parent
                #parent_node = ik_joint.control_node.get_parent()
                parent_node = self.root

            # Again, use the parent's debug node rather than attaching stuff to the parent directly,
            # so we can remove the debug info easily later on by removing the debug node.
            parent_debug_node = parent_node.find("Debug_display")
            if not parent_debug_node:
                parent_debug_node = parent_node.attach_new_node("Debug_display")
                parent_debug_node.set_light_off(1)

            # Draw my offset in parent space
            lines = LineSegs()
            lines.set_thickness( thickness )
            lines.set_color( ik_joint.col[0], ik_joint.col[1], ik_joint.col[2], 1 )
            lines.move_to( 0, 0, 0 )
            my_pos = ik_joint.control_node.get_pos( parent_node )
            lines.draw_to( my_pos )
            geom = lines.create()
            parent_debug_node.attach_new_node( geom )

            # Draw my constraints:
            # These need to be drawn in parent space (since my rotation is done in parent space)
            if draw_constraints:
                if ik_joint.axis:
                    l = -get_perpendicular_vec( ik_joint.axis )*line_length

                    lines = LineSegs()
                    lines.set_color( 0.6, 0.3, 0.3 )
                    lines.set_thickness( thickness )
                    lines.move_to( 0,0,0 )
                    lines.draw_to( l )
                    ik_joint.debug_node.attach_new_node(lines.create())
        
                    lines = LineSegs()
                    lines.set_color( 0.8, 0.1, 0.2 )
                    lines.set_thickness( thickness )
                    q_min = Quat()
                    q_min.set_from_axis_angle_rad( ik_joint.min_ang, ik_joint.axis )
                    q_max = Quat()
                    q_max.set_from_axis_angle_rad( ik_joint.max_ang, ik_joint.axis )
                    lines.move_to( my_pos )
                    lines.draw_to( my_pos + q_min.xform( l ) )
                    lines.move_to( my_pos )
                    lines.draw_to( my_pos + q_max.xform( l ) )
                    parent_debug_node.attach_new_node(lines.create())

                    # Draw arc:
                    lines = LineSegs()
                    lines.set_color( 0.6, 0.3, 0.3 )
                    lines.set_thickness( thickness )
                    lines.move_to( my_pos + q_min.xform( l*0.9 ) )
                    ang = ik_joint.min_ang
                    while ang < ik_joint.max_ang:
                        ang += math.pi*0.1
                        if ang > ik_joint.max_ang:
                            ang = ik_joint.max_ang
                        q = Quat()
                        q.set_from_axis_angle_rad( ang, ik_joint.axis )
                        lines.draw_to( my_pos + q.xform( l*0.9 ) )
                    parent_debug_node.attach_new_node(lines.create())


                if ik_joint.axis:
                    lines = LineSegs()
                    lines.set_color( 0.8, 0.8, 0.8 )
                    lines.set_thickness( thickness )
                    my_pos = ik_joint.control_node.get_pos( parent_node )
                    lines.move_to( my_pos )
                    lines.draw_to( my_pos + ik_joint.axis*0.1 )
                    geom = lines.create()
                    constraints_axis = parent_node.attach_new_node( geom )

            if x_ray:
                ik_joint.debug_node.set_bin("fixed", 0)
                ik_joint.debug_node.set_depth_test(False)
                ik_joint.debug_node.set_depth_write(False)
                parent_debug_node.set_bin("fixed", 0)
                parent_debug_node.set_depth_test(False)
                parent_debug_node.set_depth_write(False)

    def remove_debug_display( self ):
        for i in range(len(self.ik_joints)):
            ik_joint = self.ik_joints[i]
            if hasattr( ik_joint, "debug_node" ):
                if ik_joint.debug_node is not None:
                    ik_joint.debug_node.remove_node()
                    ik_joint.debug_node = None

        root_debug_node = self.root.find("Debug_display")
        if root_debug_node:
            root_debug_node.remove_node()

    def calc_length( self ):
        length = 0
        for i in range(1,len(self.ik_joints)):
            b1 = self.ik_joints[i]
            b0 = self.ik_joints[i-1]
            diff = b1.control_node.get_pos( self.root ) - b0.control_node.get_pos( self.root )
            length += diff.length()
        return length

    def get_end_effector( self ):
        assert len(self.ik_joints) > 0
        return self.ik_joints[-1].get_control_node()


