from panda3d.core import *
from CCDIK.ik_chain import IKChain
from CCDIK.utils import *
from walk_cycle import WalkCycle
from CCDIK.armature_utils import ArmatureUtils

class Biped():

    def __init__( self ):

        ##################################
        # Set up main body:

        self.torso_height = 1.6
        self.torso_node = render.attach_new_node("Torso")
        self.torso_node.set_pos( 0, 0, self.torso_height )
        geom = create_axes( 0.3 )
        self.torso_node.attach_new_node( geom )
        
        hip_node = self.torso_node.attach_new_node( "Hip" )
        hip_node.set_pos( 0, 0, -0.6 )

        ##################################
        # Set up body movement:

        self.target_node = render.attach_new_node( "WalkTarget" )
        #geom = createAxes( 0.2 )
        #self.targetNode.attachNewNode( geom )
        self.walk_speed = 1  # m/s
        self.turn_speed = 2
        self.new_random_target()

        ##################################
        # Set up Armature and Joints:
        au = ArmatureUtils()
            
        joint = None
        offset_length = 0.5
        root_joint = au.create_joint( "root" )

        ###############
        # Left leg:
        hip_l = au.create_joint( "hip_l", parent_joint=root_joint )
        
        # First, rotate 90 degrees outwards:
        upper_leg_l = au.create_joint( "upper_leg_l", parent_joint=hip_l,
                translate=-LVector3f.unit_x()*0.13 )

        lower_leg_l = au.create_joint( "lower_leg_l", parent_joint=upper_leg_l,
                translate=LVector3f.unit_z()*0.45 )

        foot_l = au.create_joint( "foot_l", parent_joint=lower_leg_l,
                translate=LVector3f.unit_z()*0.6 )

        ###############
        # Right leg:
        hip_r = au.create_joint( "hip_r", parent_joint=root_joint )
        # First, rotate 90 degrees outwards:
        upper_leg_r = au.create_joint( "upper_leg_r", parent_joint=hip_r,
                translate=LVector3f.unit_x()*0.13 )

        lower_leg_r = au.create_joint( "lower_leg_r", parent_joint=upper_leg_r,
                translate=LVector3f.unit_z()*0.45 )

        foot_r = au.create_joint( "foot_r", parent_joint=lower_leg_r,
                translate=LVector3f.unit_z()*0.6 )

        ## IMPORTANT! Let the Armature_utils create the actor and set up control nodes:
        au.finalize()

        ## IMPORTANT! Attach the created actor to the scene, otherwise you won't see anything!
        au.get_actor().reparent_to( hip_node )

        ##################################
        # Set up left IK Chain:

        self.ik_chain_leg_left = IKChain( au.get_actor() )

        bone = self.ik_chain_leg_left.add_joint( hip_l, au.get_control_node( hip_l.get_name() ) )
        bone = self.ik_chain_leg_left.add_joint( upper_leg_l, au.get_control_node( upper_leg_l.get_name() ),
                parent_bone=bone )
        bone = self.ik_chain_leg_left.add_joint( lower_leg_l, au.get_control_node( lower_leg_l.get_name() ),
                parent_bone=bone )
        bone = self.ik_chain_leg_left.add_joint( foot_l, au.get_control_node( foot_l.get_name() ),
                parent_bone=bone )

        self.ik_chain_leg_left.set_static( hip_l.get_name() )
        self.ik_chain_leg_left.set_hinge_constraint( lower_leg_l.get_name(),
                LVector3f.unit_x(), min_ang=0, max_ang=math.pi*0.5 )

        self.ik_chain_leg_left.debug_display()

        ##################################
        # Set up right IK Chain:

        self.ik_chain_leg_right = IKChain( au.get_actor() )

        bone = self.ik_chain_leg_right.add_joint( hip_r, au.get_control_node( hip_r.get_name() ) )
        bone = self.ik_chain_leg_right.add_joint( upper_leg_r, au.get_control_node( upper_leg_r.get_name() ),
                parent_bone=bone )
        bone = self.ik_chain_leg_right.add_joint( lower_leg_r, au.get_control_node( lower_leg_r.get_name() ),
                parent_bone=bone )
        bone = self.ik_chain_leg_right.add_joint( foot_r, au.get_control_node( foot_r.get_name() ),
                parent_bone=bone )

        self.ik_chain_leg_right.set_static( hip_r.get_name() )
        self.ik_chain_leg_right.set_hinge_constraint( lower_leg_r.get_name(),
                LVector3f.unit_x(), min_ang=0, max_ang=math.pi*0.5 )

        self.ik_chain_leg_right.debug_display()

        #self.ik_chain_leg_left.update_ik()
        #self.ik_chain_leg_right.update_ik()

        #################################################
        # Foot targets:

        # Set up two targets that the foot should reach:
        self.foot_target_left = render.attach_new_node("FootTargetLeft")
        self.foot_target_right = render.attach_new_node("FootTargetRight")
        geom = create_axes( 0.1 )
        self.foot_target_left.attach_new_node( geom )
        self.foot_target_right.attach_new_node( geom )
        self.ik_chain_leg_left.set_target( self.foot_target_left )
        self.ik_chain_leg_right.set_target( self.foot_target_right )

        # Set up two nodes which stay (rigidly) infront of the body, on the floor.
        # Whenever a leg needs to take a step, the target will be placed on this position:
        self.planned_foot_target_left = self.torso_node.attach_new_node( "PlannedFootTargetLeft" )
        self.planned_foot_target_right = self.torso_node.attach_new_node( "PlannedFootTargetRight" )

        step_dist = 0.35
        self.planned_foot_target_left.set_pos( -0.15, step_dist, -self.torso_height )
        self.planned_foot_target_right.set_pos( 0.15, step_dist, -self.torso_height )
        self.planned_foot_target_left.attach_new_node( geom )
        self.planned_foot_target_right.attach_new_node( geom )

        self.leg_movement_speed = self.walk_speed*3

        self.step_left = False
        self.step_right = False
        
        self.walk_cycle = WalkCycle( 2, 0.75 )

        #################################################
        ## Set up controls and labels:

        base.taskMgr.add( self.walk, "BipedWalk")
        base.accept( "+", self.speed_up )
        base.accept( "-", self.slow_down )

        label("[WASD]: Move Camera", 1)
        label("[Mouse Wheel]: Zoom Camera", 2)
        label("[Middle Mouse]: Rotate Camera", 3)
        label("[+]: Speed up", 5)
        label("[-]: Slow down", 6)

    def speed_up( self ):
        self.walk_speed += 0.5
        self.walk_speed = min(self.walk_speed, 3)
        self.turn_speed = self.walk_speed*2
        self.leg_movement_speed = self.walk_speed*3

    def slow_down( self ):
        self.walk_speed -= 0.5
        self.walk_speed = max(self.walk_speed, 0)
        self.turn_speed = self.walk_speed*2
        self.leg_movement_speed = self.walk_speed*3

    def walk( self, task ):

        #############################
        # Update body:

        prev_pos = self.torso_node.get_pos()

        diff = self.target_node.get_pos( self.torso_node )
        diff.z = 0
        diff_n = diff.normalized()
        ang = LVector3f.unit_y().angle_rad( diff_n )
        axis = LVector3f.unit_y().cross( diff_n )
        axis.normalize()
        max_rot = self.turn_speed*globalClock.get_dt()
        ang_clamped = 0
        if axis.length() > 0.999:
            # Limit angle:
            ang_clamped = max( -max_rot, min( max_rot, ang ) )
            q = Quat()
            q.set_from_axis_angle_rad( ang_clamped, axis )

            q_old = self.torso_node.get_quat()
            q_new = q*q_old
            self.torso_node.set_quat( q_new  )
        if abs( ang ) < max_rot:
            step = diff_n*self.walk_speed*globalClock.get_dt()
            if step.length_squared() > diff.length_squared():
                self.new_random_target()
                step = diff
            step = self.torso_node.get_quat().xform( step )
            self.torso_node.set_pos( self.torso_node.get_pos() + step )

        # Calculate how far we've walked this frame:
        cur_walk_dist = (prev_pos - self.torso_node.get_pos()).length()

        #############################
        # Update legs:

        # Move planned foot target further forward (longer steps) when character is
        # walking faster:
        step_dist = cur_walk_dist*0.1/globalClock.dt
        self.planned_foot_target_left.set_pos( -0.15, step_dist, -self.torso_height )
        self.planned_foot_target_right.set_pos( 0.15, step_dist, -self.torso_height )

        # Update the walkcycle to determine if a step needs to be taken:
        #update = cur_walk_dist*0.1/globalClock.dt
        update = cur_walk_dist
        update += ang_clamped*0.5
        self.walk_cycle.update_time( update )

        if self.walk_cycle.step_required[0]:
            #self.foot_target_left.set_pos( self.planned_foot_target_left.get_pos( render ) )
            self.walk_cycle.step( 0 )
            self.step_left = True
        if self.walk_cycle.step_required[1]:
            #self.foot_target_right.set_pos( self.planned_foot_target_right.get_pos( render ) )
            self.walk_cycle.step( 1 )
            self.step_right = True

        if self.step_left:
            diff = self.planned_foot_target_left.get_pos(render) - self.foot_target_left.get_pos()
            leg_move_dist = self.leg_movement_speed*globalClock.dt
            if diff.length() < leg_move_dist:
                self.foot_target_left.set_pos( self.planned_foot_target_left.get_pos( render ) )
                self.step_left = False
            else:
                moved = self.foot_target_left.get_pos() + diff.normalized()*leg_move_dist
                self.foot_target_left.set_pos( moved )

        if self.step_right:
            diff = self.planned_foot_target_right.get_pos(render) - self.foot_target_right.get_pos()
            leg_move_dist = self.leg_movement_speed*globalClock.dt
            if diff.length() < leg_move_dist:
                self.foot_target_right.set_pos( self.planned_foot_target_right.get_pos( render ) )
                self.step_right = False
            else:
                moved = self.foot_target_right.get_pos() + diff.normalized()*leg_move_dist
                self.foot_target_right.set_pos( moved )

        self.ik_chain_leg_left.update_ik()
        self.ik_chain_leg_right.update_ik()

        return task.cont

    
    def new_random_target( self ):

        self.target_node.set_pos(
                LVector3f( random.random()*10-5,
                    random.random()*10-5,
                    0 ) )




if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CCDIK.camera_control import CameraControl

    class MyApp(ShowBase):

        def __init__(self):

            #####################################
            ## Set up scene

            ShowBase.__init__(self)
            base.disableMouse()
            base.set_frame_rate_meter(True)

            wp = WindowProperties()
            wp.set_size(1800, 960)
            self.win.request_properties(wp)

            base.set_background_color(0,0,0)

            grid = create_grid( 20, 1 )
            render.attach_new_node( grid )
            axes = create_axes( 1000, bothways=True, thickness=3 )
            render.attach_new_node( axes )

            #####################################
            # Set up Biped
            self.biped = Biped()

            #####################################
            # Set up Camera and input:

            self.cam_control = CameraControl( camera, self.mouseWatcherNode, speed=0.02 )
            #self.cam_control.attach_to( self.biped.torso_node )
            
            self.taskMgr.add( self.cam_control.move_camera, "MoveCameraTask")

            self.accept( "wheel_down", self.cam_control.wheel_down )
            self.accept( "wheel_up", self.cam_control.wheel_up )


    app = MyApp()
    app.run()
