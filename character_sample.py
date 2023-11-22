from panda3d.core import *
from CCDIK.ik_chain import IKChain
from CCDIK.ik_actor import IKActor
from CCDIK.utils import *
from walk_cycle import WalkCycle
from foot_arc import FootArc
from collision_terrain import CollisionTerrain
from direct.actor.Actor import Actor

class RiggedChar():

    def __init__( self, terrain ):

        annealing_exponent = 6

        ##################################
        # Set up main body:

        self.root_node = render.attach_new_node("Torso")
        geom = create_axes( 0.3 )
        self.root_node.attach_new_node( geom )

        # How high the root node should currently be (Determined by the average position of all
        # grounded feet):
        self.root_height = 0.956756    # Distance between root node and the ground
        #self.root_height = 0
        self.root_node.set_pos( 0, 0, self.root_height )
        self.cur_target_height = self.root_height


        ##################################
        # Set up body movement:

        self.target_node = render.attach_new_node( "Walk_target" )
        #geom = create_axes( 0.2 )
        #self.target_node.attach_new_node( geom )
        self.walk_speed = 0.5  # m/s
        self.turn_speed = 1
        self.height_adjustment_speed = self.walk_speed
        self.new_random_target()

        ##################################
        # Set up legs:

        self.model = loader.load_model( "Meshes/person.bam" )

        # Standard material:
        m = Material()
        m.set_base_color((0.1, 0.5, 0.1, 1))
        m.set_ambient((0.1,0.1,0.1,1))
        m.set_specular((0.1,0.7,0.1,1))

        self.ik_actor = IKActor( self.model )
        self.ik_actor.reparent_to( self.root_node )
        self.ik_actor.actor.set_material(m)

        #render.find("**/Body").set_material(m)
        #render.find("**/Body").set_shader_auto()

#        root_bone = actor.expose_joint(None, "model_root", "Bone")
#        root_bone_control = actor.control_joint(None, "model_root", "Bone")
#        root_bone_control.set_hpr( 45, 45, 45 )

        self.ik_chain_leg_left = self.ik_actor.create_ik_chain( ["Hip.L", "UpperLeg.L", "LowerLeg.L", "Foot.L"] )
        self.ik_chain_leg_left.set_annealing_exponent( annealing_exponent )
        self.ik_chain_leg_left.get_ik_joint( "Hip.L" ).set_hinge_constraint( axis=LVector3f.unit_z(),
                min_ang=-math.pi*0.05, max_ang=math.pi*0.05 )
        self.ik_chain_leg_left.get_ik_joint( "UpperLeg.L" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.2, max_ang=math.pi*0.2 )
        self.ik_chain_leg_left.get_ik_joint( "LowerLeg.L" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.5, max_ang=-math.pi*0.05 )
        self.ik_chain_leg_left.get_ik_joint( "Foot.L" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.5, max_ang=math.pi*0.5 )


        self.ik_chain_leg_right = self.ik_actor.create_ik_chain( ["Hip.R", "UpperLeg.R", "LowerLeg.R", "Foot.R"] )
        self.ik_chain_leg_right.set_annealing_exponent( annealing_exponent )
        self.ik_chain_leg_right.get_ik_joint( "Hip.R" ).set_hinge_constraint( axis=LVector3f.unit_z(),
                min_ang=-math.pi*0.05, max_ang=math.pi*0.05 )
        self.ik_chain_leg_right.get_ik_joint( "UpperLeg.R" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.2, max_ang=math.pi*0.2 )
        self.ik_chain_leg_right.get_ik_joint( "LowerLeg.R" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.5, max_ang=-math.pi*0.05 )
        self.ik_chain_leg_right.get_ik_joint( "Foot.R" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.5, max_ang=math.pi*0.5 )


        #self.ik_chain_leg_left.update_ik()
        #self.ik_chain_leg_right.update_ik()

        #################################################
        # Set up arm chains:

        self.ik_chain_arm_left = self.ik_actor.create_ik_chain( ["Shoulder.L", "UpperArm.L", "LowerArm.L", "Hand.L"] )
        self.ik_chain_arm_left.set_annealing_exponent( annealing_exponent )
        self.ik_chain_arm_left.get_ik_joint( "Shoulder.L" ).set_hinge_constraint( axis=LVector3f.unit_z(),
                min_ang=math.pi*0.05, max_ang=math.pi*0.05 )
        self.ik_chain_arm_left.get_ik_joint( "UpperArm.L" ).set_hinge_constraint( axis=LVector3f.unit_y(),
                min_ang=-math.pi*0.5, max_ang=math.pi*0.5 )
        self.ik_chain_arm_left.get_ik_joint( "LowerArm.L" ).set_hinge_constraint( axis=LVector3f.unit_z(),
                min_ang=-math.pi*0.5, max_ang=0 )
        self.ik_chain_arm_left.get_ik_joint( "Hand.L" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.3, max_ang=math.pi*0.3 )

        self.ik_chain_arm_right = self.ik_actor.create_ik_chain( ["Shoulder.R", "UpperArm.R", "LowerArm.R", "Hand.R"] )
        self.ik_chain_arm_right.set_annealing_exponent( annealing_exponent )
        self.ik_chain_arm_right.get_ik_joint( "Shoulder.R" ).set_hinge_constraint( axis=LVector3f.unit_z(),
                min_ang=math.pi*0.05, max_ang=math.pi*0.05 )
        self.ik_chain_arm_right.get_ik_joint( "UpperArm.R" ).set_hinge_constraint( axis=LVector3f.unit_y(),
                min_ang=-math.pi*0.5, max_ang=math.pi*0.5 )
        self.ik_chain_arm_right.get_ik_joint( "LowerArm.R" ).set_hinge_constraint( axis=LVector3f.unit_z(),
                min_ang=0, max_ang=math.pi*0.5 )
        self.ik_chain_arm_right.get_ik_joint( "Hand.R" ).set_hinge_constraint( axis=LVector3f.unit_x(),
                min_ang=-math.pi*0.3, max_ang=math.pi*0.3 )



        ############################

        self.ik_chain_leg_left.debug_display( line_length=0.1 )
        self.ik_chain_leg_right.debug_display( line_length=0.1 )
        self.ik_chain_arm_left.debug_display( line_length=0.1 )
        self.ik_chain_arm_right.debug_display( line_length=0.1 )

        #################################################
        # Foot targets:

        # Set up two targets that the foot should reach:
        self.foot_target_left = render.attach_new_node("Foot_target_left")
        self.foot_target_right = render.attach_new_node("Foot_target_right")
        geom = create_axes( 0.15 )
        self.foot_target_left.attach_new_node( geom )
        self.foot_target_right.attach_new_node( geom )
        self.ik_chain_leg_left.set_target( self.foot_target_left )
        self.ik_chain_leg_right.set_target( self.foot_target_right )
    
        # x_ray:
        self.foot_target_left.set_bin("fixed", 0)
        self.foot_target_left.set_depth_test(False)
        self.foot_target_left.set_depth_write(False)
        self.foot_target_right.set_bin("fixed", 0)
        self.foot_target_right.set_depth_test(False)
        self.foot_target_right.set_depth_write(False)
 

        # Set up two nodes which stay (rigidly) infront of the body, on the floor.
        # Whenever a leg needs to take a step, the target will be placed on this position:
        self.planned_rotation = self.root_node.attach_new_node( "Planned_rotation_node" )
        self.planned_foot_target_left = self.planned_rotation.attach_new_node( "Planned_foot_target_left" )
        self.planned_foot_target_right = self.planned_rotation.attach_new_node( "Planned_foot_target_right" )
        # Get distance from root bone to foot bone. This is the length of the leg, i.e. it tells
        # us how far the planned foot position should be away from the root:
        foot_node = self.ik_actor.get_control_node( "Foot.L" )
        foot_pos = foot_node.get_pos( self.root_node )
        foot_height = self.root_height + foot_pos.get_z()
        self.foot_outwards = abs(foot_pos.get_x())
        self.foot_height_offset = LVector3f(0,0,foot_height)
        print("Foot height:", foot_height, self.root_height, foot_node, foot_node.get_pos( self.root_node ))

        self.step_dist = 0.35     # Length of a step
        self.planned_foot_target_left.set_pos( -self.foot_outwards, self.step_dist, 0 )
        self.planned_foot_target_right.set_pos( self.foot_outwards, self.step_dist, 0 )
        self.planned_foot_target_left.attach_new_node( geom )
        self.planned_foot_target_right.attach_new_node( geom )

        self.leg_movement_speed = self.walk_speed*2

        self.step_arc_left = None
        self.step_arc_right = None
        
        self.walk_cycle = WalkCycle( 2, 0.5 )

        #self.noise = Perlin_noise2()
        #self.noise.set_scale( 0.1, 0.1 )

        self.hand_base_pos_left = self.root_node.attach_new_node("Hand_target_left")
        self.hand_base_pos_left.set_pos( -0.3, 0, -0.3 )
        self.hand_target_left = self.hand_base_pos_left.attach_new_node("Hand_target_left")

        self.hand_base_pos_right = self.root_node.attach_new_node("Hand_target_right")
        self.hand_base_pos_right.set_pos( 0.3, 0, -0.3 )
        self.hand_target_right = self.hand_base_pos_right.attach_new_node("Hand_target_right")

        self.ik_chain_arm_left.set_target( self.hand_target_left )
        self.ik_chain_arm_right.set_target( self.hand_target_right )

        self.hand_target_left.attach_new_node( geom )
        self.hand_target_right.attach_new_node( geom )
        

        ###########################################
        ## Set up lights:

        light = PointLight("PointLight")
        light.set_color_temperature( 9000 )
        #light.attenuation = (1, 0.5, 0.5)
        light.attenuation = (0.75, 0, 0.05)
        light_node = render.attach_new_node( light )
        light_node.set_pos( 0, 0, 3 )
        render.set_light( light_node ) 
        #light.set_shadow_caster(True, 1024, 1024, -2000 ) # low sort value to render early!

        alight = AmbientLight('alight')
        alight.set_color((0.2, 0.3, 0.2, 1))
        alnp = render.attach_new_node(alight)
        render.set_light(alnp)

        #################################################


        base.taskMgr.add( self.walk, "Rigged_char_walk")
        base.accept( "+", self.speed_up )
        base.accept( "-", self.slow_down )

        ##################################
        # Set up collision:
        self.terrain = terrain
        
        self.collision_ray = CollisionRay()
        self.collision_ray.direction = -LVector3f.unit_z()    # Trace down
        cn = CollisionNode( "Root_ray_node" )
        cn.add_solid( self.collision_ray )
        self.ray_node = self.root_node.attach_new_node( cn )

        self.collision_traverser = CollisionTraverser()
        self.collision_queue = CollisionHandlerQueue()
        self.collision_traverser.add_collider( self.ray_node, self.collision_queue )
        #self.collision_traverser.traverse( self.cave.collision_floor_root )
        
        ##################################
        # Control upper body:
        self.torso_bone = self.ik_actor.get_control_node( "LowerSpine" )

    def speed_up( self ):
        self.walk_speed += 0.1
        self.walk_speed = min(self.walk_speed, 0.5)
        self.turn_speed = self.walk_speed*2
        self.height_adjustment_speed = self.walk_speed
        self.leg_movement_speed = 0.3 + self.walk_speed*1.2

    def slow_down( self ):
        self.walk_speed -= 0.1
        self.walk_speed = max(self.walk_speed, 0)
        self.turn_speed = self.walk_speed*2
        self.height_adjustment_speed = self.walk_speed
        self.leg_movement_speed = 0.3 + self.walk_speed*1.2

    def walk( self, task ):

        #############################
        # Update body:

        prev_pos = self.root_node.get_pos()

        diff = self.target_node.get_pos( self.root_node )
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

            q_old = self.root_node.get_quat()
            q_new = q*q_old
            self.root_node.set_quat( q_new  )
        if abs( ang ) < max_rot:
            step = diff_n*self.walk_speed*globalClock.get_dt()
            if step.length_squared() > diff.length_squared():
                self.new_random_target()
                step = diff
            step = self.root_node.get_quat().xform( step )
            self.root_node.set_pos( self.root_node.get_pos() + step )

        #############################
        # Calculate how far we've walked this frame:
        cur_walk_dist = (prev_pos - self.root_node.get_pos()).length()

        cur_walk_speed = cur_walk_dist/globalClock.get_dt()

        update = cur_walk_dist*0.75
        update += ang_clamped*0.5
        self.walk_cycle.update_time( update )

        #############################
        # Rotate torso:
        cycle = math.sin( self.walk_cycle.cycle_time/self.walk_cycle.cycle_duration*math.pi*2 )
        self.torso_bone.set_hpr( -4*cycle, -2, 0 )

        #############################
        # Move body up and down depending on foot placement:

        #if not self.step_arc_left and self.step_arc_right:
        #    self.cur_target_height = self.foot_target_left.get_pos().get_z()
        #elif not self.step_arc_right and self.step_arc_left:
        #    self.cur_target_height = self.foot_target_right.get_pos().get_z()
        #elif not self.step_arc_left and not self.step_arc_right:
        foot_pos_l = self.foot_target_left.get_pos()
        #if self.step_arc_left:
        #    foot_pos_l = self.step_arc_left.end_pos
        foot_pos_r = self.foot_target_right.get_pos()
        #if self.step_arc_right:
        #    foot_pos_r = self.step_arc_right.end_pos
        self.cur_target_height = 0.5*(foot_pos_l.get_z() +\
                foot_pos_r.get_z()) + self.root_height - self.foot_height_offset.get_z()

        cur_pos = self.root_node.get_pos()
        height_adjustment = self.cur_target_height - cur_pos.get_z()
        limit = self.height_adjustment_speed * globalClock.get_dt()
        height_adjustment = min( max( height_adjustment, -limit), limit )
        self.root_node.set_pos( cur_pos.get_x(), cur_pos.get_y(), cur_pos.get_z() + height_adjustment )


        #############################
        # Update arms:

        self.hand_target_left.set_pos( 0, -cycle*min(cur_walk_speed*0.16,0.3)+0.1, 0 )
        self.hand_target_right.set_pos( 0, cycle*min(cur_walk_speed*0.16,0.3)+0.1, 0 )

        self.ik_chain_arm_left.update_ik()
        self.ik_chain_arm_right.update_ik()

        #############################
        # Update legs:

        # TODO: Rotate planned_rotation
        #q = Quat()
        #q.set_from_axis_angle_rad( self.turn_speed, LVector3f.unit_z() )
        #self.planned_rotation.set_quat( q )

        # Move planned foot target further forward (longer steps) when character is
        # walking faster:
        cur_step_dist = 0.1
        if cur_walk_speed > 0:
            cur_step_dist = self.step_dist + self.walk_speed*0.2
        p = LVector3f( -self.foot_outwards, cur_step_dist, 0 )
        pw = render.get_relative_point( self.planned_rotation, p )
        #p.z = self.noise( pw.x, pw.y )*0.05 + 0.05
        self.planned_foot_target_left.set_pos( p )
        p = LVector3f( self.foot_outwards, cur_step_dist, 0 )
        pw = render.get_relative_point( self.planned_rotation, p )
        #p.z = self.noise( pw.x, pw.y )*0.05 + 0.05
        self.planned_foot_target_right.set_pos( p )

        # Update the walkcycle to determine if a step needs to be taken:
        #update = cur_walk_dist*0.1/globalClock.dt

        if self.walk_cycle.step_required[0]:
            #self.foot_target_left.set_pos( self.planned_foot_target_left.get_pos( render ) )
            self.walk_cycle.step( 0 )    # Tell walk cycle that step has been taken
            #h = min( cur_walk_speed*0.2, 0.3)
            h = 0.05
            target_pos = self.find_ground_pos( self.planned_foot_target_left.get_pos( self.root_node ) )
            self.step_arc_left = FootArc( self.foot_target_left.get_pos() - self.foot_height_offset,
                    render.get_relative_point( self.root_node, target_pos ), max_step_height=h )

        if self.walk_cycle.step_required[1]:
            #self.foot_target_right.set_pos( self.planned_foot_target_right.get_pos( render ) )
            self.walk_cycle.step( 1 )    # Tell walk cycle that step has been taken
            #h = min( cur_walk_speed*0.2, 0.3)
            h = 0.05
            target_pos = self.find_ground_pos( self.planned_foot_target_right.get_pos( self.root_node ) )
            self.step_arc_right = FootArc( self.foot_target_right.get_pos() - self.foot_height_offset,
                    render.get_relative_point( self.root_node, target_pos ), max_step_height=h )

        if self.step_arc_left:
            leg_move_dist = self.leg_movement_speed*globalClock.dt
            self.step_arc_left.update( leg_move_dist )
            self.foot_target_left.set_pos( self.step_arc_left.get_pos() + self.foot_height_offset )
            if self.step_arc_left.done():
                self.step_arc_left = None

        if self.step_arc_right:
            leg_move_dist = self.leg_movement_speed*globalClock.dt
            self.step_arc_right.update( leg_move_dist )
            self.foot_target_right.set_pos( self.step_arc_right.get_pos() + self.foot_height_offset )
            if self.step_arc_right.done():
                self.step_arc_right = None

        self.ik_chain_leg_left.update_ik()
        self.ik_chain_leg_right.update_ik()

        ##################################
        ## Let toes always face horizontally:
        ## Note: Not sure if this works correctly yet!

        toe_node = self.ik_actor.get_control_node( "Toes.L" )
        hpr = toe_node.get_hpr( self.root_node )
        toe_node.set_hpr( self.root_node, hpr.get_x(), 0, hpr.get_z() )

        toe_node = self.ik_actor.get_control_node( "Toes.R" )
        hpr = toe_node.get_hpr( self.root_node )
        toe_node.set_hpr( self.root_node, hpr.get_x(), 0, hpr.get_z() )

        return task.cont

    def find_ground_pos( self, input_pos ):
    
        self.ray_node.set_pos( input_pos + LVector3f.unit_z()*2 )
        self.collision_traverser.traverse( self.terrain.root )
        if len( self.collision_queue.get_entries() ) > 0:
            self.collision_queue.sort_entries()
            ground_pos = self.collision_queue.get_entry(0).get_surface_point( self.root_node )
            return ground_pos
        else:
            return input_pos
    
    def new_random_target( self ):

        self.target_node.set_pos(
                LVector3f( random.random()*9-4.5,
                    random.random()*9-4.5,
                    0 ) )


if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CCDIK.camera_control import CameraControl

    class MyApp(ShowBase):

        def __init__(self):

            #####################################
            ## Set up scene

            ShowBase.__init__(self)
            base.disable_mouse()
            base.set_frame_rate_meter(True)

            wp = WindowProperties()
            wp.set_size(1800, 960)
            self.win.request_properties(wp)

            base.set_background_color(0,0,0)

            grid = create_grid( 20, 1 )
            render.attach_new_node( grid )
            axes = create_axes( 1000, bothways=True, thickness=3 )
            render.attach_new_node( axes )

            terrain = CollisionTerrain( 5, 0.25, render, height=1 )

            #####################################
            # Set up character
            self.character = RiggedChar( terrain )
            #self.character2 = Rigged_char( terrain )

            #####################################
            # Set up Camera and input:

            #focus_node = render.attach_new_node( "Camera_focus_node" )
            self.cam_control = CameraControl( camera, self.mouseWatcherNode, speed = 0.02 )
            self.cam_control.focus_point = LVector3f( 0, 0, 1 )
            
            self.taskMgr.add( self.cam_control.move_camera, "Move_camera_task")

            self.accept( "wheel_down", self.cam_control.wheel_down )
            self.accept( "wheel_up", self.cam_control.wheel_up )

            #####################################

            label("[WASD]: Move Camera", 1)
            label("[Mouse Wheel]: Zoom Camera", 2)
            label("[Middle Mouse]: Rotate Camera", 3)
            label("[+]: Speed up", 5)
            label("[-]: Slow down", 6)


    app = MyApp()
    app.run()
