import sys,os
from CCDIK.ik_chain import IKChain
from CCDIK.ik_actor import IKActor
from CCDIK.utils import *
from direct.actor.Actor import Actor
from panda3d.core import *

if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CCDIK.camera_control import CameraControl

    class MyApp(ShowBase):

        def __init__(self):

            ###########################################
            ## Basic window setup:

            ShowBase.__init__(self)
            base.disable_mouse()
            base.set_frame_rate_meter(True)

            wp = WindowProperties()
            wp.set_size(1800, 960)
            self.win.request_properties(wp)

            base.set_background_color(0,0,0)

            ###########################################
            ## Visualize grid:

            grid = create_grid( 20, 1 )
            render.attach_new_node( grid )
            axes = create_axes( 1000, bothways=True, thickness=3 )
            render.attach_new_node( axes )

            ###########################################
            ## Set up lights:

            light = PointLight("PointLight")
            light.set_color_temperature( 5000 )
            #light.attenuation = (1, 0.5, 0.5)
            light.attenuation = (0.75, 0, 0.05)
            light_node = render.attach_new_node( light )
            light_node.set_pos( 0, 0, 3 )
            render.set_light( light_node ) 
            #light.set_shadow_caster(True, 1024, 1024, -2000 ) # low sort value to render early!

            alight = AmbientLight('alight')
            alight.set_color((0.5, 0.5, 0.5, 1))
            alnp = render.attach_new_node(alight)
            render.set_light(alnp)

            ############################################
            ## Set up model:

            self.model = loader.load_model( "Meshes/Tentacle.bam" )

            self.root = render.attach_new_node("Root")
            self.root.set_pos( 0, 0, 2 )

            ############################################
            ## Create an IKActor which will generate control nodes for each bone:

            self.ik_actor = IKActor( self.model )
            self.ik_actor.reparent_to( self.root )

            ############################################
            ## Set up an IK Chain by passing the bone names which should be part of the chain
            ## to the IKActor:
    
            joint_names = []
            joint_names.append( "Bone" )
            for i in range(1,8):
                joint_names.append( f"Bone.{i:03d}" )

            self.ik_chain = self.ik_actor.create_ik_chain( joint_names )
            self.ik_chain.set_annealing_exponent( 3 )

            # Set constraints:
            self.ik_chain.set_ball_constraint( "Bone", min_ang=-math.pi*0.9, max_ang=math.pi*0.9 )
            for i in range(1,8):
                # Set X-axis constraint for bones with even index
                if i % 2 == 0:
                    self.ik_chain.set_hinge_constraint( f"Bone.{i:03d}", LVector3f.unit_x(),
                            min_ang=-math.pi*0.6, max_ang=math.pi*0.6 )
                # Set Z-axis constraint for the others:
                else:
                    self.ik_chain.set_hinge_constraint( f"Bone.{i:03d}", LVector3f.unit_z(),
                            min_ang=-math.pi*0.6, max_ang=math.pi*0.6 )


            # Visualize the joints:
            self.ik_chain.debug_display( line_length=0.5 )

            self.racket = None

            ##################################
            ## Target point:

            point = create_point( thickness=10 )
            self.ik_target = render.attach_new_node( point )
            
            self.task_mgr.add( self.move_target, "MoveTarget" )

            self.ik_chain.set_target( self.ik_target )


            ############################################
            ## Set up camera:
            focus_node = render.attach_new_node( "CameraFocusNode" )
            self.cam_control = CameraControl( camera, self.mouseWatcherNode )
            
            self.taskMgr.add( self.cam_control.move_camera, "MoveCameraTask")

            ############################################
            ## Set up controls:

            self.accept( "wheel_down", self.cam_control.wheel_down )
            self.accept( "wheel_up", self.cam_control.wheel_up )

            self.animate_target = True
            self.animation_time = 0
            self.accept( "p", self.toggle_animation )
            self.accept( "j-repeat", self.move_root_down )
            self.accept( "k-repeat", self.move_root_up )
            self.accept( "j", self.move_root_down )
            self.accept( "k", self.move_root_up )
            self.accept( "1", self.set_hinge_constraints )
            self.accept( "2", self.set_ball_constraints )

            self.accept( "r", self.toggle_racket )

            self.accept( "+", self.increase_annealing_exponent )
            self.accept( "-", self.decrease_annealing_exponent )
        
            label("[WASD]: Move Camera", 1)
            label("[Mouse Wheel]: Zoom Camera", 2)
            label("[Middle Mouse]: Rotate Camera", 3)
            label("[P]: Pause Animation", 5)
            label("[J]: Move Root Down", 6)
            label("[K]: Move Root Up", 7)
            label("[1]: Use Hinge Constraints", 8)
            label("[2]: Use Ball Constraints", 9)
            label("[R]: Attach a racket to the end effector bone", 11)

            label("[+]: Increase annealing exponent", 13)
            label("[-]: Decrease annealing exponent", 14)

            self.info_texts = {}
            self.update_info()

            print("---------------------------------")
            print("Full tree:")
            render.ls()
            print("---------------------------------")

        def move_target( self, task ):
            if self.animate_target:
                speed = 0.4
                self.ik_target.set_pos( 2.5*math.sin(speed*self.animation_time),
                        13*math.sin(speed*self.animation_time*1.6+2),
                        math.cos(speed*self.animation_time*1.6+2) )

                self.animation_time += globalClock.get_dt()

            self.ik_chain.update_ik()
            return task.cont

        def set_hinge_constraints( self ):

            # Set constraints:
            self.ik_chain.set_hinge_constraint( "Bone", LVector3f.unit_z(),
                    min_ang=-math.pi*0.9, max_ang=math.pi*0.9 )
            for i in range(1,8):
                # Set X-axis constraint for bones with even index
                if i % 2 == 0:
                    self.ik_chain.set_hinge_constraint( f"Bone.{i:03d}", LVector3f.unit_x(),
                            min_ang=-math.pi*0.6, max_ang=math.pi*0.6 )
                # Set Z-axis constraint for the others:
                else:
                    self.ik_chain.set_hinge_constraint( f"Bone.{i:03d}", LVector3f.unit_z(),
                            min_ang=-math.pi*0.6, max_ang=math.pi*0.6 )

            self.ik_chain.debug_display( line_length=0.5 )

        def set_ball_constraints( self ):

            # Set constraints:
            self.ik_chain.set_ball_constraint( "Bone",
                    min_ang=-math.pi*0.9, max_ang=math.pi*0.9 )
            for i in range(1,8):
                self.ik_chain.set_ball_constraint( f"Bone.{i:03d}",
                        min_ang=-math.pi*0.6, max_ang=math.pi*0.6 )

            self.ik_chain.debug_display( line_length=0.5 )

        def toggle_racket( self ):
            if self.racket:
                self.racket.remove_node()
                self.racket = None
            else:
                self.racket = create_racket()
                end_effector = self.ik_chain.get_ik_joint( f"Bone.007" )
                self.racket.reparent_to( end_effector.control_node )

        def increase_annealing_exponent( self ):
            cur = self.ik_chain.annealing_exponent
            self.ik_chain.set_annealing_exponent( cur + 1 )
            self.update_info()
        def decrease_annealing_exponent( self ):
            cur = self.ik_chain.annealing_exponent
            self.ik_chain.set_annealing_exponent( cur - 1 )
            self.update_info()

        def move_root_up( self ):
            self.root.set_pos( self.root.get_pos() + LVector3f.unit_z()*globalClock.get_dt()*3 )

        def move_root_down( self ):
            self.root.set_pos( self.root.get_pos() - LVector3f.unit_z()*globalClock.get_dt()*3 )

        def toggle_animation( self ):
            self.animate_target = (self.animate_target == False)

        def update_info( self ):
            for k, v in self.info_texts.items():
                v.remove_node()
            self.info_texts["annealing"] = \
                    info( f"Annealing exponent: {self.ik_chain.annealing_exponent}", 1 )

    app = MyApp()
    app.run()
