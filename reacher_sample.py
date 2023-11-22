from CCDIK.ik_chain import IKChain
from CCDIK.armature_utils import ArmatureUtils
from CCDIK.utils import *


if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CCDIK.camera_control import CameraControl

    class MyApp(ShowBase):

        def __init__(self):
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

            root = render.attach_new_node("Root")
            root.set_pos( 1, 1, 1 )

            #######################################
            ## Set up the joints using an ArmatureUtils instance:
            au = ArmatureUtils()
            
            joint = None
            for i in range( 6 ):
                offset_length = 0.5
                if i == 0:
                    offset_length = 0.1
                joint = au.create_joint( f"joint{i}",
                        parent_joint = joint, translate=LVector3f.unit_x()*offset_length,
                        )

            au.finalize()

            ## IMPORTANT! Attach the created actor to the scene, otherwise you won't see anything!
            au.get_actor().reparent_to( render )

            #######################################
            ## Create an IK chain from the armature:

            self.ik_chain = IKChain()
            self.ik_chain.set_annealing_exponent( 4 )

            ik_joint = None
            for i in range( 6 ):
                name = f"joint{i}"
                # Now we can conveniently retrieve everything we need from the Armature_utils...
                joint = au.get_joint( name )
                control_node = au.get_control_node( name )
                # ... and add it to the chain:
                ik_joint = self.ik_chain.add_joint( joint, control_node, parent_ik_joint=ik_joint )
                if i < 4:
                    self.ik_chain.get_ik_joint( name ).set_hinge_constraint( LVector3f.unit_z(),
                            min_ang=-math.pi*0.25, max_ang=math.pi*0.25 )
                else:
                    self.ik_chain.get_ik_joint( name ).set_hinge_constraint( LVector3f.unit_y(),
                            min_ang=-math.pi*0.25, max_ang=math.pi*0.25 )

            self.ik_chain.debug_display()


            ############################################
            ## Set up camera:
            focus_node = render.attach_new_node( "Camera_focus_node" )
            self.cam_control = CameraControl( camera, self.mouseWatcherNode )
            
            self.task_mgr.add( self.cam_control.move_camera, "Move_camera_task")

            ##################################
            ## Target point:

            point = create_point( thickness=10 )

            self.ik_target = render.attach_new_node( point )
            self.ik_target.set_pos( 2,0,2 )
            
            self.task_mgr.add( self.move_target, "Move_target" )

            self.ik_chain.set_target( self.ik_target )

            ############################################
            ## Placeholder:
            self.racket = None

            ############################################
            ## Set up controls:

            self.accept( "wheel_down", self.cam_control.wheel_down )
            self.accept( "wheel_up", self.cam_control.wheel_up )

            self.accept( "p", self.toggle_animation )
            self.animate_target = True

            self.accept( "r", self.toggle_racket )

            self.accept( "+", self.increase_annealing_exponent )
            self.accept( "-", self.decrease_annealing_exponent )

            label("[WASD]: Move Camera", 1)
            label("[Mouse Wheel]: Zoom Camera", 2)
            label("[Middle Mouse]: Rotate Camera", 3)
            label("[P]: Pause Animation", 5)
            label("[R]: Attach a racket to the end effector bone", 7)

            label("[+]: Increase annealing exponent", 9)
            label("[-]: Decrease annealing exponent", 10)

            self.info_texts = {}
            self.update_info()


        def move_target( self, task ):
            if self.animate_target:
                speed = 1
                self.ik_target.set_pos(
                        4*math.sin(speed*task.time*1.6+2),
                        2.5*math.sin(speed*task.time),
                        math.cos(speed*task.time*1.6+2) )

            self.ik_chain.update_ik()
            return task.cont

        def toggle_racket( self ):
            if self.racket:
                self.racket.remove_node()
                self.racket = None
            else:
                self.racket = create_racket()
                end_effector = self.ik_chain.get_ik_joint( "joint5" )
                self.racket.reparent_to( end_effector.control_node )

        def increase_annealing_exponent( self ):
            cur = self.ik_chain.annealing_exponent
            self.ik_chain.set_annealing_exponent( cur + 1 )
            self.update_info()
        def decrease_annealing_exponent( self ):
            cur = self.ik_chain.annealing_exponent
            self.ik_chain.set_annealing_exponent( cur - 1 )
            self.update_info()


        def toggle_animation( self ):
            self.animate_target = (self.animate_target == False)

        def update_info( self ):
            for k, v in self.info_texts.items():
                v.remove_node()
            self.info_texts["annealing"] = \
                    info( f"Annealing exponent: {self.ik_chain.annealing_exponent}", 1 )

    app = MyApp()
    app.run()
