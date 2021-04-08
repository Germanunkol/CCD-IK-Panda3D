import sys,os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from IKChain import IKChain
from ArmatureUtils import ArmatureUtils
from Utils import *

if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CameraControl import CameraControl

    class MyApp(ShowBase):

        def __init__(self):
            ShowBase.__init__(self)
            base.disableMouse()
            base.setFrameRateMeter(True)

            wp = WindowProperties()
            wp.setSize(1800, 960)
            self.win.requestProperties(wp)

            base.setBackgroundColor(0,0,0)

            grid = createGrid( 20, 1 )
            render.attachNewNode( grid )
            axes = createAxes( 1000, bothways=True, thickness=3 )
            render.attachNewNode( axes )

            root = render.attachNewNode("Root")
            root.setPos( 1, 1, 1 )

            #######################################
            ## Set up the joints using an ArmatureUtils instance:
            au = ArmatureUtils()
            
            joint = None
            for i in range( 6 ):
                offsetLength = 0.5
                if i == 0:
                    offsetLength = 0.1
                joint = au.createJoint( f"joint{i}",
                        parentJoint = joint, translate=LVector3f.unitX()*offsetLength,
                        )

            au.finalize()

            ## IMPORTANT! Attach the created actor to the scene, otherwise you won't see anything!
            au.getActor().reparentTo( render )

            #######################################
            ## Create an IK chain from the armature:

            self.ikChain = IKChain( au.getActor() )

            bone = None
            for i in range( 6 ):
                name = f"joint{i}"
                # Now we can conveniently retrieve everything we need from the ArmatureUtils...
                joint = au.getJoint( name )
                controlNode = au.getControlNode( name )
                # ... and add it to the chain:
                bone = self.ikChain.addJoint( joint, controlNode, parentBone=bone )
                if i < 4:
                    self.ikChain.setHingeConstraint( name, LVector3f.unitZ(),
                            minAng=-math.pi*0.25, maxAng=math.pi*0.25 )
                else:
                    self.ikChain.setHingeConstraint( name, LVector3f.unitY(),
                            minAng=-math.pi*0.25, maxAng=math.pi*0.25 )

            self.ikChain.debugDisplay()


            ############################################
            ## Set up camera:
            focusNode = render.attachNewNode( "CameraFocusNode" )
            self.camControl = CameraControl( camera, self.mouseWatcherNode )
            
            self.taskMgr.add( self.camControl.moveCamera, "MoveCameraTask")

            ##################################
            ## Target point:

            point = createPoint( thickness=10 )

            self.ikTarget = render.attachNewNode( point )
            self.ikTarget.setPos( 2,0,2 )
            
            self.taskMgr.add( self.moveTarget, "MoveTarget" )

            self.ikChain.setTarget( self.ikTarget )

            ############################################
            ## Set up controls:

            self.accept( "wheel_down", self.camControl.wheelDown )
            self.accept( "wheel_up", self.camControl.wheelUp )

            self.accept( "p", self.toggleAnimation )
            self.animateTarget = True

            self.accept( "r", self.toggleRacket )
            self.racket = None

            label("[WASD]: Move Camera", 1)
            label("[Mouse Wheel]: Zoom Camera", 2)
            label("[Middle Mouse]: Rotate Camera", 3)
            label("[P]: Pause Animation", 5)
            label("[R]: Attach a racket to the end effector bone", 7)

        def moveTarget( self, task ):
            if self.animateTarget:
                speed = 1
                self.ikTarget.setPos(
                        4*math.sin(speed*task.time*1.6+2),
                        2.5*math.sin(speed*task.time),
                        math.cos(speed*task.time*1.6+2) )

            self.ikChain.updateIK()
            return task.cont

        def toggleRacket( self ):
            if self.racket:
                self.racket.removeNode()
                self.racket = None
            else:
                self.racket = createRacket()
                endEffector = self.ikChain.getBone( "joint5" )
                self.racket.reparentTo( endEffector.controlNode )



        def toggleAnimation( self ):
            self.animateTarget = (self.animateTarget == False)


    app = MyApp()
    app.run()
