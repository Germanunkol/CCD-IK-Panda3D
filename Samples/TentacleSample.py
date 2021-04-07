import sys,os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from IKChain import IKChain
from IKActor import IKActor
from Utils import *
from direct.actor.Actor import Actor
from direct.gui.OnscreenText import OnscreenText
from panda3d.core import *

# Macro-like function used to reduce the amount to code needed to create the
# on screen instructions. Shamelessly stolen from the Panda3D samples.
def genLabelText(text, i):
    return OnscreenText(text=text, parent=base.a2dTopLeft, scale=.06,
                        pos=(0.06, -.08 * i), fg=(1, 1, 1, 1),
                        shadow=(0, 0, 0, .5), align=TextNode.ALeft)

if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CameraControl import CameraControl

    class MyApp(ShowBase):

        def __init__(self):

            ###########################################
            ## Basic window setup:

            ShowBase.__init__(self)
            base.disableMouse()
            base.setFrameRateMeter(True)

            wp = WindowProperties()
            wp.setSize(1800, 960)
            self.win.requestProperties(wp)

            base.setBackgroundColor(0,0,0)

            ###########################################
            ## Visualize grid:

            grid = createGrid( 20, 1 )
            render.attachNewNode( grid )
            axes = createAxes( 1000, bothways=True, thickness=3 )
            render.attachNewNode( axes )

            ###########################################
            ## Set up lights:

            light = PointLight("PointLight")
            light.setColorTemperature( 5000 )
            #light.attenuation = (1, 0.5, 0.5)
            light.attenuation = (0.75, 0, 0.05)
            lightNode = render.attachNewNode( light )
            lightNode.setPos( 0, 0, 3 )
            render.setLight( lightNode ) 
            #light.setShadowCaster(True, 1024, 1024, -2000 ) # low sort value to render early!

            alight = AmbientLight('alight')
            alight.setColor((0.5, 0.5, 0.5, 1))
            alnp = render.attachNewNode(alight)
            render.setLight(alnp)

            ############################################
            ## Set up model:

            self.model = loader.loadModel( "Meshes/Tentacle.bam" )

            self.root = render.attachNewNode("Root")
            self.root.setPos( 0, 0, 2 )

            ############################################
            ## Create an IKActor which will generate control nodes for each bone:

            self.ikActor = IKActor( self.model )
            self.ikActor.reparentTo( self.root )

            ############################################
            ## Set up an IK Chain by passing the bone names which should be part of the chain
            ## to the IKActor:
    
            jointNames = []
            jointNames.append( "Bone" )
            for i in range(1,8):
                jointNames.append( f"Bone.{i:03d}" )

            self.ikChain = self.ikActor.createIKChain( jointNames )

            # Set constraints:
            self.ikChain.setBallConstraint( "Bone", minAng=-math.pi*0.9, maxAng=math.pi*0.9 )
            for i in range(1,8):
                # Set X-axis constraint for bones with even index
                if i % 2 == 0:
                    self.ikChain.setHingeConstraint( f"Bone.{i:03d}", LVector3f.unitX(),
                            minAng=-math.pi*0.6, maxAng=math.pi*0.6 )
                # Set Z-axis constraint for the others:
                else:
                    self.ikChain.setHingeConstraint( f"Bone.{i:03d}", LVector3f.unitZ(),
                            minAng=-math.pi*0.6, maxAng=math.pi*0.6 )


            # Visualize the joints:
            self.ikChain.debugDisplay( lineLength=0.5 )

            ##################################
            ## Target point:

            point = createPoint( thickness=10 )
            self.ikTarget = render.attachNewNode( point )
            
            self.taskMgr.add( self.moveTarget, "MoveTarget" )

            self.ikChain.setTarget( self.ikTarget )


            ############################################
            ## Set up camera:
            focusNode = render.attachNewNode( "CameraFocusNode" )
            self.camControl = CameraControl( camera, self.mouseWatcherNode )
            
            self.taskMgr.add( self.camControl.moveCamera, "MoveCameraTask")

            ############################################
            ## Set up controls:

            self.accept( "wheel_down", self.camControl.wheelDown )
            self.accept( "wheel_up", self.camControl.wheelUp )

            self.animateTarget = True
            self.animationTime = 0
            self.accept( "p", self.toggleAnimation )
            self.accept( "j-repeat", self.moveRootDown )
            self.accept( "k-repeat", self.moveRootUp )
            self.accept( "j", self.moveRootDown )
            self.accept( "k", self.moveRootUp )
            self.accept( "1", self.setHingeConstraints )
            self.accept( "2", self.setBallConstraints )
        
            self.onekeyText = genLabelText("[WASD]: Move Camera", 1)
            self.onekeyText = genLabelText("[Mouse Wheel]: Zoom Camera", 2)
            self.onekeyText = genLabelText("[p]: Pause Animation", 3)
            self.onekeyText = genLabelText("[j]: Move Root Up", 4)
            self.onekeyText = genLabelText("[k]: Move Root Down", 5)
            self.onekeyText = genLabelText("[1]: Use Hinge Constraints", 6)
            self.onekeyText = genLabelText("[2]: Use Ball Constraints", 7)

            print("---------------------------------")
            print("Full tree:")
            render.ls()
            print("---------------------------------")

        def moveTarget( self, task ):
            if self.animateTarget:
                speed = 0.4
                self.ikTarget.setPos( 2.5*math.sin(speed*self.animationTime),
                        13*math.sin(speed*self.animationTime*1.6+2),
                        math.cos(speed*self.animationTime*1.6+2) )

                self.animationTime += globalClock.getDt()

            self.ikChain.updateIK()
            return task.cont

        def setHingeConstraints( self ):

            # Set constraints:
            self.ikChain.setHingeConstraint( "Bone", LVector3f.unitZ(),
                    minAng=-math.pi*0.9, maxAng=math.pi*0.9 )
            for i in range(1,8):
                # Set X-axis constraint for bones with even index
                if i % 2 == 0:
                    self.ikChain.setHingeConstraint( f"Bone.{i:03d}", LVector3f.unitX(),
                            minAng=-math.pi*0.6, maxAng=math.pi*0.6 )
                # Set Z-axis constraint for the others:
                else:
                    self.ikChain.setHingeConstraint( f"Bone.{i:03d}", LVector3f.unitZ(),
                            minAng=-math.pi*0.6, maxAng=math.pi*0.6 )

            self.ikChain.debugDisplay( lineLength=0.5 )

        def setBallConstraints( self ):

            # Set constraints:
            self.ikChain.setBallConstraint( "Bone",
                    minAng=-math.pi*0.9, maxAng=math.pi*0.9 )
            for i in range(1,8):
                self.ikChain.setBallConstraint( f"Bone.{i:03d}",
                        minAng=-math.pi*0.6, maxAng=math.pi*0.6 )

            self.ikChain.debugDisplay( lineLength=0.5 )


        def moveRootUp( self ):
            self.root.setPos( self.root.getPos() + LVector3f.unitZ()*globalClock.getDt()*3 )

        def moveRootDown( self ):
            self.root.setPos( self.root.getPos() - LVector3f.unitZ()*globalClock.getDt()*3 )

        def toggleAnimation( self ):
            self.animateTarget = (self.animateTarget == False)


    app = MyApp()
    app.run()
