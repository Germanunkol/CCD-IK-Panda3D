from panda3d.core import *
from IKChain import IKChain
from Utils import *

class Biped():

    def __init__( self ):

        ##################################
        # Set up main body:

        self.torsoHeight = 1.6
        self.torsoNode = render.attachNewNode("Torso")
        self.torsoNode.setPos( 0, 0, self.torsoHeight )
        geom = createAxes( 0.3 )
        self.torsoNode.attachNewNode( geom )
        
        hipNode = self.torsoNode.attachNewNode( "Hip" )
        hipNode.setPos( 0, 0, -0.6 )

        ##################################
        # Set up right leg:
        self.ikChainLegLeft = IKChain( hipNode )

        # Hip:
        bone = self.ikChainLegLeft.addBone( offset=LVector3f.unitY()*0.13,
                minAng = math.pi*0.4,
                maxAng = math.pi*0.6,
                rotAxis = LVector3f.unitZ()
                )

        # We want a fixed 90° angle between the hip node and the thigh, so
        # rotate down:
        bone = self.ikChainLegLeft.addBone( offset=LVector3f.zero(),
                minAng = -math.pi*0.5,
                maxAng = -math.pi*0.5,
                rotAxis = LVector3f.unitX(),
                parentBone = bone
                )

        # Thigh:
        bone = self.ikChainLegLeft.addBone( offset=LVector3f.unitY()*0.45,
                minAng = -math.pi*0.3,
                maxAng = math.pi*0.3,
                rotAxis = LVector3f.unitZ(),
                parentBone = bone
                )
        
        # Shin:
        bone = self.ikChainLegLeft.addBone( offset=LVector3f.unitY()*0.55,
                minAng = 0,
                maxAng = math.pi*0.3,
                rotAxis = LVector3f.unitZ(),
                parentBone = bone
                )

        # Required!
        self.ikChainLegLeft.finalize()

        self.ikChainLegLeft.debugDisplay()
        

        ##################################
        # Set up right leg:
        self.ikChainLegRight = IKChain( hipNode )

        # Hip:
        bone = self.ikChainLegRight.addBone( offset=LVector3f.unitY()*0.13,
                minAng = -math.pi*0.6,
                maxAng = -math.pi*0.4,
                rotAxis = LVector3f.unitZ()
                )

        # We want a fixed 90° angle between the hip node and the thigh, so
        # rotate down:
        bone = self.ikChainLegRight.addBone( offset=LVector3f.zero(),
                minAng = -math.pi*0.5,
                maxAng = -math.pi*0.5,
                rotAxis = LVector3f.unitX(),
                parentBone = bone
                )

        # Thigh:
        bone = self.ikChainLegRight.addBone( offset=LVector3f.unitY()*0.45,
                minAng = -math.pi*0.3,
                maxAng = math.pi*0.3,
                rotAxis = LVector3f.unitZ(),
                parentBone = bone
                )
        
        # Shin:
        bone = self.ikChainLegRight.addBone( offset=LVector3f.unitY()*0.55,
                minAng = -math.pi*0.3,
                maxAng = 0,
                rotAxis = LVector3f.unitZ(),
                parentBone = bone
                )

        # Required!
        self.ikChainLegRight.finalize()

        self.ikChainLegRight.debugDisplay()

        #################################################
        # Foot targets:

        self.footTargetLeft = render.attachNewNode("FootTargetLeft")
        self.footTargetRight = render.attachNewNode("FootTargetRight")
        geom = createAxes( 0.1 )
        self.footTargetLeft.attachNewNode( geom )
        self.footTargetRight.attachNewNode( geom )
        self.ikChainLegLeft.setTarget( self.footTargetLeft )
        self.ikChainLegRight.setTarget( self.footTargetRight )

        #################################################

        self.targetNode = render.attachNewNode( "WalkTarget" )
        geom = createAxes( 0.2 )
        self.targetNode.attachNewNode( geom )
        self.walkSpeed = 1.5  # m/s
        self.turnSpeed = 1
        self.newRandomTarget()
        
        base.taskMgr.add( self.walk, "BipedWalk")

    def walk( self, task ):

        diff = self.targetNode.getPos( self.torsoNode )
        diff.z = 0
        diffN = diff.normalized()
        ang = LVector3f.unitY().angleRad( diffN )
        axis = LVector3f.unitY().cross( diffN )
        axis.normalize()
        maxRot = self.turnSpeed*globalClock.getDt()
        if axis.length() > 0.999:
            # Limit angle:
            angClamped = max( -maxRot, min( maxRot, ang ) )
            q = Quat()
            q.setFromAxisAngleRad( angClamped, axis )

            qOld = self.torsoNode.getQuat()
            qNew = q*qOld
            self.torsoNode.setQuat( qNew  )
        if abs( ang ) < maxRot:
            step = diffN*self.walkSpeed*globalClock.getDt()
            if step.lengthSquared() > diff.lengthSquared():
                self.newRandomTarget()
                step = diff
            step = self.torsoNode.getQuat().xform( step )
            self.torsoNode.setPos( self.torsoNode.getPos() + step )
 

        self.ikChainLegLeft.updateIK()
        self.ikChainLegRight.updateIK()

        return task.cont

    
    def newRandomTarget( self ):

        self.targetNode.setPos(
                LVector3f( random.random()*10-5,
                    random.random()*10-5,
                    self.torsoHeight ) )




if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CameraControl import CameraControl

    class MyApp(ShowBase):

        def __init__(self):

            #####################################
            ## Set up scene

            ShowBase.__init__(self)
            base.disableMouse()

            wp = WindowProperties()
            wp.setSize(1800, 960)
            self.win.requestProperties(wp)

            base.setBackgroundColor(0,0,0)

            grid = createGrid( 20, 1 )
            render.attachNewNode( grid )
            axes = createAxes( 1000, bothways=True, thickness=3 )
            render.attachNewNode( axes )

            #####################################
            # Set up Biped
            self.biped = Biped()

            #####################################
            # Set up Camera and input:

            focusNode = render.attachNewNode( "CameraFocusNode" )
            self.camControl = CameraControl( camera, self.mouseWatcherNode )
            
            self.taskMgr.add( self.camControl.moveCamera, "MoveCameraTask")

            self.accept( "wheel_down", self.camControl.wheelDown )
            self.accept( "wheel_up", self.camControl.wheelUp )

            self.accept( "z", self.toggleAnimation )
            self.animateTarget = True

        def toggleAnimation( self ):
            self.animateTarget = (self.animateTarget == False)


    app = MyApp()
    app.run()
