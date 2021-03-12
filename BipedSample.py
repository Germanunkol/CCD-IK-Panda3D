from panda3d.core import *
from IKChain import IKChain
from Utils import *
from WalkCycle import WalkCycle

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
        # Set up body movement:

        self.targetNode = render.attachNewNode( "WalkTarget" )
        geom = createAxes( 0.2 )
        self.targetNode.attachNewNode( geom )
        self.walkSpeed = 1  # m/s
        self.turnSpeed = 2
        self.newRandomTarget()

        ##################################
        # Set up right leg:

        # First, rotate 90 degrees outwards:
        legRootLeft = hipNode.attachNewNode( "LegRootLeft" )
        legRootLeft.setHpr( 90, 0, 0 )

        self.ikChainLegLeft = IKChain( legRootLeft )

        # Hip:
        bone = self.ikChainLegLeft.addBone( offset=LVector3f.unitY()*0.13,
                minAng = -math.pi*0.2,
                maxAng = math.pi*0.2,
                rotAxis = None
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
                maxAng = math.pi*0.6,
                rotAxis = LVector3f.unitZ(),
                parentBone = bone
                )

        # Required!
        self.ikChainLegLeft.finalize()

        self.ikChainLegLeft.debugDisplay()
        

        ##################################
        # Set up right leg:

        # First, rotate 90 degrees outwards:
        legRootRight = hipNode.attachNewNode( "LegRootRight" )
        legRootRight.setHpr( -90, 0, 0 )

        self.ikChainLegRight = IKChain( legRootRight )

        # Hip:
        bone = self.ikChainLegRight.addBone( offset=LVector3f.unitY()*0.13,
                minAng = -math.pi*0.2,
                maxAng = math.pi*0.2,
                rotAxis = None
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
                minAng = -math.pi*0.6,
                maxAng = 0,
                rotAxis = LVector3f.unitZ(),
                parentBone = bone
                )

        # Required!
        self.ikChainLegRight.finalize()

        self.ikChainLegRight.debugDisplay()

        #################################################
        # Foot targets:

        # Set up two targets that the foot should reach:
        self.footTargetLeft = render.attachNewNode("FootTargetLeft")
        self.footTargetRight = render.attachNewNode("FootTargetRight")
        geom = createAxes( 0.1 )

        self.footTargetLeft.attachNewNode( geom )
        self.footTargetRight.attachNewNode( geom )
        self.ikChainLegLeft.setTarget( self.footTargetLeft )
        self.ikChainLegRight.setTarget( self.footTargetRight )

        # Set up two nodes which stay (rigidly) infront of the body, on the floor.
        # Whenever a leg needs to take a step, the target will be placed on this position:
        self.plannedFootTargetLeft = self.torsoNode.attachNewNode( "PlannedFootTargetLeft" )
        self.plannedFootTargetRight = self.torsoNode.attachNewNode( "PlannedFootTargetRight" )

        stepDist = 0.15
        self.plannedFootTargetLeft.setPos( -0.15, stepDist, -self.torsoHeight )
        self.plannedFootTargetRight.setPos( 0.15, stepDist, -self.torsoHeight )
        self.plannedFootTargetLeft.attachNewNode( geom )
        self.plannedFootTargetRight.attachNewNode( geom )

        self.legMovementSpeed = self.walkSpeed*3

        self.stepLeft = False
        self.stepRight = False
        
        self.walkCycle = WalkCycle( 2, 0.75 )

        #################################################

        base.taskMgr.add( self.walk, "BipedWalk")
        base.accept( "+", self.speedUp )
        base.accept( "-", self.slowDown )

    def speedUp( self ):
        self.walkSpeed += 0.5
        self.walkSpeed = min(self.walkSpeed, 3)
        self.turnSpeed = self.walkSpeed*2
        self.legMovementSpeed = self.walkSpeed*3

    def slowDown( self ):
        self.walkSpeed -= 0.5
        self.walkSpeed = max(self.walkSpeed, 0)
        self.turnSpeed = self.walkSpeed*2
        self.legMovementSpeed = self.walkSpeed*3

    def walk( self, task ):

        #############################
        # Update body:

        prevPos = self.torsoNode.getPos()

        diff = self.targetNode.getPos( self.torsoNode )
        diff.z = 0
        diffN = diff.normalized()
        ang = LVector3f.unitY().angleRad( diffN )
        axis = LVector3f.unitY().cross( diffN )
        axis.normalize()
        maxRot = self.turnSpeed*globalClock.getDt()
        angClamped = 0
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

        # Calculate how far we've walked this frame:
        curWalkDist = (prevPos - self.torsoNode.getPos()).length()

        #############################
        # Update legs:

        # Move planned foot target further forward (longer steps) when character is
        # walking faster:
        stepDist = curWalkDist*0.1/globalClock.dt
        self.plannedFootTargetLeft.setPos( -0.15, stepDist, -self.torsoHeight )
        self.plannedFootTargetRight.setPos( 0.15, stepDist, -self.torsoHeight )

        # Update the walkcycle to determine if a step needs to be taken:
        #update = curWalkDist*0.1/globalClock.dt
        update = curWalkDist
        update += angClamped*0.5
        self.walkCycle.updateTime( update )

        if self.walkCycle.stepRequired[0]:
            #self.footTargetLeft.setPos( self.plannedFootTargetLeft.getPos( render ) )
            self.walkCycle.step( 0 )
            self.stepLeft = True
        if self.walkCycle.stepRequired[1]:
            #self.footTargetRight.setPos( self.plannedFootTargetRight.getPos( render ) )
            self.walkCycle.step( 1 )
            self.stepRight = True

        if self.stepLeft:
            diff = self.plannedFootTargetLeft.getPos(render) - self.footTargetLeft.getPos()
            legMoveDist = self.legMovementSpeed*globalClock.dt
            if diff.length() < legMoveDist:
                self.footTargetLeft.setPos( self.plannedFootTargetLeft.getPos( render ) )
                self.stepLeft = False
            else:
                moved = self.footTargetLeft.getPos() + diff.normalized()*legMoveDist
                self.footTargetLeft.setPos( moved )

        if self.stepRight:
            diff = self.plannedFootTargetRight.getPos(render) - self.footTargetRight.getPos()
            legMoveDist = self.legMovementSpeed*globalClock.dt
            if diff.length() < legMoveDist:
                self.footTargetRight.setPos( self.plannedFootTargetRight.getPos( render ) )
                self.stepRight = False
            else:
                moved = self.footTargetRight.getPos() + diff.normalized()*legMoveDist
                self.footTargetRight.setPos( moved )

        self.ikChainLegLeft.updateIK()
        self.ikChainLegRight.updateIK()

        return task.cont

    
    def newRandomTarget( self ):

        self.targetNode.setPos(
                LVector3f( random.random()*10-5,
                    random.random()*10-5,
                    0 ) )




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
