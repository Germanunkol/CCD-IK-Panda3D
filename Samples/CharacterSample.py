from panda3d.core import *
import sys,os
sys.path.insert(1, os.path.join(sys.path[0], '..'))
from IKChain import IKChain
from IKActor import IKActor
from Utils import *
from WalkCycle import WalkCycle
from FootArc import FootArc
from CollisionTerrain import CollisionTerrain
from direct.actor.Actor import Actor

class RiggedChar():

    def __init__( self, terrain ):


        ##################################
        # Set up main body:

        self.rootNode = render.attachNewNode("Torso")
        geom = createAxes( 0.3 )
        self.rootNode.attachNewNode( geom )

        # How high the root node should currently be (Determined by the average position of all
        # grounded feet):
        self.rootHeight = 0.956756    # Distance between root node and the ground
        #self.rootHeight = 0
        self.rootNode.setPos( 0, 0, self.rootHeight )
        self.curTargetHeight = self.rootHeight


        ##################################
        # Set up body movement:

        self.targetNode = render.attachNewNode( "WalkTarget" )
        #geom = createAxes( 0.2 )
        #self.targetNode.attachNewNode( geom )
        self.walkSpeed = 0.5  # m/s
        self.turnSpeed = 1
        self.heightAdjustmentSpeed = self.walkSpeed
        self.newRandomTarget()

        ##################################
        # Set up legs:

        self.model = loader.loadModel( "Meshes/person.bam" )

        # Standard material:
        m = Material()
        m.setBaseColor((0.1, 0.5, 0.1, 1))
        m.setAmbient((0.1,0.1,0.1,1))
        m.setSpecular((0.1,0.7,0.1,1))

        self.ikActor = IKActor( self.model )
        self.ikActor.reparentTo( self.rootNode )
        self.ikActor.actor.setMaterial(m)

        #render.find("**/Body").setMaterial(m)
        #render.find("**/Body").setShaderAuto()

#        rootBone = actor.exposeJoint(None, "modelRoot", "Bone")
#        rootBoneControl = actor.controlJoint(None, "modelRoot", "Bone")
#        rootBoneControl.setHpr( 45, 45, 45 )

        self.ikChainLegLeft = self.ikActor.createIKChain( ["Hip.L", "UpperLeg.L", "LowerLeg.L", "Foot.L"] )
        #self.ikChainLegLeft.setStatic( "Hips" )
        self.ikChainLegLeft.setHingeConstraint( "Hip.L", axis=LVector3f.unitZ(),
                minAng=-math.pi*0.05, maxAng=math.pi*0.05 )
        self.ikChainLegLeft.setHingeConstraint( "UpperLeg.L", axis=LVector3f.unitX(),
                minAng=-math.pi*0.2, maxAng=math.pi*0.2 )
        self.ikChainLegLeft.setHingeConstraint( "LowerLeg.L", axis=LVector3f.unitX(),
                minAng=-math.pi*0.5, maxAng=-math.pi*0.05 )
        self.ikChainLegLeft.setHingeConstraint( "Foot.L", axis=LVector3f.unitX(),
                minAng=-math.pi*0.5, maxAng=math.pi*0.5 )


        self.ikChainLegRight = self.ikActor.createIKChain( ["Hip.R", "UpperLeg.R", "LowerLeg.R", "Foot.R"] )
        #self.ikChainLegRight.setStatic( "Hips" )
        self.ikChainLegRight.setHingeConstraint( "Hip.R", axis=LVector3f.unitZ(),
                minAng=-math.pi*0.05, maxAng=math.pi*0.05 )
        self.ikChainLegRight.setHingeConstraint( "UpperLeg.R", axis=LVector3f.unitX(),
                minAng=-math.pi*0.2, maxAng=math.pi*0.2 )
        self.ikChainLegRight.setHingeConstraint( "LowerLeg.R", axis=LVector3f.unitX(),
                minAng=-math.pi*0.5, maxAng=-math.pi*0.05 )
        self.ikChainLegRight.setHingeConstraint( "Foot.R", axis=LVector3f.unitX(),
                minAng=-math.pi*0.5, maxAng=math.pi*0.5 )


        #self.ikChainLegLeft.updateIK()
        #self.ikChainLegRight.updateIK()

        #################################################
        # Set up arm chains:

        self.ikChainArmLeft = self.ikActor.createIKChain( ["Shoulder.L", "UpperArm.L", "LowerArm.L", "Hand.L"] )
        self.ikChainArmLeft.setHingeConstraint( "Shoulder.L", axis=LVector3f.unitZ(),
                minAng=math.pi*0.05, maxAng=math.pi*0.05 )
        self.ikChainArmLeft.setHingeConstraint( "UpperArm.L", axis=LVector3f.unitY(),
                minAng=-math.pi*0.5, maxAng=math.pi*0.5 )
        self.ikChainArmLeft.setHingeConstraint( "LowerArm.L", axis=LVector3f.unitZ(),
                minAng=-math.pi*0.5, maxAng=0 )
        self.ikChainArmLeft.setHingeConstraint( "Hand.L", axis=LVector3f.unitX(),
                minAng=-math.pi*0.3, maxAng=math.pi*0.3 )

        self.ikChainArmRight = self.ikActor.createIKChain( ["Shoulder.R", "UpperArm.R", "LowerArm.R", "Hand.R"] )
        self.ikChainArmRight.setHingeConstraint( "Shoulder.R", axis=LVector3f.unitZ(),
                minAng=math.pi*0.05, maxAng=math.pi*0.05 )
        self.ikChainArmRight.setHingeConstraint( "UpperArm.R", axis=LVector3f.unitY(),
                minAng=-math.pi*0.5, maxAng=math.pi*0.5 )
        self.ikChainArmRight.setHingeConstraint( "LowerArm.R", axis=LVector3f.unitZ(),
                minAng=0, maxAng=math.pi*0.5 )
        self.ikChainArmRight.setHingeConstraint( "Hand.R", axis=LVector3f.unitX(),
                minAng=-math.pi*0.3, maxAng=math.pi*0.3 )



        ############################

        self.ikChainLegLeft.debugDisplay( lineLength=0.1 )
        self.ikChainLegRight.debugDisplay( lineLength=0.1 )
        self.ikChainArmLeft.debugDisplay( lineLength=0.1 )
        self.ikChainArmRight.debugDisplay( lineLength=0.1 )

        #################################################
        # Foot targets:

        # Set up two targets that the foot should reach:
        self.footTargetLeft = render.attachNewNode("FootTargetLeft")
        self.footTargetRight = render.attachNewNode("FootTargetRight")
        geom = createAxes( 0.15 )
        self.footTargetLeft.attachNewNode( geom )
        self.footTargetRight.attachNewNode( geom )
        self.ikChainLegLeft.setTarget( self.footTargetLeft )
        self.ikChainLegRight.setTarget( self.footTargetRight )
    
        # xRay:
        self.footTargetLeft.setBin("fixed", 0)
        self.footTargetLeft.setDepthTest(False)
        self.footTargetLeft.setDepthWrite(False)
        self.footTargetRight.setBin("fixed", 0)
        self.footTargetRight.setDepthTest(False)
        self.footTargetRight.setDepthWrite(False)
 

        # Set up two nodes which stay (rigidly) infront of the body, on the floor.
        # Whenever a leg needs to take a step, the target will be placed on this position:
        self.plannedRotation = self.rootNode.attachNewNode( "PlannedRotationNode" )
        self.plannedFootTargetLeft = self.plannedRotation.attachNewNode( "PlannedFootTargetLeft" )
        self.plannedFootTargetRight = self.plannedRotation.attachNewNode( "PlannedFootTargetRight" )
        # Get distance from root bone to foot bone. This is the length of the leg, i.e. it tells
        # us how far the planned foot position should be away from the root:
        footNode = self.ikActor.getControlNode( "Foot.L" )
        footPos = footNode.getPos( self.rootNode )
        footHeight = self.rootHeight + footPos.getZ()
        self.footOutwards = abs(footPos.getX())
        self.footHeightOffset = LVector3f(0,0,footHeight)
        print("Foot height:", footHeight, self.rootHeight, footNode, footNode.getPos( self.rootNode ))

        self.stepDist = 0.35     # Length of a step
        self.plannedFootTargetLeft.setPos( -self.footOutwards, self.stepDist, 0 )
        self.plannedFootTargetRight.setPos( self.footOutwards, self.stepDist, 0 )
        self.plannedFootTargetLeft.attachNewNode( geom )
        self.plannedFootTargetRight.attachNewNode( geom )

        self.legMovementSpeed = self.walkSpeed*2

        self.stepArcLeft = None
        self.stepArcRight = None
        
        self.walkCycle = WalkCycle( 2, 0.5 )

        #self.noise = PerlinNoise2()
        #self.noise.setScale( 0.1, 0.1 )

        self.handBasePosLeft = self.rootNode.attachNewNode("HandTargetLeft")
        self.handBasePosLeft.setPos( -0.3, 0, -0.3 )
        self.handTargetLeft = self.handBasePosLeft.attachNewNode("HandTargetLeft")

        self.handBasePosRight = self.rootNode.attachNewNode("HandTargetRight")
        self.handBasePosRight.setPos( 0.3, 0, -0.3 )
        self.handTargetRight = self.handBasePosRight.attachNewNode("HandTargetRight")

        self.ikChainArmLeft.setTarget( self.handTargetLeft )
        self.ikChainArmRight.setTarget( self.handTargetRight )

        self.handTargetLeft.attachNewNode( geom )
        self.handTargetRight.attachNewNode( geom )
        

        ###########################################
        ## Set up lights:

        light = PointLight("PointLight")
        light.setColorTemperature( 9000 )
        #light.attenuation = (1, 0.5, 0.5)
        light.attenuation = (0.75, 0, 0.05)
        lightNode = render.attachNewNode( light )
        lightNode.setPos( 0, 0, 3 )
        render.setLight( lightNode ) 
        #light.setShadowCaster(True, 1024, 1024, -2000 ) # low sort value to render early!

        alight = AmbientLight('alight')
        alight.setColor((0.2, 0.3, 0.2, 1))
        alnp = render.attachNewNode(alight)
        render.setLight(alnp)

        #################################################


        base.taskMgr.add( self.walk, "RiggedCharWalk")
        base.accept( "+", self.speedUp )
        base.accept( "-", self.slowDown )

        base.accept( "p", self.printChain )

        ##################################
        # Set up collision:
        self.terrain = terrain
        
        self.collisionRay = CollisionRay()
        self.collisionRay.direction = -LVector3f.unitZ()    # Trace down
        cn = CollisionNode( "RootRayNode" )
        cn.addSolid( self.collisionRay )
        self.rayNode = self.rootNode.attachNewNode( cn )

        self.collisionTraverser = CollisionTraverser()
        self.collisionQueue = CollisionHandlerQueue()
        self.collisionTraverser.addCollider( self.rayNode, self.collisionQueue )
        #self.collisionTraverser.traverse( self.cave.collisionFloorRoot )
        
        ##################################
        # Control upper body:
        #self.torsoBone = self.ikActor.getControlNode( "Torso" )
        self.torsoBone = self.ikActor.getControlNode( "LowerSpine" )
        #self.torsoBone.setHpr( 0, -2, 0 )

        #self.rootBone = self.ikActor.getControlNode( "Hips" )

    def speedUp( self ):
        self.walkSpeed += 0.25
        self.walkSpeed = min(self.walkSpeed, 3)
        self.turnSpeed = self.walkSpeed*2
        self.heightAdjustmentSpeed = self.walkSpeed
        self.legMovementSpeed = 0.3 + self.walkSpeed*1.2

    def slowDown( self ):
        self.walkSpeed -= 0.25
        self.walkSpeed = max(self.walkSpeed, 0)
        self.turnSpeed = self.walkSpeed*2
        self.heightAdjustmentSpeed = self.walkSpeed
        self.legMovementSpeed = 0.3 + self.walkSpeed*1.2

    def walk( self, task ):

        #############################
        # Update body:

        prevPos = self.rootNode.getPos()

        diff = self.targetNode.getPos( self.rootNode )
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

            qOld = self.rootNode.getQuat()
            qNew = q*qOld
            self.rootNode.setQuat( qNew  )
        if abs( ang ) < maxRot:
            step = diffN*self.walkSpeed*globalClock.getDt()
            if step.lengthSquared() > diff.lengthSquared():
                self.newRandomTarget()
                step = diff
            step = self.rootNode.getQuat().xform( step )
            self.rootNode.setPos( self.rootNode.getPos() + step )

        # Calculate how far we've walked this frame:
        curWalkDist = (prevPos - self.rootNode.getPos()).length()

        curWalkSpeed = curWalkDist/globalClock.getDt()

        update = curWalkDist*0.75
        update += angClamped*0.5
        self.walkCycle.updateTime( update )

        curPos = self.rootNode.getPos()

        #if not self.stepArcLeft and self.stepArcRight:
        #    self.curTargetHeight = self.footTargetLeft.getPos().getZ()
        #elif not self.stepArcRight and self.stepArcLeft:
        #    self.curTargetHeight = self.footTargetRight.getPos().getZ()
        #elif not self.stepArcLeft and not self.stepArcRight:
        footPosL = self.footTargetLeft.getPos()
        #if self.stepArcLeft:
        #    footPosL = self.stepArcLeft.endPos
        footPosR = self.footTargetRight.getPos()
        #if self.stepArcRight:
        #    footPosR = self.stepArcRight.endPos
        self.curTargetHeight = 0.5*(footPosL.getZ() +\
                footPosR.getZ()) + self.rootHeight - self.footHeightOffset.getZ()

        heightAdjustment = self.curTargetHeight - curPos.getZ()
        limit = self.heightAdjustmentSpeed * globalClock.getDt()
        heightAdjustment = min( max( heightAdjustment, -limit), limit )
        self.rootNode.setPos( curPos.getX(), curPos.getY(), curPos.getZ() + heightAdjustment )

        # Rotate torso:
        cycle = math.sin( self.walkCycle.cycleTime/self.walkCycle.cycleDuration*math.pi*2 )
        self.torsoBone.setHpr( -4*cycle, -2, 0 )

        #############################
        # Update arms:

        self.handTargetLeft.setPos( 0, -cycle*min(curWalkSpeed*0.16,0.3)+0.1, 0 )
        self.handTargetRight.setPos( 0, cycle*min(curWalkSpeed*0.16,0.3)+0.1, 0 )

        self.ikChainArmLeft.updateIK()
        self.ikChainArmRight.updateIK()

        #############################
        # Update legs:

        # TODO: Rotate plannedRotation
        #q = Quat()
        #q.setFromAxisAngleRad( self.turnSpeed, LVector3f.unitZ() )
        #self.plannedRotation.setQuat( q )

        # Move planned foot target further forward (longer steps) when character is
        # walking faster:
        curStepDist = 0.1
        if curWalkSpeed > 0:
            curStepDist = self.stepDist + self.walkSpeed*0.2
        p = LVector3f( -self.footOutwards, curStepDist, 0 )
        pw = render.getRelativePoint( self.plannedRotation, p )
        #p.z = self.noise( pw.x, pw.y )*0.05 + 0.05
        self.plannedFootTargetLeft.setPos( p )
        p = LVector3f( self.footOutwards, curStepDist, 0 )
        pw = render.getRelativePoint( self.plannedRotation, p )
        #p.z = self.noise( pw.x, pw.y )*0.05 + 0.05
        self.plannedFootTargetRight.setPos( p )

        # Update the walkcycle to determine if a step needs to be taken:
        #update = curWalkDist*0.1/globalClock.dt

        if self.walkCycle.stepRequired[0]:
            #self.footTargetLeft.setPos( self.plannedFootTargetLeft.getPos( render ) )
            self.walkCycle.step( 0 )    # Tell walk cycle that step has been taken
            #h = min( curWalkSpeed*0.2, 0.3)
            h = 0.05
            targetPos = self.findGroundPos( self.plannedFootTargetLeft.getPos( self.rootNode ) )
            print("targetPos", targetPos)
            print( "footTargetPOs", self.footTargetLeft.getPos() )
            print( "targetPos rel", render.getRelativePoint( self.rootNode, targetPos ) )
            self.stepArcLeft = FootArc( self.footTargetLeft.getPos() - self.footHeightOffset,
                    render.getRelativePoint( self.rootNode, targetPos ), maxStepHeight=h )
            print( "curPos", self.stepArcLeft.getPos() )

        if self.walkCycle.stepRequired[1]:
            #self.footTargetRight.setPos( self.plannedFootTargetRight.getPos( render ) )
            self.walkCycle.step( 1 )    # Tell walk cycle that step has been taken
            #h = min( curWalkSpeed*0.2, 0.3)
            h = 0.05
            targetPos = self.findGroundPos( self.plannedFootTargetRight.getPos( self.rootNode ) )
            self.stepArcRight = FootArc( self.footTargetRight.getPos() - self.footHeightOffset,
                    render.getRelativePoint( self.rootNode, targetPos ), maxStepHeight=h )

        if self.stepArcLeft:
            legMoveDist = self.legMovementSpeed*globalClock.dt
            self.stepArcLeft.update( legMoveDist )
            self.footTargetLeft.setPos( self.stepArcLeft.getPos() + self.footHeightOffset )
            if self.stepArcLeft.done():
                self.stepArcLeft = None

        if self.stepArcRight:
            legMoveDist = self.legMovementSpeed*globalClock.dt
            self.stepArcRight.update( legMoveDist )
            self.footTargetRight.setPos( self.stepArcRight.getPos() + self.footHeightOffset )
            if self.stepArcRight.done():
                self.stepArcRight = None

        self.ikChainLegLeft.updateIK()
        self.ikChainLegRight.updateIK()

        ##################################
        ## Let toes always face horizontally:
        ## Note: Not sure if this works correctly yet!

        toeNode = self.ikActor.getControlNode( "Toes.L" )
        hpr = toeNode.getHpr( self.rootNode )
        toeNode.setHpr( self.rootNode, hpr.getX(), 0, hpr.getZ() )

        toeNode = self.ikActor.getControlNode( "Toes.R" )
        hpr = toeNode.getHpr( self.rootNode )
        toeNode.setHpr( self.rootNode, hpr.getX(), 0, hpr.getZ() )

        return task.cont

    def findGroundPos( self, inputPos ):
    
        self.rayNode.setPos( inputPos + LVector3f.unitZ()*2 )
        self.collisionTraverser.traverse( self.terrain.root )
        if len( self.collisionQueue.getEntries() ) > 0:
            self.collisionQueue.sortEntries()
            groundPos = self.collisionQueue.getEntry(0).getSurfacePoint( self.rootNode )
            return groundPos
        else:
            return inputPos
    
    def newRandomTarget( self ):

        self.targetNode.setPos(
                LVector3f( random.random()*9-4.5,
                    random.random()*9-4.5,
                    0 ) )


    def printChain( self ):
        self.ikChainLegLeft.debhugPrint()


if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CameraControl import CameraControl

    class MyApp(ShowBase):

        def __init__(self):

            #####################################
            ## Set up scene

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

            terrain = CollisionTerrain( 5, 0.25, render, height=1 )

            #####################################
            # Set up character
            self.character = RiggedChar( terrain )
            #self.character2 = RiggedChar( terrain )

            #####################################
            # Set up Camera and input:

            #focusNode = render.attachNewNode( "CameraFocusNode" )
            self.camControl = CameraControl( camera, self.mouseWatcherNode, speed = 0.02 )
            self.camControl.focusPoint = LVector3f( 0, 0, 1 )
            
            self.taskMgr.add( self.camControl.moveCamera, "MoveCameraTask")

            self.accept( "wheel_down", self.camControl.wheelDown )
            self.accept( "wheel_up", self.camControl.wheelUp )

            self.accept( "z", self.toggleAnimation )
            self.animateTarget = True

        def toggleAnimation( self ):
            self.animateTarget = (self.animateTarget == False)


    app = MyApp()
    app.run()
