from panda3d.core import *
from direct.actor.Actor import Actor
import random, math
from Bone import Bone
from Utils import *
from VecUtils import *

class IKChain():

    def __init__( self, actor=None ):

        # We need an actor to be able to control (and expose) joints. If it already exists,
        # likely because the model was loaded from a file - great then just use that.
        # However, if it doesn't exist, this means we first need to set up all the joints before
        # we create the actor, so keep it empty for now!
        self.actor = actor

        self.bones = []
        self.target = None
        self.targetReached = False

        self.debugDisplayEnabled = False

        self.endEffector = None

    def fromArmature( character, parent, actor, jointList ):

        chain = IKChain( parent, char=character, actor=actor )

        bone = None
        #parentJoint = chain.skeleton
        jointNames = [j["name"] for j in jointList]
        for j in jointList:

            # Get the name of the current bone:
            jointName = j["name"]

            # Get the type of constraint
            axis = "auto"
            if "axis" in j.keys():
                axis = j["axis"]
            # Get the constraint values:
            minAng = -math.pi
            if "minAng" in j.keys():
                minAng = j["minAng"]
            maxAng = -math.pi
            if "maxAng" in j.keys():
                maxAng = j["maxAng"]

            joint = character.findJoint( jointName )
            if not joint:
                raise Exception("Could not find joint with name " + jointName + " in character!")
       
            # Retrieve the transform of the joint. We'll treat this as the "base" pose of the
            # transform and store it for later use.
            mat = joint.getTransform()

            # Get the translation:
            t = mat.getRow3(3)

            static = False

            # Set up rotation axis:
            if axis == None:    # No axis, i.e. the constraint should be a ball joint:
                pass
            elif axis == "static":
                static=True
                axis=None

            elif isinstance( axis, LVector3f ): # Axis was given, use it:
                if axis.length() <  1e-9:
                    raise Exception("Axis given for joint " + jointName + " has length zero")
                axis = axis.normalized()


            elif axis == "auto":# Ask the system to automatically determine rotation axis:

                # Try to get the rotation from the joint transform:
                rot = Quat()
                rot.setFromMatrix( mat )
                rot.normalize()
                axis = rot.getAxisNormalized()
                ang = rot.getAngleRad()

                 # If the angle is zero, then this axis cannot be trusted. 
                if abs(ang) < 1e-8:
                    raise Exception("Cannot automatically determine rotation axis for joint " + jointName + " (its angle is zero)! Please specify a rotation axis manually, or set it to 'None'.")

            else:
                raise Exception("Axis or joint " + jointName + " invalid (must be None, 'auto', or a LVector3f)")

            bone = chain.addBone( t, rotAxis=axis, minAng=minAng, maxAng=maxAng,
                    parentBone=bone,
                    joint=joint,
                    static=static )

        chain.finalize()
        return chain

    #def addBone( self, offset=None, rotAxis=None, minAng=0, maxAng=0, parentBone=None, joint=None, static=False ):
    def addBone( self, joint, controlNode, parentBone=None, static=False ):

        if parentBone:
            parentIKNode = parentBone.controlNode
        else:
            parentIKNode = self.actor
        
        name = joint.getName()

        bone = Bone( joint, parent=parentBone, static=static )

        #ikNode = parentIKNode.attachNewNode( name )
        # Same local pos as the exposed joints:
        #ikNode.setPos( controlNode.getPos() )

        #bone.ikNode = ikNode
        bone.controlNode = controlNode

        self.bones.append(bone)

        return bone

    def getBone( self, jointName ):
        for b in self.bones:
            if b.joint.getName() == jointName:
                return b
        raise Exception(f"Cannot find joint {jointName}!")

    def setStatic( self, jointName, static=True ):
        b = self.getBone( jointName )
        b.static = static

    def setHingeConstraint( self, jointName, axis, minAng=-math.pi, maxAng=math.pi ):
        b = self.getBone( jointName )
        b.axis = axis.normalized()
        b.minAng = minAng
        b.maxAng = maxAng

        if self.debugDisplayEnabled:
            self.debugDisplay()

    #def finalize( self ):

    #    # Create an actor so we can expose and control nodes:
    #    # Note: If the model was loaded from a file, the actor already exists

    #    # Root of the chain
    #    parentIKNode = self.actor

    #    # For each bone, create:
    #    # - a control node which will be used to update the bone position after IK solving
    #    # - an exposed node which we can attach things to, to render them
    #    # - a normal NodePath node called ikNode, which we'll use during IK solving
    #    for bone in self.bones:
    #        name = bone.joint.getName()
    #        # Separate nodes for IK:
    #        ikNode = parentIKNode.attachNewNode( name )
    #        # Same local pos as the exposed joints:
    #        ikNode.setPos( controlNode.getPos() )
    #        bone.ikNode = ikNode
    #        parentIKNode = ikNode

    #    self.endEffector = self.bones[-1].ikNode.attachNewNode( "EndEffector" )
    #    #self.endEffector.setPos( self.bones[-1].offset )

    def updateIK( self, threshold = 1e-2, minIterations=1, maxIterations=10 ):

        # Solve the IK chain for the IK nodes:
        if self.target:
            self.inverseKinematicsCCD( threshold, minIterations, maxIterations )

        # Copy the data from the IK chain to the actual bones.
        # This will end up affecting the actual mesh.
        for bone in self.bones:
            bone.controlNode.setQuat( bone.controlNode.getQuat() )


    def inverseKinematicsCCD( self, threshold = 1e-2, minIterations=1, maxIterations=10 ):

        if not self.endEffector:
            self.endEffector = self.bones[-1].controlNode.attachNewNode( "EndEffector" )

        self.targetReached = False
        for i in range(maxIterations):

            if i >= minIterations:
                err = (self.target.getPos(render)-self.endEffector.getPos(render)).length()
                if err < threshold:
                    self.targetReached = True
                    break

            for j in range(len(self.bones)-1):
                bone = self.bones[-j-2]

                if bone.static:
                    continue

                boneNode = bone.controlNode
                if bone.parent:
                    parentNode = bone.parent.controlNode
                else:
                    parentNode = self.actor

                target = self.target.getPos( boneNode )

                pos = LPoint3.zero()
                ee = self.endEffector.getPos( boneNode )

                d1 = target-pos
                d2 = ee-pos

                cross = d1.cross(d2).normalized()
                if cross.lengthSquared() < 1e-9:
                    continue

                ang = d2.normalized().signedAngleRad( d1.normalized(), cross )
                q = Quat()
                q.setFromAxisAngleRad( ang, cross )
                # Add this rotation to the current rotation:
                qOld = boneNode.getQuat()
                qNew = q*qOld
                qNew.normalize()
                #boneNode.setQuat( qNew )

                # Correct rotation for hinge:
                if bone.axis:
                    #qInv = boneNode.getQuat()
                    #qInv.invertInPlace()
                    #myAxisInParentSpace = qInv.xform( bone.axis )
                    myAxisInParentSpace = bone.axis
                    swing, twist = swingTwistDecomposition( qNew, -myAxisInParentSpace )
                    qNew = twist

                rotAxis = qNew.getAxis()
                rotAxis.normalize()
                ang = qNew.getAngleRad()
                if rotAxis.lengthSquared() > 1e-3 and not math.isnan(ang) and abs(ang) > 0: # valid rotation axis?
                    # reduce the angle  
                    ang = ang % (math.pi*2)
                    # force into the minimum absolute value residue class, so that -180 < angle <= 180  
                    if ang > math.pi:
                        ang -= 2*math.pi

                    if abs(ang) > 1e-6 and abs(ang) < math.pi*2:
                        if bone.axis and (rotAxis - bone.axis).lengthSquared() > 0.5:
                            # Clamp the rotation value:
                            ang = max( -bone.maxAng, min( -bone.minAng, ang ) )
                        else:
                            # Clamp the rotation value:
                            ang = max( bone.minAng, min( bone.maxAng, ang ) )
                            #ang = -ang
                            #rotAxis = -rotAxis

                    #annealing = (j+1)/len(self.bones)
                    #print("annealing", annealing)
                    #q = qOld + (qNew-qOld)*annealing

                    qNew.setFromAxisAngleRad( ang, rotAxis )

                    boneNode.setQuat( qNew )


    def setTarget( self, node ):
        self.target = node

    def debugDisplay( self, lineLength=0.2, xRay=True, drawConstraints=True ):

        self.removeDebugDisplay()

        self.debugDisplayEnabled = True

        axesGeom = createAxes( lineLength )

        for i in range(len(self.bones)):
            bone = self.bones[i]

            # Attach a new node to the ikNode. All debug info will be attached to this node.
            # This is only for cleaner removal of the node later on - by remuving the debug node,
            # all debug info will be cleared. Otherwise this node has no significance and we could
            # just as well attach everything to the ikNode itself
            bone.debugNode = bone.controlNode.attachNewNode("DebugDisplay")
            bone.debugNode.setLightOff(1)

            # Draw axes at my location and rotation (i.e. after applying my transform to my parent):
            axes = bone.debugNode.attachNewNode( axesGeom )
            #point = bone.ikNode.attachNewNode( createPoint( col=bone.col ) )

            # Retrieve parent space:
            if bone.parent:
                # If we have a parent, then this parent is a bone.
                parentNode = bone.parent.controlNode
            else:
                # Otherwise the parent is the actor itself, i.e. the root of the skeleton
                parentNode = self.actor

            # Again, use the parent's debug node rather than attaching stuff to the ikNode directly,
            # so we can remove the debug info easily later on by removing the debug node.
            parentDebugNode = parentNode.find("DebugDisplay")
            if not parentDebugNode:
                parentDebugNode = parentNode.attachNewNode("DebugDisplay")
                parentDebugNode.setLightOff(1)

            # Draw my offset in parent space
            lines = LineSegs()
            lines.setThickness( 3 )
            lines.setColor( bone.col[0], bone.col[1], bone.col[2], 1 )
            lines.moveTo( 0, 0, 0 )
            myPos = bone.controlNode.getPos( parentNode )
            lines.drawTo( myPos )
            geom = lines.create()
            parentDebugNode.attachNewNode( geom )

            # Draw my constraints:
            # These need to be drawn in parent space (since my rotation is done in parent space)
            if drawConstraints:
                if bone.axis:
                    l = getPerpendicularVec( bone.axis )*lineLength

                    lines = LineSegs()
                    lines.setColor( 0.6, 0.3, 0.3 )
                    lines.setThickness( 5 )
                    lines.moveTo( 0,0,0 )
                    lines.drawTo( l )
                    bone.debugNode.attachNewNode(lines.create())
        
                    lines = LineSegs()
                    lines.setColor( 0.8, 0.1, 0.2 )
                    lines.setThickness( 3 )
                    qMin = Quat()
                    qMin.setFromAxisAngleRad( bone.minAng, bone.axis )
                    qMax = Quat()
                    qMax.setFromAxisAngleRad( bone.maxAng, bone.axis )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMin.xform( l ) )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMax.xform( l ) )
                    parentDebugNode.attachNewNode(lines.create())

                    # Draw arc:
                    lines = LineSegs()
                    lines.setColor( 0.6, 0.3, 0.3 )
                    lines.setThickness( 2 )
                    lines.moveTo( myPos + qMin.xform( l*0.9 ) )
                    ang = bone.minAng
                    while ang < bone.maxAng:
                        ang += math.pi*0.1
                        if ang > bone.maxAng:
                            ang = bone.maxAng
                        q = Quat()
                        q.setFromAxisAngleRad( ang, bone.axis )
                        lines.drawTo( myPos + q.xform( l*0.9 ) )
                    parentDebugNode.attachNewNode(lines.create())


                if bone.axis:
                    lines = LineSegs()
                    lines.setColor( 0.8, 0.8, 0.8 )
                    lines.setThickness( 4 )
                    myPos = bone.controlNode.getPos( parentNode )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + bone.axis*0.1 )
                    geom = lines.create()
                    constraintsAxis = parentNode.attachNewNode( geom )
                    print("drawing axis", parentNode)

            if xRay:
                bone.debugNode.setBin("fixed", 0)
                bone.debugNode.setDepthTest(False)
                bone.debugNode.setDepthWrite(False)
                parentDebugNode.setBin("fixed", 0)
                parentDebugNode.setDepthTest(False)
                parentDebugNode.setDepthWrite(False)

    def removeDebugDisplay( self ):
        for i in range(len(self.bones)):
            bone = self.bones[i]
            if hasattr( bone, "debugNode" ):
                bone.debugNode.removeNode()
                bone.debugNode = None

        rootDebugNode = self.actor.find("DebugDisplay")
        if rootDebugNode:
            rootDebugNode.removeNode()


