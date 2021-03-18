from panda3d.core import *
from direct.actor.Actor import Actor
import random, math
from Bone import Bone
from Utils import *
from VecUtils import *

class IKChain():

    def __init__( self, parent, char=None, actor=None ):

        self.parent = parent

        if not char:
            # Create a character and set up the skeleton:
            self.char = Character("IKChain")
            self.bundle = self.char.getBundle(0)
            self.skeleton = PartGroup(self.bundle, "<skeleton>")
            self.charNodePath = NodePath(self.char)
            self.charNodePath.reparentTo( parent )

        else:
            self.char = char
            self.bundle = self.char.getBundle(0)
            self.charNodePath = NodePath(self.char)

        # We need an actor to be able to control (and expose) joints. If it already exists,
        # likely because the model was loaded from a file - great then just use that.
        # However, if it doesn't exist, this means we first need to set up all the joints before
        # we create the actor, so keep it empty for now!
        if actor:
            self.actor = actor
        else:
            self.actor = None

        self.bones = []
        self.target = None
        self.targetReached = False

        self.debugDisplayEnabled = False
        self.debugDisplayNodes = []

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

            # Set up rotation axis:
            if axis == None:    # No axis, i.e. the constraint should be a ball joint:
                pass

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
                    joint=joint )
            #parentJoint = joint

        chain.finalize()
        return chain


    def addBone( self, offset=None, rotAxis=None, minAng=0, maxAng=0, parentBone=None, joint=None ):

        if rotAxis:
            rotAxis = rotAxis.normalized()

        if not joint:
            name = "joint" + str(len( self.bones ))

            if rotAxis:
                rot = Mat4.rotateMat( minAng/math.pi*180, rotAxis )
            else:
                rot = Mat4.identMat()
            translate = Mat4.translateMat(offset)
            transform = rot*translate
        
            if parentBone is None:
                joint = CharacterJoint( self.char, self.bundle, self.skeleton, name, transform )
            else:
                joint = CharacterJoint( self.char, self.bundle, parentBone.joint, name, transform )

        bone = Bone( offset, rotAxis, minAng, maxAng, joint, parent=parentBone )

        self.bones.append(bone)

        return bone

    def finalize( self ):

        # Create an actor so we can expose and control nodes:
        # Note: If the model was loaded from a file, the actor already exists
        if not self.actor:
            self.actor = Actor(self.charNodePath)#, {'simplechar' : anim})
            self.actor.reparentTo(self.parent)

        # Root of the chain
        parentIKNode = self.actor

        # For each bone, create:
        # - a control node which will be used to update the bone position after IK solving
        # - an exposed node which we can attach things to, to render them
        # - a normal NodePath node called ikNode, which we'll use during IK solving
        for bone in self.bones:
            name = bone.joint.getName()
            controlNode = self.actor.controlJoint(None, "modelRoot", name )
            bone.controlNode = controlNode
            exposedNode = self.actor.exposeJoint(None, "modelRoot", name )
            bone.exposedNode = exposedNode
            # Separate nodes for IK:
            ikNode = parentIKNode.attachNewNode( name )
            # Same local pos as the exposed joints:
            ikNode.setPos( controlNode.getPos() )
            bone.ikNode = ikNode
            parentIKNode = ikNode

        self.endEffector = self.bones[-1].ikNode.attachNewNode( "EndEffector" )
        #self.endEffector.setPos( self.bones[-1].offset )

    def updateIK( self ):

        # Solve the IK chain for the IK nodes:
        if self.target:
            self.inverseKinematicsCCD()

        # Copy the data from the IK chain to the actual bones.
        # This will end up affecting the actual mesh.
        for bone in self.bones:
            bone.controlNode.setQuat( bone.ikNode.getQuat() )

        if self.debugDisplayEnabled:
            self.removeDebugDisplay()
            self.createDebugDisplay()


    def inverseKinematicsCCD( self, threshold = 1e-2, minIterations=1, maxIterations=10 ):

        self.targetReached = False
        for i in range(maxIterations):

            if i >= minIterations:
                err = (self.target.getPos(render)-self.endEffector.getPos(render)).length()
                if err < threshold:
                    self.targetReached = True
                    break

            for j in range(len(self.bones)-1):
                bone = self.bones[-j-2]

                boneNode = bone.ikNode
                if bone.parent:
                    parentNode = bone.parent.ikNode
                else:
                    parentNode = self.charNodePath

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


    def debugDisplay( self, enable=True ):

        self.debugDisplayEnabled = enable

        if enable:
            self.createDebugDisplay()
        else:
            self.removeDebugDisplay()

    def removeDebugDisplay( self ):
        # Clear previous nodes:
        for n in self.debugDisplayNodes:
            n.removeNode()
        self.debugDisplayNodes = []

    def createDebugDisplay( self ):

        self.removeDebugDisplay()

        xRay = True
        drawConstraints = True
    
        axisGeom = createAxes( 0.1 )

        for bone in self.bones:

            # Draw axis and point at my location (i.e. after applying my transform to my parent):
            axis = bone.exposedNode.attachNewNode( axisGeom )
            point = bone.exposedNode.attachNewNode( createPoint( col=bone.col ) )


            # Retrieve parent space:
            if bone.parent:
                parentNode = bone.parent.exposedNode
            else:
                parentNode = self.charNodePath

            # Draw my offset in parent space
            lines = LineSegs()
            lines.setThickness( 3 )
            lines.setColor( bone.col[0], bone.col[1], bone.col[2], 1 )
            lines.moveTo( 0, 0, 0 )
            myPos = bone.exposedNode.getPos( parentNode )
            lines.drawTo( myPos )
            geom = lines.create()
            n = parentNode.attachNewNode( geom )

            # Draw my constraints:
            # These need to be drawn in parent space (since my rotation is done in parent space)
            if drawConstraints:
                lines = LineSegs()
                lines.setColor( 0.6, 0.2, 0.2 )
                #lines.setColor( 0.02, 0.02, 0.02 )
                if bone.axis:
                    qMin = Quat()
                    qMin.setFromAxisAngleRad( bone.minAng, bone.axis )
                    qMax = Quat()
                    qMax.setFromAxisAngleRad( bone.maxAng, bone.axis )
                    l = bone.offset*0.5
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMin.xform( l ) )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMax.xform( l ) )
                else:
                    qMin = Quat()
                    qMin.setFromAxisAngleRad( bone.minAng, LVector3f.unitY() )
                    qMax = Quat()
                    qMax.setFromAxisAngleRad( bone.maxAng, LVector3f.unitY() )
                    l = bone.offset*0.5
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMin.xform( l ) )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMax.xform( l ) )
                    qMin = Quat()
                    qMin.setFromAxisAngleRad( bone.minAng, LVector3f.unitZ() )
                    qMax = Quat()
                    qMax.setFromAxisAngleRad( bone.maxAng, LVector3f.unitZ() )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMin.xform( l ) )
                    lines.moveTo( myPos )
                    lines.drawTo( myPos + qMax.xform( l ) )

                constraints = parentNode.attachNewNode(lines.create())

            if xRay:
                axis.setBin("fixed", 0)
                axis.setDepthTest(False)
                axis.setDepthWrite(False)
                point.setBin("fixed", 0)
                point.setDepthTest(False)
                point.setDepthWrite(False)
                n.setBin("fixed", 0)
                n.setDepthTest(False)
                n.setDepthWrite(False)
                if drawConstraints:
                    constraints.setBin("fixed", 0)
                    constraints.setDepthTest(False)
                    constraints.setDepthWrite(False)
    
            self.debugDisplayNodes.append( axis )
            self.debugDisplayNodes.append( point )
            self.debugDisplayNodes.append( n )
            if drawConstraints:
                self.debugDisplayNodes.append( constraints )
    
        #axisGeom = createAxes( 0.5, thickness=2 )
        #axisRoot = self.charNodePath.attachNewNode( axisGeom )
        #axisEE = self.endEffector.attachNewNode( axisGeom )
        #if xRay:
        #    axisRoot.setBin("fixed", 0)
        #    axisRoot.setDepthTest(False)
        #    axisRoot.setDepthWrite(False)
        #    axisEE.setBin("fixed", 0)
        #    axisEE.setDepthTest(False)
        #    axisEE.setDepthWrite(False)


