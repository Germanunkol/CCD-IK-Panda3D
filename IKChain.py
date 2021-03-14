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
            #self.root = CharacterJoint(self.char, self.bundle, self.skeleton, rootName,
            #        Mat4.identMat())
        else:
            self.char = char
            self.bundle = self.char.getBundle(0)
            self.skeleton = NodePath(self.char).find("<skeleton>")
            #self.root = self.char.findJoint( rootName )
            #if not self.root:
            #    raise Exception( "Joint '" + rootName + "' not found in character!" )

        self.actor = actor
        self.charNodePath = NodePath(self.char)

        self.bones = []
        self.target = None
        self.targetReached = False

        self.debugDisplayEnabled = False
        self.debugDisplayNodes = []

    def fromArmature( character, parent, actor, jointNameList):

        chain = IKChain( parent, char=character, actor=actor )

        bone = None
        parentJoint = chain.skeleton
        for jointName in jointNameList:
            joint = character.findJoint( jointName )
            if not joint:
                raise Exception("Could not find joint with name " + jointName + " in character!")
       
            # The "offset" is the translation of the parent joint:
            parentMat = parentJoint.getValue()
            t = parentMat.getRow3(3)

            # The rotation is determined by this joint:
            mat = joint.getValue()
            rot = Quat()
            rot.setFromMatrix( mat )
            rot.normalize()
            axis = rot.getAxisNormalized()
            ang = rot.getAngleRad()
            bone = chain.addBone( t, rotAxis=axis, minAng=ang-0.6*math.pi, maxAng=ang+0.6*math.pi,
                    parentBone=bone,
                    joint=joint )
            parentJoint = joint

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
            mat = Mat4()
            joint.getNetTransform(mat)


        bone = Bone( offset, rotAxis, minAng, maxAng, joint, parent=parentBone )

        self.bones.append(bone)

        return bone

    def finalize( self ):

        if not self.actor:
            # Create an actor so we can expose nodes:
            self.actor = Actor(self.charNodePath)#, {'simplechar' : anim})
            self.actor.reparentTo(self.parent)
        
        #self.rootExposedNode = self.actor.exposeJoint( None, "modelRoot", self.root.getName() )

        # Root of the chain
        parentIKNode = self.charNodePath

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
        self.endEffector.setPos( self.bones[-1].offset )

    def updateIK( self ):

        # Solve the IK chain for the IK nodes:
        if self.target:
            self.inverseKinematicsCCD()

        # Copy the data from the IK chain to the actual bones.
        # This will end up affecting the actual mesh.
        for bone in self.bones:
            bone.controlNode.setQuat( bone.ikNode.getQuat() )


    def inverseKinematicsCCD( self, threshold = 1e-2, minIterations=1, maxIterations=10 ):

        self.targetReached = False
        for i in range(maxIterations):

            if i >= minIterations:
                err = (self.target.getPos(render)-self.endEffector.getPos(render)).lengthSquared()
                if err < threshold:
                    self.targetReached = True
                    break

            for j in range(len(self.bones)):
                bone = self.bones[-j-1]

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
    
        axisGeom = createAxes( 0.07, thickness=3 )

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
            lines = LineSegs()
            lines.setColor( 0.6, 0.2, 0.2 )
            #lines.setColor( 0.02, 0.02, 0.02 )
            if bone.axis:
                qMin = Quat()
                qMin.setFromAxisAngleRad( bone.minAng, bone.axis )
                qMax = Quat()
                qMax.setFromAxisAngleRad( bone.maxAng, bone.axis )
                l = bone.offset.normalized()*0.3
                lines.moveTo( myPos )
                lines.drawTo( myPos + qMin.xform( l ) )
                lines.moveTo( myPos )
                lines.drawTo( myPos + qMax.xform( l ) )
            else:
                qMin = Quat()
                qMin.setFromAxisAngleRad( bone.minAng, LVector3f.unitY() )
                qMax = Quat()
                qMax.setFromAxisAngleRad( bone.maxAng, LVector3f.unitY() )
                l = bone.offset.normalized()*0.3
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
                constraints.setBin("fixed", 0)
                constraints.setDepthTest(False)
                constraints.setDepthWrite(False)
    
            self.debugDisplayNodes.append( axis )
            self.debugDisplayNodes.append( point )
            self.debugDisplayNodes.append( n )
            self.debugDisplayNodes.append( constraints )
    
        axisGeom = createAxes( 1 )
        axisRoot = self.charNodePath.attachNewNode( axisGeom )
        axisEE = self.endEffector.attachNewNode( axisGeom )
        if xRay:
            axisRoot.setBin("fixed", 0)
            axisRoot.setDepthTest(False)
            axisRoot.setDepthWrite(False)
            axisEE.setBin("fixed", 0)
            axisEE.setDepthTest(False)
            axisEE.setDepthWrite(False)


