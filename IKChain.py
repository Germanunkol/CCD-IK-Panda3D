from panda3d.core import *
from direct.actor.Actor import Actor
import random, math
from Bone import Bone
from Utils import *
from VecUtils import *

class IKChain():

    def __init__( self, parent ):

        # Create a character.
        self.char = Character('IKChain')
        self.bundle = self.char.getBundle(0)
        self.skeleton = PartGroup(self.bundle, '<skeleton>')

        self.root = CharacterJoint(self.char, self.bundle, self.skeleton, 'root',
                      Mat4.identMat())

        self.bones = []
        self.target = None
        self.targetReached = False
        self.parent = parent

    def addBone( self, offset, rotAxis=None, minAng=0, maxAng=0, parentBone=None ):

        name = "joint" + str(len( self.bones ))
 
        if parentBone is None:
            transform = Mat4.identMat()
            joint = CharacterJoint( self.char, self.bundle, self.root, name, transform )
        else:
            transform = Mat4.translateMat(parentBone.offset)
            joint = CharacterJoint( self.char, self.bundle, parentBone.joint, name, transform )

        if rotAxis:
            rotAxis = rotAxis.normalized()
        bone = Bone( offset, rotAxis, minAng, maxAng, joint, parent=parentBone )

        self.bones.append(bone)

        return bone

    def finalize( self ):

        self.charNodePath = NodePath(self.char)
        self.actor = Actor(self.charNodePath)#, {'simplechar' : anim})
        self.actor.reparentTo(self.parent)
        self.rootExposedNode = self.actor.exposeJoint( None, "modelRoot", self.root.getName() )

        # Root of the chain
        parentIKNode = self.rootExposedNode

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
                    parentNode = self.rootExposedNode

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

    def debugDisplay( self ):
    
        for bone in self.bones:

            col = (random.random(), random.random(), random.random())
            #col = (0,0,0)

            lines = LineSegs()
            lines.setThickness(6)
            lines.setColor( col[0], col[1], col[2] )
            lines.moveTo(0, 0, 0)
            #lines.drawTo(np.getPos(parentNode))
            lnp = bone.exposedNode.attachNewNode(lines.create())
            lnp.setBin("fixed", 40)
            #lnp.setDepthWrite(False)
            #lnp.setDepthTest(False)

            lines = LineSegs()
            lines.setThickness(12)
            lines.setColor( col[0], col[1], col[2] )
            lines.moveTo(0, 0, 0)
            lines.drawTo(bone.offset)
            #lines.drawTo(np.getPos(parentNode))
            lnp = bone.exposedNode.attachNewNode(lines.create())
            lnp.setBin("fixed", 40)
            #lnp.setDepthWrite(False)
            #lnp.setDepthTest(False)

            if bone.parent:
                parentNode = bone.parent.exposedNode
            else:
                parentNode = self.rootExposedNode

            lines = LineSegs()
            lines.setColor( 0.6, 0.2, 0.2 )
            #lines.setColor( 0.02, 0.02, 0.02 )
            myPos = bone.exposedNode.getPos( parentNode )
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

            lnp = parentNode.attachNewNode(lines.create())
            lnp.setBin("fixed", 40)
            #lnp.setDepthWrite(False)
            #lnp.setDepthTest(False)

            if bone.axis:
                lines = LineSegs()
                lines.setColor( 0.6, 0.6, 0.6 )
                lines.setThickness(3)
                lines.moveTo( myPos )
                lines.drawTo( myPos + bone.axis.normalized()*0.3 )

                lnp = parentNode.attachNewNode(lines.create())
                lnp.setBin("fixed", 40)
                #lnp.setDepthWrite(False)
                #lnp.setDepthTest(False)
#

