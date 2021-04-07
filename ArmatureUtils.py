from panda3d.core import *
from direct.actor.Actor import Actor

class ArmatureUtils():

    def __init__( self ):

        # Set up the components required to create joints:
        self.char = Character("IKChain")
        self.bundle = self.char.getBundle(0)
        self.skeleton = PartGroup(self.bundle, "<skeleton>")

        self.charNodePath = NodePath( self.char )
        self.actor = None       # Will be initialized after all the joints are created

        self.joints = {}
        self.controlNodes = {}

    def createJoint( self, name, parentJoint=None, rotMat=None, rotAxis=None, rotAngRad=0, translate=None, controlNode=None, static=False ):
        """ Utility function which helps to create a joint.
        """

        if name in self.joints.keys():
            raise sys.Exception( f"Cannot create joint with name {name}, already exists!" )

        if rotAxis and rotMat:
            raise sys.Exception( "Only either rotMat or rotAxis can be used, not both!" )

        if rotAxis:
            rotMat = Mat4.rotateMat( minAng/math.pi*180, rotAxis )

        if not rotMat:
            rotMat = Mat4.identMat()

        if translate:
            translateMat = Mat4.translateMat( translate )
        else:
            translateMat = Mat4.identMat()

        transformMat = rotMat*translateMat

        if not parentJoint:
            joint = CharacterJoint( self.char, self.bundle, self.skeleton, name, transformMat )
        else:
            joint = CharacterJoint( self.char, self.bundle, parentJoint, name, transformMat )

        self.joints[name] = joint
        
        return joint

    def finalize( self ):

        self.actor = Actor( self.charNodePath )

        for name, joint in self.joints.items():
            controlNode = self.actor.controlJoint( None, "modelRoot", name )
            self.controlNodes[name] = controlNode

    def getJoint( self, name ):
        return self.joints[name]

    def getControlNode( self, name ):
        return self.controlNodes[name]

    def getActor( self ):
        if not self.actor:
            raise sys.Exception( "Call 'finalize()' before calling 'getActor()'!" ) 
        return self.actor
