from panda3d.core import *
from direct.actor.Actor import Actor
import math

class ArmatureUtils():

    def __init__( self ):

        # Set up the components required to create joints:
        self.char = Character("IKChain")
        self.bundle = self.char.get_bundle(0)
        self.skeleton = PartGroup(self.bundle, "<skeleton>")

        self.char_node_path = NodePath( self.char )
        self.actor = None       # Will be initialized after all the joints are created

        self.joints = {}
        self.control_nodes = {}

    def create_joint( self, name, parent_joint=None, rot_mat=None, rot_axis=None, rot_ang_rad=0, translate=None, control_node=None, static=False ):
        """ Utility function which helps to create a joint.
        """

        if name in self.joints.keys():
            raise sys.Exception( f"Cannot create joint with name {name}, already exists!" )

        if rot_axis and rot_mat:
            raise sys.Exception( "Only either rot_mat or rot_axis can be used, not both!" )

        if rot_axis:
            rot_mat = Mat4.rotate_mat( rot_ang_rad/math.pi*180, rot_axis )

        if not rot_mat:
            rot_mat = Mat4.ident_mat()

        if translate:
            translate_mat = Mat4.translate_mat( translate )
        else:
            translate_mat = Mat4.ident_mat()

        transform_mat = rot_mat*translate_mat

        if not parent_joint:
            joint = CharacterJoint( self.char, self.bundle, self.skeleton, name, transform_mat )
        else:
            joint = CharacterJoint( self.char, self.bundle, parent_joint, name, transform_mat )

        self.joints[name] = joint
        
        return joint

    def finalize( self ):

        self.actor = Actor( self.char_node_path )

        for name, joint in self.joints.items():
            control_node = self.actor.control_joint( None, "modelRoot", name )
            self.control_nodes[name] = control_node

    def get_joint( self, name ):
        return self.joints[name]

    def get_control_node( self, name ):
        return self.control_nodes[name]

    def get_actor( self ):
        if not self.actor:
            raise sys.Exception( "Call 'finalize()' before calling 'get_actor()'!" ) 
        return self.actor
