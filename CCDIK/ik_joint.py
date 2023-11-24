import random
from panda3d.core import LVector3f
import math

class IKJoint:

    def __init__( self, joint, parent=None, static=False ):

        self.axis = None
        self.min_ang = -math.pi
        self.max_ang = math.pi
        self.joint = joint
        self.parent = parent
        self.static = static
        self.control_node = None
        self.debug_node = None
        #self.ik_node = None
        self.children = []
        if parent:
            self.parent.add_child( self )
    
        self.col = ( 0.2, 0.2, 0.5 )

    def add_child( self, child ):
        if not child in self.children:
            self.children.append( child )

    def set_static( self, static=True ):
        self.static = static
    def get_statis( self ):
        return self.static

    def set_hinge_constraint( self, axis, min_ang=-math.pi, max_ang=math.pi ):
        self.axis = axis.normalized()
        self.min_ang = min_ang
        self.max_ang = max_ang

    def set_ball_constraint( self, min_ang=-math.pi, max_ang=math.pi ):
        self.axis = None
        self.min_ang = min_ang
        self.max_ang = max_ang

    def get_has_rotation_axis( self ):
        return self.axis is not None
    def get_axis( self ):
        return self.axis

    def get_control_node( self ):
        return self.control_node

    def get_min_ang( self ):
        return self.min_ang
    def get_max_ang( self ):
        return self.max_ang
    def get_col( self ):
        return self.col
    def get_debug_node( self ):
        return self.debug_node

    def get_parent( self ):
        return self.parent
