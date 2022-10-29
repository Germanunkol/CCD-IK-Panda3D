import random
from panda3d.core import LVector3f
import math

class Bone():

    def __init__( self, joint, parent=None, static=False ):

        self.axis = None
        self.min_ang = -math.pi
        self.max_ang = math.pi
        self.joint = joint
        self.parent = parent
        self.static = static
        self.control_node = None
        self.exposed_node = None
        #self.ik_node = None
        self.children = []
        if parent:
            self.parent.add_child( self )
    
        self.col = ( 0.2, 0.2, 0.5 )

    def add_child( self, child ):
        if not child in self.children:
            self.children.append( child )

