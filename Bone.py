import random
from panda3d.core import LVector3f
import math

class Bone():

    def __init__( self, joint, parent=None, static=False ):

        self.axis = None
        self.minAng = -math.pi*0.5
        self.maxAng = math.pi*0.5
        self.joint = joint
        self.parent = parent
        self.static = static
        self.controlNode = None
        self.exposedNode = None
        self.ikNode = None
        self.children = []
        if parent:
            self.parent.addChild( self )
    
        self.col = ( 0.5, 0.5, 0.5 )

    def addChild( self, child ):
        if not child in self.children:
            self.children.append( child )

