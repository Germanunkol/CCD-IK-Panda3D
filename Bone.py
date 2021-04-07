import random
from panda3d.core import LVector3f
import math

class Bone():

    def __init__( self, joint, parent=None, static=False ):

        self.axis = None
        self.minAng = -math.pi
        self.maxAng = math.pi
        self.joint = joint
        self.parent = parent
        self.static = static
        self.controlNode = None
        self.exposedNode = None
        #self.ikNode = None
        self.children = []
        if parent:
            self.parent.addChild( self )
    
        self.col = ( 0.2, 0.2, 0.5 )

    def addChild( self, child ):
        if not child in self.children:
            self.children.append( child )

