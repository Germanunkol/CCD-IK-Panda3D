import random

class Bone():

    def __init__( self, offset, axis, minAng, maxAng, joint, parent=None ):

        self.offset = offset
        self.axis = axis
        if axis:
            self.axis = self.axis.normalized()
        self.minAng = minAng
        self.maxAng = maxAng
        self.targetAng = 0
        self.joint = joint
        self.parent = parent
        self.controlNode = None
        self.exposedNode = None
        self.ikNode = None
        self.children = []
        if parent:
            self.parent.addChild( self )
    
        self.col = (random.random(), random.random(), random.random())

    def addChild( self, child ):
        if not child in self.children:
            self.children.append( child )

