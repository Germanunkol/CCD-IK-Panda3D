import random

class Bone():

    def __init__( self, offset, axis, minAng, maxAng, joint, parent=None, static=False ):

        self.offset = offset
        print("offset:", offset)
        if axis:
            self.axis = axis.normalized()
        else:
            self.axis = None
        self.minAng = minAng
        self.maxAng = maxAng
        self.targetAng = 0
        self.joint = joint
        self.parent = parent
        self.static = static
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

