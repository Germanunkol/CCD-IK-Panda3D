from panda3d.core import *

class CollisionTerrain():

    def __init__( self, size, stepSize, parent, height=0.5 ):

        self.noise1 = PerlinNoise2()
        self.noise1.setScale( 2, 2 )
        self.noise2 = PerlinNoise2()
        self.noise2.setScale( 0.2, 0.2 )

        steps = int(size/stepSize)

        self.root = parent.attachNewNode("TerrainNode")
        #self.root.setPos(0,0,-2)
        self.collisionNode = CollisionNode("TerrainCollisionNode")
        for x in range(-steps, steps):
            for y in range(-steps, steps):
                x1 = x*stepSize
                y1 = y*stepSize
                x2 = (x+1)*stepSize
                y2 = (y+1)*stepSize
                p1 = Point3( x1, y1, self.noise( x1, y1 )*height )
                p2 = Point3( x1, y2, self.noise( x1, y2 )*height )
                p3 = Point3( x2, y2, self.noise( x2, y2 )*height )
                p4 = Point3( x2, y1, self.noise( x2, y1 )*height )

                self.collisionNode.addSolid( CollisionPolygon( p3, p2, p1 ) )
                self.collisionNode.addSolid( CollisionPolygon( p4, p3, p1 ) )

        self.colliderNode = self.root.attachNewNode( self.collisionNode )
        self.colliderNode.show()

    def noise( self, x, y ):

        n = self.noise1( x, y )*0.9 + self.noise2( x, y )*0.1 + (y**3)*0.002
        return n


