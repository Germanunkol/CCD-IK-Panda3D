
from panda3d.core import *
import random, math
from direct.gui.OnscreenText import OnscreenText

# Join two indices to make one unique index:
def joinInds( i, j, base=1000 ):
    if i < j:
        return i*base + j
    else:
        return j*base + i

def cycleIndex( i, maximum ):
    while i >= maximum:
        i -= maximum
    while i < 0:
        i += maximum
    return i

def nextPowerOfTwo( x ):
    power = int(math.ceil(math.log(x) / math.log(2)))
    return 2**power

def sign( x ):
    if x < 0:
        return -1
    elif x > 0:
        return 1
    else:
        return 0

def createAxes( size, bothways=False, thickness=1 ):

    lines = LineSegs()
    lines.setThickness( thickness )

    lines.setColor( 1,0.1,0.1,0.1 )
    if bothways:
        lines.moveTo( -size, 0, 0 )
    else:
        lines.moveTo( 0, 0, 0 )
    lines.drawTo( size, 0, 0 )

    lines.setColor( 0.1,1,0.1,0.1 )
    if bothways:
        lines.moveTo( 0, -size, 0 )
    else:
        lines.moveTo( 0, 0, 0 )
    lines.drawTo( 0, size, 0 )

    lines.setColor( 0.1,0.1,1,0.1 )
    if bothways:
        lines.moveTo( 0, 0, -size )
    else:
        lines.moveTo( 0, 0, 0 )
    lines.drawTo( 0, 0, size )

    geom = lines.create()
    return geom

def createGrid( gridSize, squareSize ):

    numSquares = int(gridSize/squareSize)

    # Render Grid floor:
    lines = LineSegs()
    lines.setThickness( 0.3 )
    lines.setColor( 0.1,0.1,0.1,0.1 )
    for ix in range( -numSquares, numSquares ):
        if ix == 0:
            continue
        x = ix*squareSize
        lines.moveTo( x, -gridSize, 0 )
        lines.drawTo( x, gridSize, 0 )

    for iy in range( -numSquares, numSquares ):
        if iy == 0:
            continue
        y = iy*squareSize
        lines.moveTo( -gridSize, y, 0 )
        lines.drawTo( gridSize, y, 0 )

    geom = lines.create()
    return geom

def createPoint( thickness=5, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    lines = LineSegs()
    lines.setThickness( thickness )
    lines.setColor( *col )
    lines.moveTo( 0,0,0 )
    geom = lines.create()
    return geom

def createSegment( p1, p2, thickness=5, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    lines = LineSegs()
    lines.setThickness( thickness )
    lines.setColor( *col )
    lines.moveTo( p1 )
    lines.drawTo( p2 )
    geom = lines.create()
    return geom

def createRacket( thickness=5, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    handleLen = 0.9
    ringBase = 1.1
    ringWidth = 0.7
    ringHeight = 0.9

    # Create handle:
    lines = LineSegs()
    lines.setThickness( thickness )
    lines.setColor( *col )
    lines.moveTo( 0,0,0 )
    lines.drawTo( 0,0,handleLen )
    # Create ring:
    lines.moveTo( 0,0,ringBase )
    for i in range( 16 ):
        ang = (i+1)/16*math.pi*2
        print(ang)
        x = ringWidth*0.5*math.sin( ang )
        z = -ringHeight*0.5*math.cos( ang ) + ringHeight*0.5 + ringBase
        lines.drawTo( x, 0, z )
    ang = math.pi*0.1
    x = ringWidth*0.5*math.sin( ang )
    z = -ringHeight*0.5*math.cos( ang ) + ringHeight*0.5 + ringBase
    lines.moveTo( 0,0,handleLen )
    lines.drawTo( x, 0, z )
    ang = -math.pi*0.1
    x = ringWidth*0.5*math.sin( ang )
    z = -ringHeight*0.5*math.cos( ang ) + ringHeight*0.5 + ringBase
    lines.moveTo( 0,0,handleLen )
    lines.drawTo( x, 0, z )

    racket = lines.create()

    ## Create net:
    #lines = LineSegs()
    #lines.setThickness( thickness*0.5 )
    #lines.setColor( 1,1,1 )
    #for i in range( 10 ):
    #    z = ringBase + ringHeight*i/10
    #    #x = 
    #    #ang = 
    #    dz = ringHeight*i/10 - ringHeight*0.5
    #    print("dz",dz)

    #net = lines.create()

    node = NodePath( racket )
    #node.attachTo( net )
    return node


def createDebugNormals( verts, length=0.1, thickness=1, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    lines = LineSegs()
    lines.setThickness( thickness )
    lines.setColor( *col )
    for v in verts:
        pos = v[0]
        normal = v[1]
        isBorderVert = v[2]
        if not isBorderVert:
            lines.moveTo( pos )
            lines.drawTo( pos + normal*length )
    geom = lines.create()
    return geom

# Translated from Ogre3D's OgreVector.h.
# Originally based on Stan Melax's article in Game Programming Gems
def getRotationBetween( src, dest, fallbackAxis=LVector3f.zero() ):
    v0 = src.normalized()
    v1 = dest.normalized()

    q = Quat()

    d = v0.dot( v1 )
    if d >= 1:
        return Quat.identQuat()
    if d < (1e-6 - 1):
        if fallbackAxis != LVector3f.zero():
            # rotate 180 degrees about the fallback axis:
            q.setFromAxisAngleRad( math.pi, fallbackAxis )
        else:
            # generate an axis
            axis = LVector3f.unitX().cross( v0 )
            if( axis.lengthSquared() < 1e-9 ):
                axis = LVector3f.unitY().cross( v0 )
            axis.normalize()
            q.setFromAxisAngleRad( math.pi, axis )
    else:
        s = math.sqrt( (1+d)*2 )
        invs = 1/s

        c = v0.cross( v1 )

        q.setI( c.x * invs )
        q.setJ( c.y * invs )
        q.setK( c.z * invs )
        q.setR( s * 0.5 )
        q.normalize()
        
    return q

def getPerpendicularVec( vec ):
    vec = vec.normalized()
    vec2 = LVector3f.unitY()

    ang = vec.angleDeg( vec2 )
    if ang < 0.1 or ang > 179.9:       # Parallel?
        # Choose a different vector:
        vec2 = LVector3f.unitX()

    return vec.cross(vec2).normalized()


# Macro-like function used to reduce the amount to code needed to create the
# on screen instructions. Shamelessly stolen from the Panda3D samples.
def label( text, line ):
    lbl = OnscreenText( text=text,
            parent=base.a2dTopLeft,
            scale=.05,
            pos=(0.06, -.06 * line - 0.03),
            fg=(1, 1, 1, 1),
            align=TextNode.ALeft)

    return lbl


