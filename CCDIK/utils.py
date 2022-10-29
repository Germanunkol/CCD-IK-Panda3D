
from panda3d.core import *
import random, math
from direct.gui.OnscreenText import OnscreenText

# Join two indices to make one unique index:
def join_inds( i, j, base=1000 ):
    if i < j:
        return i*base + j
    else:
        return j*base + i

def cycle_index( i, maximum ):
    while i >= maximum:
        i -= maximum
    while i < 0:
        i += maximum
    return i

def next_power_of_two( x ):
    power = int(math.ceil(math.log(x) / math.log(2)))
    return 2**power

def sign( x ):
    if x < 0:
        return -1
    elif x > 0:
        return 1
    else:
        return 0

def create_axes( size, bothways=False, thickness=1 ):

    lines = LineSegs()
    lines.set_thickness( thickness )

    lines.set_color( 1,0.1,0.1,0.1 )
    if bothways:
        lines.move_to( -size, 0, 0 )
    else:
        lines.move_to( 0, 0, 0 )
    lines.draw_to( size, 0, 0 )

    lines.set_color( 0.1,1,0.1,0.1 )
    if bothways:
        lines.move_to( 0, -size, 0 )
    else:
        lines.move_to( 0, 0, 0 )
    lines.draw_to( 0, size, 0 )

    lines.set_color( 0.1,0.1,1,0.1 )
    if bothways:
        lines.move_to( 0, 0, -size )
    else:
        lines.move_to( 0, 0, 0 )
    lines.draw_to( 0, 0, size )

    geom = lines.create()
    return geom

def create_grid( grid_size, square_size ):

    num_squares = int(grid_size/square_size)

    # Render Grid floor:
    lines = LineSegs()
    lines.set_thickness( 0.3 )
    lines.set_color( 0.1,0.1,0.1,0.1 )
    for ix in range( -num_squares, num_squares ):
        if ix == 0:
            continue
        x = ix*square_size
        lines.move_to( x, -grid_size, 0 )
        lines.draw_to( x, grid_size, 0 )

    for iy in range( -num_squares, num_squares ):
        if iy == 0:
            continue
        y = iy*square_size
        lines.move_to( -grid_size, y, 0 )
        lines.draw_to( grid_size, y, 0 )

    geom = lines.create()
    return geom

def create_point( thickness=5, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    lines = LineSegs()
    lines.set_thickness( thickness )
    lines.set_color( *col )
    lines.move_to( 0,0,0 )
    geom = lines.create()
    return geom

def create_segment( p1, p2, thickness=5, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    lines = LineSegs()
    lines.set_thickness( thickness )
    lines.set_color( *col )
    lines.move_to( p1 )
    lines.draw_to( p2 )
    geom = lines.create()
    return geom

def create_racket( thickness=5, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    handle_len = 0.9
    ring_base = 1.1
    ring_width = 0.7
    ring_height = 0.9

    # Create handle:
    lines = LineSegs()
    lines.set_thickness( thickness )
    lines.set_color( *col )
    lines.move_to( 0,0,0 )
    lines.draw_to( 0,0,handle_len )
    # Create ring:
    lines.move_to( 0,0,ring_base )
    for i in range( 16 ):
        ang = (i+1)/16*math.pi*2
        print(ang)
        x = ring_width*0.5*math.sin( ang )
        z = -ring_height*0.5*math.cos( ang ) + ring_height*0.5 + ring_base
        lines.draw_to( x, 0, z )
    ang = math.pi*0.1
    x = ring_width*0.5*math.sin( ang )
    z = -ring_height*0.5*math.cos( ang ) + ring_height*0.5 + ring_base
    lines.move_to( 0,0,handle_len )
    lines.draw_to( x, 0, z )
    ang = -math.pi*0.1
    x = ring_width*0.5*math.sin( ang )
    z = -ring_height*0.5*math.cos( ang ) + ring_height*0.5 + ring_base
    lines.move_to( 0,0,handle_len )
    lines.draw_to( x, 0, z )

    racket = lines.create()

    ## Create net:
    #lines = LineSegs()
    #lines.set_thickness( thickness*0.5 )
    #lines.set_color( 1,1,1 )
    #for i in range( 10 ):
    #    z = ring_base + ring_height*i/10
    #    #x = 
    #    #ang = 
    #    dz = ring_height*i/10 - ring_height*0.5
    #    print("dz",dz)

    #net = lines.create()

    node = NodePath( racket )
    #node.attachTo( net )
    return node


def create_debug_normals( verts, length=0.1, thickness=1, col=None ):

    if col == None:
        col = (random.random(), random.random(), random.random())

    lines = LineSegs()
    lines.set_thickness( thickness )
    lines.set_color( *col )
    for v in verts:
        pos = v[0]
        normal = v[1]
        is_border_vert = v[2]
        if not is_border_vert:
            lines.move_to( pos )
            lines.draw_to( pos + normal*length )
    geom = lines.create()
    return geom

# Translated from Ogre3D's OgreVector.h.
# Originally based on Stan Melax's article in Game Programming Gems
def get_rotation_between( src, dest, fallback_axis=LVector3f.zero() ):
    v0 = src.normalized()
    v1 = dest.normalized()

    q = Quat()

    d = v0.dot( v1 )
    if d >= 1:
        return Quat.ident_quat()
    if d < (1e-6 - 1):
        if fallback_axis != LVector3f.zero():
            # rotate 180 degrees about the fallback axis:
            q.set_from_axis_angle_rad( math.pi, fallback_axis )
        else:
            # generate an axis
            axis = LVector3f.unit_x().cross( v0 )
            if( axis.length_squared() < 1e-9 ):
                axis = LVector3f.unit_y().cross( v0 )
            axis.normalize()
            q.set_from_axis_angle_rad( math.pi, axis )
    else:
        s = math.sqrt( (1+d)*2 )
        invs = 1/s

        c = v0.cross( v1 )

        q.set_i( c.x * invs )
        q.set_j( c.y * invs )
        q.set_k( c.z * invs )
        q.set_r( s * 0.5 )
        q.normalize()
        
    return q

def get_perpendicular_vec( vec ):
    vec = vec.normalized()
    vec2 = LVector3f.unit_y()

    ang = vec.angle_deg( vec2 )
    if ang < 0.1 or ang > 179.9:       # Parallel?
        # Choose a different vector:
        vec2 = LVector3f.unit_x()

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

def x_ray_node( node ):
    node.set_bin("fixed", 0)
    node.set_depth_test(False)
    node.set_depth_write(False)


