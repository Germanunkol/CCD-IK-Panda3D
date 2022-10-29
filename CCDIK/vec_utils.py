import math
import random
import numpy as np

from panda3d.core import LMatrix3f, LVector3f, BoundingBox, BoundingVolume, Quat

def distance_to_segment( p, seg ):

  seg_len = (seg[1] - seg[0]).length()
  if seg_len == 0:
      return (p - seg[0]).length()
  t = ((p.x - seg[0].x)*(seg[1].x -seg[0].x) + (p.y - seg[0].y)*(seg[1].y - seg[0].y) + (p.z - seg[0].z)*(seg[1].z-seg[0].z))/seg_len
  t = max( min( t, 1.0 ), 0.0 )
  base = seg[0] + (seg[1]-seg[0])*t
  return (p - base).length()


# Given line 1 (p1 + t1*d1) and line 2 (p2 + t2*d2),
# find the shortest distance between them
def line_line_distance( p1, d1, p2, d2 ):

    # Check if they are parallel:
    n = d1.cross( d2 )
    if n.length() == 0:
        return (p1-p2).length()

    d = n.dot( p1 - p2 )/n.length()

    return abs(d)

def seg_seg_intersection( start1, end1, start2, end2 ):

    width1 = max( start1.size, end1.size )
    width2 = max( start2.size, end2.size )
  
    min1 = LVector3f(
            min( start1.pos.x, end1.pos.x ) - width1,
            min( start1.pos.y, end1.pos.y ) - width1,
            min( start1.pos.z, end1.pos.z ) - width1 )
    max1 = LVector3f(
            max( start1.pos.x, end1.pos.x ) + width1,
            max( start1.pos.y, end1.pos.y ) + width1,
            max( start1.pos.z, end1.pos.z ) + width1 )
    
    min2 = LVector3f(
            min( start2.pos.x, end2.pos.x ) - width2,
            min( start2.pos.y, end2.pos.y ) - width2,
            min( start2.pos.z, end2.pos.z ) - width2 )
    max2 = LVector3f(
            max( start2.pos.x, end2.pos.x ) + width2,
            max( start2.pos.y, end2.pos.y ) + width2,
            max( start2.pos.z, end2.pos.z ) + width2 )


    # If the bounding boxes overlap, also check if the lines cross:
    contains = bb1.contains(bb2)
    if contains == BoundingVolume.IF_possible:
        p1,p2,dist = seg_seg_distance( start1.pos, end1.pos, start2.pos, end2.pos )
        if dist < width1 + width2:
            return True
    return False

# Adapted from https://stackoverflow.com/a/18994296/1936575
def seg_seg_distance( a0, a1, b0, b1 ):

    ''' Given two line segments return the closest points
    on each segment and their distance
    '''

    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    mag_a = A.length()
    mag_b = B.length()
    
    _A = A / mag_a
    _B = B / mag_b
    
    cross = _A.cross(_B)
    denom = cross.length_squared()
    
    # If lines are parallel (denom=0) test if lines overlap.
    # If they don't overlap then there is a closest point solution.
    # If they do overlap, there are infinite closest positions, but there is a closest distance
    if not denom:
        d0 = _A.dot(b0-a0)
        
        d1 = _A.dot(b1-a0)
            
        # Is segment B before A?
        if d0 <= 0 >= d1:
            if abs(d0) < abs(d1):
                return a0,b0,(a0-b0).length()
            return a0,b1,(a0-b1).length()
                
                
        # Is segment B after A?
        elif d0 >= mag_a <= d1:
            if abs(d0) < abs(d1):
                return a1,b0,(a1-b0).length()
            return a1,b1,(a1-b1).length()
            
        # Segments overlap, return distance between parallel segments
        return None,None,(((d0*_A)+a0)-b0).length()
        
    
    
    # Lines criss-cross: Calculate the projected closest points
    t = (b0 - a0);
    det_a = LMatrix3f(t, _B, cross).determinant()
    det_b = LMatrix3f(t, _A, cross).determinant()

    t0 = det_a/denom;
    t1 = det_b/denom;

    p_a = a0 + (_A * t0) # Projected closest point on segment A
    p_b = b0 + (_B * t1) # Projected closest point on segment B


    # Clamp projections
    if t0 < 0:
        p_a = a0
    elif t0 > mag_a:
        p_a = a1
    
    if t1 < 0:
        p_b = b0
    elif t1 > mag_b:
        p_b = b1
        
    # Clamp projection A
    if (t0 < 0) or (t0 > mag_a):
        dot = _B.dot(p_a-b0)
        if dot < 0:
            dot = 0
        elif dot > mag_b:
            dot = mag_b
        p_b = b0 + (_B * dot)

    # Clamp projection B
    if (t1 < 0) or (t1 > mag_b):
        dot = _A.dot(p_b-a0)
        if dot < 0:
            dot = 0
        elif dot > mag_a:
            dot = mag_a
        p_a = a0 + (_A * dot)

    
    return p_a,p_b,(p_a-p_b).length()

# Swing-Twist decomposition based on: 
# https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
#   Decompose the rotation to 2 parts.
#   1. Twist - rotation around the "twist_axis" vector
#   2. Swing - rotation around axis that is perpendicular to "twist_axis" vector
#   The rotation can be composed back by 
#   rotation = swing * twist
#
#   has singularity in case of swing_rotation close to 180 degrees rotation.
#   if the input quaternion is of non-unit length, the outputs are non-unit as well
#   otherwise, outputs are both unit
def swing_twist_decomposition( rotation, twist_axis ):
    ra = LVector3f( rotation.get_i(), rotation.get_j(), rotation.get_k() )
    p = ra.project( twist_axis ) # return projection v1 onto v2
    twist = Quat( rotation.get_r(), p.get_x(), p.get_y(), p.get_z() )
    twist.normalize()
    swing = rotation * twist.conjugate()
    return swing, twist

def closest_point_on_line(a, b, p):
    ap = p-a
    ab = b-a
    result = a + ab * ap.dot(ab)/ab.dot(ab)
    return result

def closest_point_on_segment( a, b, p ):
    ap = p-a
    ab = b-a
    dist = ap.dot(ab)/ab.dot(ab)
    if dist < 0:
        return a, dist
    if dist > 1:
        return b, dist
    result = a + ab * dist
    return result, dist

def closest_point_on_path( path, p ):
    d2 = np.inf
    closest_point = None
    reached_path_end = False
    for i in range(len(path)-1):
        x, amount_on_segment = closest_point_on_segment( path[i].pos, path[i+1].pos, p )
        #if amount_on_segment >= 0 and amount_on_segment <= 1:
        dist_squared = (x-p).length_squared()
        if dist_squared < d2:
            d2 = dist_squared
            closest_point = x
        if i == len(path)-2:
            if amount_on_segment >= 1:
                reached_path_end = True
    if not closest_point:
        reached_path_end = True
    return closest_point, reached_path_end

def get_point_on_path( path, amount ):
    seg_lengths = []
    for i in range(len(path)-1):
        seg_length = (path[i+1].pos - path[i].pos).length()
        seg_lengths.append( seg_length )
    path_length = sum(seg_lengths)

    assert path_length > 0, "Path length must be > 0!"

    walked_amount = 0
    for i in range(len(path)-1):
        seg_amount = seg_lengths[i]/path_length
        if walked_amount + seg_amount >= amount:
            rest_amount = amount - walked_amount
            rel = rest_amount/seg_amount
            return (path[i+1].pos - path[i].pos)*rel + path[i].pos

        walked_amount += seg_amount
    return None

def get_point_on_path_dist( path, dist ):
    l = path_length( path )
    if l <= 0:
        return path[0].pos

    for i in range(len(path)-1):
        seg_length = (path[i+1].pos - path[i].pos).length()
        if seg_length > dist:
            amount = dist/seg_length
            return path[i+1].pos*amount + path[i].pos*(1-amount)
        else:
            dist -= seg_length
    return path[-1].pos

def get_point_and_normal_on_path_dist( path, dist ):
    l = path_length( path )
    if l <= 0:
        return path[0].pos, path[0].normal

    for i in range(len(path)-1):
        seg_length = (path[i+1].pos - path[i].pos).length()
        if seg_length > dist:
            amount = dist/seg_length
            pos = path[i+1].pos*amount + path[i].pos*(1-amount)
            normal = path[i+1].normal*amount + path[i].normal*(1-amount)
            return pos, normal
        else:
            dist -= seg_length
    return path[-1].pos, path[-1].normal

def path_length( path ):
    l = 0
    for i in range(len(path)-1):
        seg_length = (path[i+1].pos - path[i].pos).length()
        l += seg_length
    return l


def uniform_random_direction( allow_zero=False ):

    found = False
    while not found:
        vec = LVector3f( random.random()*2-1, random.random()*2-1, random.random()*2-1 )
        length = vec.length()
        if length <= 1:
            if allow_zero or length > 1e-6:
                found = True

    return vec.normalized()

def slerp( q1, q2, t ):

    q_diff = q1.conjugate() * q2
    theta = q_diff.get_angle_rad()*0.5
    if abs(theta) < 1e-7:
        return q1

    a3 = math.sin(theta)
    a1 = math.sin((1-t)*theta)
    a2 = math.sin(t*theta)

    return (q1*a1 + q2*a2)/a3

def shortest_rotation( a, b ):
    if a.dot(b) < 0:
        b2 = Quat( -b.get_r(), -b.get_i(), -b.get_j(), -b.get_k() )
        q = a * b2.conjugate()
    else:
        q = a * b.conjugate()
    q.normalize()
    return q

if __name__ == "__main__":

    from panda3d.core import LVector3f
    p1 = LVector3f( 0, 0, 0 )
    d1 = LVector3f( 1, 0, 0 )
    p2 = LVector3f( 0, 0, 1 )
    d2 = LVector3f( 0, 1, 0 )
    print( line_line_distance( p1, d1, p2, d2 ) )



