import math
import random
import numpy as np

from panda3d.core import LMatrix3f, LVector3f, BoundingBox, BoundingVolume, Quat

def distanceToSegment( p, seg ):

  segLen = (seg[1] - seg[0]).length()
  if segLen == 0:
      return (p - seg[0]).length()
  t = ((p.x - seg[0].x)*(seg[1].x -seg[0].x) + (p.y - seg[0].y)*(seg[1].y - seg[0].y) + (p.z - seg[0].z)*(seg[1].z-seg[0].z))/segLen
  t = max( min( t, 1.0 ), 0.0 )
  base = seg[0] + (seg[1]-seg[0])*t
  return (p - base).length()


# Given line 1 (p1 + t1*d1) and line 2 (p2 + t2*d2),
# find the shortest distance between them
def lineLineDistance( p1, d1, p2, d2 ):

    # Check if they are parallel:
    n = d1.cross( d2 )
    if n.length() == 0:
        return (p1-p2).length()

    d = n.dot( p1 - p2 )/n.length()

    return abs(d)

def segSegIntersection( start1, end1, start2, end2 ):

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
        p1,p2,dist = segSegDistance( start1.pos, end1.pos, start2.pos, end2.pos )
        if dist < width1 + width2:
            return True
    return False

# Adapted from https://stackoverflow.com/a/18994296/1936575
def segSegDistance( a0, a1, b0, b1 ):

    ''' Given two line segments return the closest points
    on each segment and their distance
    '''

    # Calculate denomitator
    A = a1 - a0
    B = b1 - b0
    magA = A.length()
    magB = B.length()
    
    _A = A / magA
    _B = B / magB
    
    cross = _A.cross(_B)
    denom = cross.lengthSquared()
    
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
        elif d0 >= magA <= d1:
            if abs(d0) < abs(d1):
                return a1,b0,(a1-b0).length()
            return a1,b1,(a1-b1).length()
            
        # Segments overlap, return distance between parallel segments
        return None,None,(((d0*_A)+a0)-b0).length()
        
    
    
    # Lines criss-cross: Calculate the projected closest points
    t = (b0 - a0);
    detA = LMatrix3f(t, _B, cross).determinant()
    detB = LMatrix3f(t, _A, cross).determinant()

    t0 = detA/denom;
    t1 = detB/denom;

    pA = a0 + (_A * t0) # Projected closest point on segment A
    pB = b0 + (_B * t1) # Projected closest point on segment B


    # Clamp projections
    if t0 < 0:
        pA = a0
    elif t0 > magA:
        pA = a1
    
    if t1 < 0:
        pB = b0
    elif t1 > magB:
        pB = b1
        
    # Clamp projection A
    if (t0 < 0) or (t0 > magA):
        dot = _B.dot(pA-b0)
        if dot < 0:
            dot = 0
        elif dot > magB:
            dot = magB
        pB = b0 + (_B * dot)

    # Clamp projection B
    if (t1 < 0) or (t1 > magB):
        dot = _A.dot(pB-a0)
        if dot < 0:
            dot = 0
        elif dot > magA:
            dot = magA
        pA = a0 + (_A * dot)

    
    return pA,pB,(pA-pB).length()

# Swing-Twist decomposition based on: 
# https://stackoverflow.com/questions/3684269/component-of-a-quaternion-rotation-around-an-axis
#   Decompose the rotation to 2 parts.
#   1. Twist - rotation around the "twistAxis" vector
#   2. Swing - rotation around axis that is perpendicular to "twistAxis" vector
#   The rotation can be composed back by 
#   rotation = swing * twist
#
#   has singularity in case of swing_rotation close to 180 degrees rotation.
#   if the input quaternion is of non-unit length, the outputs are non-unit as well
#   otherwise, outputs are both unit
def swingTwistDecomposition( rotation, twistAxis ):
    ra = LVector3f( rotation.getI(), rotation.getJ(), rotation.getK() )
    p = ra.project( twistAxis ) # return projection v1 onto v2
    twist = Quat( rotation.getR(), p.getX(), p.getY(), p.getZ() )
    twist.normalize()
    swing = rotation * twist.conjugate()
    return swing, twist

def closestPointOnLine(a, b, p):
    ap = p-a
    ab = b-a
    result = a + ab * ap.dot(ab)/ab.dot(ab)
    return result

def closestPointOnSegment( a, b, p ):
    ap = p-a
    ab = b-a
    dist = ap.dot(ab)/ab.dot(ab)
    if dist < 0:
        return a, dist
    if dist > 1:
        return b, dist
    result = a + ab * dist
    return result, dist

def closestPointOnPath( path, p ):
    d2 = np.inf
    closestPoint = None
    reachedPathEnd = False
    for i in range(len(path)-1):
        x, amountOnSegment = closestPointOnSegment( path[i].pos, path[i+1].pos, p )
        #if amountOnSegment >= 0 and amountOnSegment <= 1:
        distSquared = (x-p).lengthSquared()
        if distSquared < d2:
            d2 = distSquared
            closestPoint = x
        if i == len(path)-2:
            if amountOnSegment >= 1:
                reachedPathEnd = True
    if not closestPoint:
        reachedPathEnd = True
    return closestPoint, reachedPathEnd

def getPointOnPath( path, amount ):
    segLengths = []
    for i in range(len(path)-1):
        segLength = (path[i+1].pos - path[i].pos).length()
        segLengths.append( segLength )
    pathLength = sum(segLengths)

    assert pathLength > 0, "Path length must be > 0!"

    walkedAmount = 0
    for i in range(len(path)-1):
        segAmount = segLengths[i]/pathLength
        if walkedAmount + segAmount >= amount:
            restAmount = amount - walkedAmount
            rel = restAmount/segAmount
            return (path[i+1].pos - path[i].pos)*rel + path[i].pos

        walkedAmount += segAmount
    return None

def getPointOnPathDist( path, dist ):
    l = pathLength( path )
    if l <= 0:
        return path[0].pos

    for i in range(len(path)-1):
        segLength = (path[i+1].pos - path[i].pos).length()
        if segLength > dist:
            amount = dist/segLength
            return path[i+1].pos*amount + path[i].pos*(1-amount)
        else:
            dist -= segLength
    return path[-1].pos

def getPointAndNormalOnPathDist( path, dist ):
    l = pathLength( path )
    if l <= 0:
        return path[0].pos, path[0].normal

    for i in range(len(path)-1):
        segLength = (path[i+1].pos - path[i].pos).length()
        if segLength > dist:
            amount = dist/segLength
            pos = path[i+1].pos*amount + path[i].pos*(1-amount)
            normal = path[i+1].normal*amount + path[i].normal*(1-amount)
            return pos, normal
        else:
            dist -= segLength
    return path[-1].pos, path[-1].normal

def pathLength( path ):
    l = 0
    for i in range(len(path)-1):
        segLength = (path[i+1].pos - path[i].pos).length()
        l += segLength
    return l


def uniformRandomDirection( allowZero=False ):

    found = False
    while not found:
        vec = LVector3f( random.random()*2-1, random.random()*2-1, random.random()*2-1 )
        length = vec.length()
        if length <= 1:
            if allowZero or length > 1e-6:
                found = True

    return vec.normalized()

def slerp( q1, q2, t ):

    qDiff = q1.conjugate() * q2
    theta = qDiff.getAngleRad()*0.5
    if abs(theta) < 1e-7:
        return q1

    a3 = math.sin(theta)
    a1 = math.sin((1-t)*theta)
    a2 = math.sin(t*theta)

    return (q1*a1 + q2*a2)/a3

def shortestRotation( a, b ):
    if a.dot(b) < 0:
        b2 = Quat( -b.getR(), -b.getI(), -b.getJ(), -b.getK() )
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
    print( lineLineDistance( p1, d1, p2, d2 ) )



