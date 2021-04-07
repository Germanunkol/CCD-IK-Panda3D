from direct.task import Task

from panda3d.core import LPoint3, LVector3f
from panda3d.core import KeyboardButton, MouseButton
from panda3d.core import Quat
import math
from Utils import *

class CameraControl:

    def __init__( self, node, mouseWatcherNode, speed=0.2 ):

        self.node = node

        self.headingNode = render.attachNewNode( "CameraHeadingRotNode" )
        self.pitchNode = self.headingNode.attachNewNode( "CameraPitchRotNode" )
        self.node.reparentTo( self.pitchNode )

        self.headingNode.setPos( 0, 0, 0.5 )

        #self.focusNode = render.attachNewNode("CameraFocusNode")
        self.attached = False

        self.headingNode.attachNewNode( createAxes( 0.1 ) )

        self.focusPoint = LPoint3()

        self.mouseWatcherNode = mouseWatcherNode

        self.bForward = KeyboardButton.ascii_key('w')
        self.bBackward = KeyboardButton.ascii_key('s')
        self.bLeft = KeyboardButton.ascii_key('a')
        self.bRight = KeyboardButton.ascii_key('d')
        self.bSpeed = KeyboardButton.lshift()

        self.speed = speed

        self.zoom = 10


        self.heading = 0
        self.pitch = -45

        self.node.setPos( 0, -self.zoom, 0 )
        self.pitchNode.setHpr( 0, self.pitch, 0 )
        self.headingNode.setHpr( self.heading, 0, 0 )

        self.lastMousePos = (0,0)

    def attachTo( self, other ):

        self.headingNode.reparentTo( other )
        self.headingNode.setPos( 0, 0, 0 )
        self.attached = True

    def wheelUp( self ):

        self.zoom = self.zoom - 1
        self.zoom = min( max( self.zoom, 1 ), 2000 )

    def wheelDown( self ):

        self.zoom = self.zoom + 1
        self.zoom = min( max( self.zoom, 1 ), 2000 )

    def moveCamera( self, task ):

        is_down = base.mouseWatcherNode.is_button_down

        if base.mouseWatcherNode.hasMouse():
            x = base.mouseWatcherNode.getMouseX()
            y = base.mouseWatcherNode.getMouseY()
            if is_down( MouseButton.two() ):
                dx = self.lastMousePos[0] - x
                self.heading += dx*25
                #self.ang = max( 0, min( self.ang, math.pi*0.49 ) )
                dy = self.lastMousePos[1] - y
                self.pitch -= dy*25
                self.pitch = max( self.pitch, -80 )
                self.pitch = min( self.pitch, 80 )
                #self.angY = max( 0.01, min( self.angY, math.pi ) )
        
            self.lastMousePos = (x,y)

        speed = self.speed
        if is_down( self.bSpeed ):
            speed = speed*3

        if not self.attached:
            forward = (is_down(self.bRight)-is_down(self.bLeft))*speed
            sideways = -(is_down(self.bForward)-is_down(self.bBackward))*speed

            quat = Quat()
            quat.setFromAxisAngle( self.heading, LVector3f.unitZ())
            rotated = quat.xform( LVector3f( -forward, sideways, 0 ))

            self.headingNode.setPos( self.headingNode.getPos() - rotated )

        
        #self.angY = max( 0, min( math.pi*0.4, self.angY ) )
        #rY = math.sin( self.angY )
        #self.nodePos = self.focusPoint + LVector3f( math.sin(self.angY)*math.cos(self.ang)*radius, -math.sin(self.ang)*radius, radius*math.cos( self.angY) )

        self.node.lookAt( self.headingNode )

        self.node.setPos( 0, -self.zoom, 0 )
        self.pitchNode.setHpr( 0, self.pitch, 0 )
        self.headingNode.setHpr( self.heading, 0, 0 )

        return Task.cont


