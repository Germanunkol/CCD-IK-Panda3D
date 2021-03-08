from direct.task import Task

from panda3d.core import LPoint3, LVector3f
from panda3d.core import KeyboardButton, MouseButton
from panda3d.core import Quat
import math

class CameraControl:

    def __init__( self, node, mouseWatcherNode ):

        self.node = node
        self.focusNode = render.attachNewNode("CameraFocusNode")
        self.attachmentNode = None
        self.attached = True

        self.focusPoint = LPoint3()

        self.mouseWatcherNode = mouseWatcherNode

        self.bForward = KeyboardButton.ascii_key('w')
        self.bBackward = KeyboardButton.ascii_key('s')
        self.bLeft = KeyboardButton.ascii_key('a')
        self.bRight = KeyboardButton.ascii_key('d')
        self.bSpeed = KeyboardButton.lshift()

        self.speed = 0.2

        self.zoom = 10

        self.ang = 0
        self.angY = math.pi*0.5

        self.lastMousePos = (0,0)

    def toggleAttachment( self ):
        if self.attachmentNode:
            self.attached = not self.attached
        else:
            self.attached = False

    def attachTo( self, other ):

        self.attachmentNode = other
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
                self.ang -= dx
                #self.ang = max( 0, min( self.ang, math.pi*0.49 ) )
                dy = self.lastMousePos[1] - y
                self.angY -= dy
                self.angY = max( 0.01, min( self.angY, math.pi ) )
        
            self.lastMousePos = (x,y)

        speed = self.speed
        if is_down( self.bSpeed ):
            speed = speed*3

        if self.attachmentNode and self.attached:
            self.focusPoint = self.attachmentNode.getPos()
        else:
            sideways = (is_down(self.bRight)-is_down(self.bLeft))*speed
            forward = (is_down(self.bForward)-is_down(self.bBackward))*speed

            quat = Quat()
            quat.setFromAxisAngle( -180*self.ang/math.pi, LVector3f.unitZ())
            rotated = quat.xform( LVector3f( -forward, sideways, 0 ))

            self.focusPoint += rotated

        
        self.focusNode.setPos( self.focusPoint )

        radius = self.zoom
        self.angY = max( 0, min( math.pi*0.4, self.angY ) )
        rY = math.sin( self.angY )
        self.nodePos = self.focusPoint + LVector3f( rY*math.cos(self.ang)*radius, -rY*math.sin(self.ang)*radius, radius*math.cos( self.angY) )

        self.node.setPos( self.nodePos )
        self.node.lookAt( self.focusNode, LVector3f.unitZ() )

        return Task.cont


