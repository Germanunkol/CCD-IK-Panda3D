from direct.task import Task

from panda3d.core import LPoint3, LVector3f
from panda3d.core import KeyboardButton, MouseButton
from panda3d.core import Quat
import math
from .utils import *

class CameraControl:

    def __init__( self, node, mouse_watcher_node, speed=0.2 ):

        self.node = node

        self.heading_node = render.attach_new_node( "Camera_heading_rot_node" )
        self.pitch_node = self.heading_node.attach_new_node( "Camera_pitch_rot_node" )
        self.node.reparent_to( self.pitch_node )

        self.heading_node.set_pos( 0, 0, 0.5 )

        #self.focus_node = render.attach_new_node("Camera_focus_node")
        self.attached = False

        self.heading_node.attach_new_node( create_axes( 0.1 ) )

        self.focus_point = LPoint3()

        self.mouse_watcher_node = mouse_watcher_node

        self.b_forward = KeyboardButton.ascii_key('w')
        self.b_backward = KeyboardButton.ascii_key('s')
        self.b_left = KeyboardButton.ascii_key('a')
        self.b_right = KeyboardButton.ascii_key('d')
        self.b_speed = KeyboardButton.lshift()

        self.speed = speed

        self.zoom = 10


        self.heading = 0
        self.pitch = -45

        self.node.set_pos( 0, -self.zoom, 0 )
        self.pitch_node.set_hpr( 0, self.pitch, 0 )
        self.heading_node.set_hpr( self.heading, 0, 0 )

        self.last_mouse_pos = (0,0)

    def attach_to( self, other ):

        self.heading_node.reparent_to( other )
        self.heading_node.set_pos( 0, 0, 0 )
        self.attached = True

    def wheel_up( self ):

        self.zoom = self.zoom - 1
        self.zoom = min( max( self.zoom, 1 ), 2000 )

    def wheel_down( self ):

        self.zoom = self.zoom + 1
        self.zoom = min( max( self.zoom, 1 ), 2000 )

    def move_camera( self, task ):

        is_down = self.mouse_watcher_node.is_button_down

        if self.mouse_watcher_node.has_mouse():
            x = self.mouse_watcher_node.get_mouse_x()
            y = self.mouse_watcher_node.get_mouse_y()
            if is_down( MouseButton.two() ):
                dx = self.last_mouse_pos[0] - x
                self.heading += dx*25
                #self.ang = max( 0, min( self.ang, math.pi*0.49 ) )
                dy = self.last_mouse_pos[1] - y
                self.pitch -= dy*25
                self.pitch = max( self.pitch, -80 )
                self.pitch = min( self.pitch, 80 )
                #self.ang_y = max( 0.01, min( self.ang_y, math.pi ) )
        
            self.last_mouse_pos = (x,y)

        speed = self.speed
        if is_down( self.b_speed ):
            speed = speed*3

        if not self.attached:
            forward = (is_down(self.b_right)-is_down(self.b_left))*speed
            sideways = -(is_down(self.b_forward)-is_down(self.b_backward))*speed

            quat = Quat()
            quat.set_from_axis_angle( self.heading, LVector3f.unit_z())
            rotated = quat.xform( LVector3f( -forward, sideways, 0 ))

            self.heading_node.set_pos( self.heading_node.get_pos() - rotated )

        
        #self.ang_y = max( 0, min( math.pi*0.4, self.ang_y ) )
        #r_y = math.sin( self.ang_y )
        #self.node_pos = self.focus_point + LVector3f( math.sin(self.ang_y)*math.cos(self.ang)*radius, -math.sin(self.ang)*radius, radius*math.cos( self.ang_y) )

        self.node.look_at( self.heading_node )

        self.node.set_pos( 0, -self.zoom, 0 )
        self.pitch_node.set_hpr( 0, self.pitch, 0 )
        self.heading_node.set_hpr( self.heading, 0, 0 )

        return Task.cont


