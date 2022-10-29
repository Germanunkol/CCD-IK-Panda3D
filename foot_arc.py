import math
from panda3d.core import LVector3f

class FootArc():

    def __init__( self, start_pos, end_pos, offset_dir=LVector3f.unit_z(), max_step_height=0.1, max_step_dist=0.4 ):

        self.start_pos = start_pos
        self.end_pos = end_pos
        self.step_diff = (end_pos - start_pos)
        self.full_step_length = self.step_diff.length()
        self.cur_step_length = 0
        self.offset_dir = offset_dir
        self.cur_pos = self.start_pos

        # Calculate the step height, depending on the distance of the step. If it's a very small
        # step (i.e. much smaller than the norm step dist "max_step_dist"), then also don't lift the
        # foot as high as you normally would:
        step_dist = (start_pos - end_pos).length()
        self.step_height = max_step_height*min(step_dist/max_step_dist, 1)

    def update( self, amount ):

        self.cur_step_length += amount
        self.cur_step_length = min( self.cur_step_length, self.full_step_length )

        step_fraction = self.cur_step_length/self.full_step_length

        # Variable height changing from 0 to 1 and back to 0
        # TODO: Replace by cheaper function?
        height = math.sin( step_fraction*math.pi )
        #height = math.sin((1-step_fraction)*(1-step_fraction)*math.pi)

        self.cur_pos = self.start_pos + self.step_diff*step_fraction +\
                self.offset_dir*self.step_height*height

    def get_pos( self ):
        return self.cur_pos

    def done( self ):
        return self.cur_step_length >= self.full_step_length

            #diff = self.planned_foot_target_left.get_pos(render) - self.foot_target_left.get_pos()
         #   if diff.length() < leg_move_dist:
         #       self.foot_target_left.set_pos( self.planned_foot_target_left.get_pos( render ) )
         #       self.step_left = False
         #   else:
         #       moved = self.foot_target_left.get_pos() + diff.normalized()*leg_move_dist
         #       self.foot_target_left.set_pos( moved )



