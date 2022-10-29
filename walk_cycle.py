# Simple helper class which keeps a list of times for each leg and determines whether they should
# be moved. Must be updated every frame via the update time.

# Note that "Durations" and "Times" in this class could also be named "distances" instead, they
# don't really need to be times - usually what you pass to update_time() should actually increase
# when a character walks faster.
class WalkCycle():

    def __init__( self, num_legs, cycle_duration ):
        self.num_legs = num_legs
        self.cycle_duration = cycle_duration
        self.cycle_time = 0
        self.moved_legs_this_cycle = [False]*num_legs
        self.leg_step_time = [0]*num_legs
        self.step_required = [False]*num_legs
        #self.leg_movement_speed = 0

        step_time1 = 0
        step_time2 = step_time1 + cycle_duration*0.5
        for i in range(int(num_legs/2)):
            if i % 2 == 0:
                self.set_leg_step_time( i*2, step_time1 )
                self.set_leg_step_time( i*2+1, step_time2 )
            else:
                self.set_leg_step_time( i*2, step_time2 )
                self.set_leg_step_time( i*2+1, step_time1 )

    #def set_leg_movement_speed( self, speed ):
        #self.leg_movement_speed = speed

    def set_leg_step_time( self, leg_index, time ):
        while time > self.cycle_duration:
            time -= self.cycle_duration
        self.leg_step_time[leg_index] = time

    def update_time( self, dt ):
        self.cycle_time += dt
        if self.cycle_time > self.cycle_duration:
            self.moved_legs_this_cycle = [False]*self.num_legs
            self.cycle_time = self.cycle_time % self.cycle_duration

        for i in range( self.num_legs ):
            #self.step_required[i] = False
            if self.cycle_time > self.leg_step_time[i]:
                if self.moved_legs_this_cycle[i] == False:
                    self.moved_legs_this_cycle[i] = True
                    self.step_required[i] = True

    def step( self, leg_index ):
        self.step_required[leg_index] = False


