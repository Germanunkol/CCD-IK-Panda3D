
class WalkCycle():

    def __init__( self, numLegs, cycleDuration ):
        self.numLegs = numLegs
        self.cycleDuration = cycleDuration
        self.cycleTime = 0
        self.movedLegsThisCycle = [False]*numLegs
        self.legStepTime = [0]*numLegs
        self.stepRequired = [False]*numLegs
        #self.legMovementSpeed = 0

        stepTime1 = 0
        stepTime2 = stepTime1 + cycleDuration*0.5
        for i in range(int(numLegs/2)):
            if i % 2 == 0:
                self.setLegStepTime( i*2, stepTime1 )
                self.setLegStepTime( i*2+1, stepTime2 )
            else:
                self.setLegStepTime( i*2, stepTime2 )
                self.setLegStepTime( i*2+1, stepTime1 )

    #def setLegMovementSpeed( self, speed ):
        #self.legMovementSpeed = speed

    def setLegStepTime( self, legIndex, time ):
        while time > self.cycleDuration:
            time -= self.cycleDuration
        self.legStepTime[legIndex] = time

    def updateTime( self, dt ):
        self.cycleTime += dt
        print(self.cycleTime, self.cycleDuration)
        if self.cycleTime > self.cycleDuration:
            self.movedLegsThisCycle = [False]*self.numLegs
            self.cycleTime = self.cycleTime % self.cycleDuration

        for i in range( self.numLegs ):
            #self.stepRequired[i] = False
            if self.cycleTime > self.legStepTime[i]:
                if self.movedLegsThisCycle[i] == False:
                    self.movedLegsThisCycle[i] = True
                    self.stepRequired[i] = True

    def step( self, legIndex ):
        self.stepRequired[legIndex] = False


