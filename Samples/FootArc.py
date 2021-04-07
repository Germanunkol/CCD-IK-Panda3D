import math
from panda3d.core import LVector3f

class FootArc():

    def __init__( self, startPos, endPos, offsetDir=LVector3f.unitZ(), maxStepHeight=0.1, maxStepDist=0.4 ):

        self.startPos = startPos
        self.endPos = endPos
        self.stepDiff = (endPos - startPos)
        self.fullStepLength = self.stepDiff.length()
        self.curStepLength = 0
        self.offsetDir = offsetDir
        self.curPos = self.startPos

        # Calculate the step height, depending on the distance of the step. If it's a very small
        # step (i.e. much smaller than the norm step dist "maxStepDist"), then also don't lift the
        # foot as high as you normally would:
        stepDist = (startPos - endPos).length()
        self.stepHeight = maxStepHeight*min(stepDist/maxStepDist, 1)

    def update( self, amount ):

        self.curStepLength += amount
        self.curStepLength = min( self.curStepLength, self.fullStepLength )

        stepFraction = self.curStepLength/self.fullStepLength

        # Variable height changing from 0 to 1 and back to 0
        # TODO: Replace by cheaper function?
        height = math.sin( stepFraction*math.pi )
        #height = math.sin((1-stepFraction)*(1-stepFraction)*math.pi)

        self.curPos = self.startPos + self.stepDiff*stepFraction +\
                self.offsetDir*self.stepHeight*height

    def getPos( self ):
        return self.curPos

    def done( self ):
        return self.curStepLength >= self.fullStepLength

            #diff = self.plannedFootTargetLeft.getPos(render) - self.footTargetLeft.getPos()
         #   if diff.length() < legMoveDist:
         #       self.footTargetLeft.setPos( self.plannedFootTargetLeft.getPos( render ) )
         #       self.stepLeft = False
         #   else:
         #       moved = self.footTargetLeft.getPos() + diff.normalized()*legMoveDist
         #       self.footTargetLeft.setPos( moved )



