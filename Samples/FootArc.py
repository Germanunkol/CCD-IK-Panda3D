import math
from panda3d.core import LVector3f

class FootArc():

    def __init__( self, startPos, endPos, offsetDir=LVector3f.unitZ(), stepHeight=0.1 ):

        self.startPos = startPos
        self.endPos = endPos
        self.stepDiff = (endPos - startPos)
        self.fullStepLength = self.stepDiff.length()
        self.curStepLength = 0
        self.offsetDir = offsetDir
        self.stepHeight = stepHeight
        self.curPos = self.startPos

    def update( self, amount ):

        self.curStepLength += amount
        self.curStepLength = min( self.curStepLength, self.fullStepLength )

        stepFraction = self.curStepLength/self.fullStepLength

        # Variable height changing from 0 to 1 and back to 0
        # TODO: Replace by cheaper function?
        height = math.sin( stepFraction*math.pi )

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



