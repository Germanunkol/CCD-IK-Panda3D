from IKChain import IKChain
from Utils import *

if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CameraControl import CameraControl

    class MyApp(ShowBase):

        def __init__(self):
            ShowBase.__init__(self)
            base.disableMouse()

            wp = WindowProperties()
            wp.setSize(1800, 960)
            self.win.requestProperties(wp)

            base.setBackgroundColor(0,0,0)

            grid = createGrid( 20, 1 )
            render.attachNewNode( grid )
            axes = createAxes( 1000, bothways=True, thickness=3 )
            render.attachNewNode( axes )

            #
            self.ikChain = IKChain( render )

            bone = self.ikChain.addBone( offset=LVector3f.unitY(),
                    minAng = -math.pi*0.3,
                    maxAng = math.pi*0.3,
                    rotAxis = LVector3f.unitX()
                    )

            for i in range( 3 ):
                bone = self.ikChain.addBone( offset=LVector3f.unitY(),
                        minAng = -math.pi*0.3,
                        maxAng = math.pi*0.3,
                        rotAxis = LVector3f.unitZ(),
                        parentBone = bone
                        )

            self.ikChain.finalize()

            self.ikChain.debugDisplay()

            #factory.debugInfo( render )
            focusNode = render.attachNewNode( "CameraFocusNode" )
            self.camControl = CameraControl( camera, self.mouseWatcherNode )
            
            self.taskMgr.add( self.camControl.moveCamera, "MoveCameraTask")

            self.accept( "wheel_down", self.camControl.wheelDown )
            self.accept( "wheel_up", self.camControl.wheelUp )

            self.accept( "z", self.toggleAnimation )
            self.animateTarget = True

            ##################################
            ## Target point:
            col = (random.random(), random.random(), random.random())

            lines = LineSegs()
            lines.setThickness(15)
            lines.setColor( col[0], col[1], col[2] )
            lines.moveTo(0, 0, 0)
            #lines.drawTo(np.getPos(parentNode))
            #point = render.attachNewNode("Point")
            self.ikTarget = render.attachNewNode(lines.create())
            self.ikTarget.setPos( 2,0,2 )
            
            self.taskMgr.add( self.moveTarget, "MoveTarget" )

            self.ikChain.setTarget( self.ikTarget )

        def moveTarget( self, task ):
            if self.animateTarget:
                self.ikTarget.setPos( 2.5*math.sin(task.time), 5*math.sin(task.time*1.6+2), math.cos(task.time*1.6+2) )

            self.ikChain.updateIK()
            return task.cont

        def toggleAnimation( self ):
            self.animateTarget = (self.animateTarget == False)


    app = MyApp()
    app.run()
