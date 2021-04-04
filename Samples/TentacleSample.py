from IKChain import IKChain
from Utils import *
from direct.actor.Actor import Actor
from panda3d.core import *

if __name__ == "__main__":

    from direct.showbase.ShowBase import ShowBase
    from CameraControl import CameraControl

    class MyApp(ShowBase):

        def __init__(self):
            ShowBase.__init__(self)
            base.disableMouse()
            base.setFrameRateMeter(True)

            wp = WindowProperties()
            wp.setSize(1800, 960)
            self.win.requestProperties(wp)

            base.setBackgroundColor(0,0,0)

            grid = createGrid( 20, 1 )
            render.attachNewNode( grid )
            axes = createAxes( 1000, bothways=True, thickness=3 )
            render.attachNewNode( axes )


            ###########################################
            ## Set up lights:

            light = PointLight("PointLight")
            light.setColorTemperature( 5000 )
            #light.attenuation = (1, 0.5, 0.5)
            light.attenuation = (0.75, 0, 0.05)
            lightNode = render.attachNewNode( light )
            lightNode.setPos( 0, 0, 3 )
            render.setLight( lightNode ) 
            #light.setShadowCaster(True, 1024, 1024, -2000 ) # low sort value to render early!

            alight = AmbientLight('alight')
            alight.setColor((0.5, 0.5, 0.5, 1))
            alnp = render.attachNewNode(alight)
            render.setLight(alnp)

            ############################################
            ## Set up model:

            self.root = render.attachNewNode("Root")
            self.root.setPos( 0, 0, 2 )

            self.model = loader.loadModel( "Meshes/Tentacle.bam" )
            #self.model.reparentTo(root)
            #self.model.ls()

            m = Material()
            m.setBaseColor((1, 0.8, 0.3, 1))
            self.model.setMaterial(m)
            
            characterNode = self.model.find("-Character")
            actor = Actor(characterNode)#, {'simplechar' : anim})
            actor.reparentTo( self.root )

            jointList = []
            jointList.append( {"name":"Bone", "axis":LVector3f.unitX(),
                "minAng":-math.pi*0.3, "maxAng":math.pi*0.3} )
            for i in range(1,8):
                #jointList.append( {"name":"Bone.{:03d}".format(i), "axis":None,
                #    "minAng":-math.pi*0.3, "maxAng":math.pi*0.3} )
                if i % 2 == 0:
                    jointList.append( {"name":"Bone.{:03d}".format(i), "axis":LVector3f.unitX(),
                        "minAng":-math.pi*0.3, "maxAng":math.pi*0.3} )
                else:
                    jointList.append( {"name":"Bone.{:03d}".format(i), "axis":LVector3f.unitZ(),
                        "minAng":-math.pi*0.3, "maxAng":math.pi*0.3} )
             

            self.ikChain = IKChain.fromArmature( characterNode.node(), self.root, actor, jointList )

            print("chain:")
            self.ikChain.parent.ls()
            self.ikChain.debugDisplay( lineLength=0.5 )

            #factory.debugInfo( render )
            focusNode = render.attachNewNode( "CameraFocusNode" )
            self.camControl = CameraControl( camera, self.mouseWatcherNode )
            
            self.taskMgr.add( self.camControl.moveCamera, "MoveCameraTask")

            self.accept( "wheel_down", self.camControl.wheelDown )
            self.accept( "wheel_up", self.camControl.wheelUp )

            self.animateTarget = True
            self.accept( "z", self.toggleAnimation )
            self.accept( "j-repeat", self.moveRootDown )
            self.accept( "k-repeat", self.moveRootUp )

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
            
            self.taskMgr.add( self.moveTarget, "MoveTarget" )

            self.ikChain.setTarget( self.ikTarget )

            print("---------------------------------")
            print("Full tree:")
            render.ls()
            print("---------------------------------")

        def moveTarget( self, task ):
            if self.animateTarget:
                speed = 0.4
                self.ikTarget.setPos( 2.5*math.sin(speed*task.time),
                        13*math.sin(speed*task.time*1.6+2),
                        math.cos(speed*task.time*1.6+2) )

            self.ikChain.updateIK()
            return task.cont

        def moveRootUp( self ):
            self.root.setPos( self.root.getPos() + LVector3f.unitZ()*globalClock.getDt()*3 )

        def moveRootDown( self ):
            self.root.setPos( self.root.getPos() - LVector3f.unitZ()*globalClock.getDt()*3 )


        def toggleAnimation( self ):
            self.animate = (self.animateTarget == False)


    app = MyApp()
    app.run()