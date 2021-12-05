from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

app = ShowBase()

def printBones( model ):
    characterNode = model.find("-Character")
    c = characterNode.node().getBundle(0).getChild(0)
    while c:
        print(c.getName())
        if isinstance( c, CharacterJoint ):
            print(characterNode.node().findJoint( c.getName() ).getTransform())
            mat = characterNode.node().findJoint( c.getName() ).getTransform()

            t = mat.getRow3(3)
            rot = Quat()
            rot.setFromMatrix( mat )
            rot.normalize()
            axis = rot.getAxisNormalized()
            ang = rot.getAngleRad()
            print(axis, ang, t)

        if c.getNumChildren() == 0:
            break
        c = c.getChild(0)

print("Tentacle.bam bones:")
model = loader.loadModel( "Tentacle.bam" )
printBones( model )

#print("TentacleBroken.bam bones:")
#model = loader.loadModel( "TentacleBroken.bam" )
#printBones( model )

app.run()
