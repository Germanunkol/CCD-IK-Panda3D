from direct.actor.Actor import Actor
from IKChain import IKChain

class IKActor():

    def __init__( self, model ):

        self.model = model

        self.characterNode = self.model.find("-Character")
        self.actor = Actor(self.characterNode)

        self.parent = None

        self.controlNodes = {}
        self.exposeNodes = {}

        # Retrieve all joints of the actor:
        joints = self.actor.getJoints()

        # Save all joint names for convenience:
        self.jointNames = [j.getName() for j in joints]
        print("Found joints:", self.jointNames)

        # Keep dictionary of joints, accessible by name for convenience
        self.joints = {}
        for j in joints:
            self.joints[j.getName()] = j
            print(j.getName(), j.getTransform())

        #for j in joints:
        #    print("j", j.getName())
        #    parentControlNode = self.getControlNode( j.getName() )
        #    for c in j.getChildren():
        #        print("child", c.getName())

        #print("LS")
        #self.actor.ls()

    def reparentTo( self, parent ):

        self.actor.reparentTo( parent )
        self.parent = parent

    def getControlNode( self, jointName ):

        assert jointName in self.jointNames, "Cannot control joint '" + jointName + "': Not found!"

        if jointName in self.controlNodes.keys():
            return self.controlNodes[jointName]
        else:
            controlNode = self.actor.controlJoint( None, "modelRoot", jointName )
            self.controlNodes[jointName] = controlNode
            return controlNode

    def getExposeNode( self, jointName ):

        assert jointName in self.jointNames, "Cannot expose joint '" + jointName + "': Not found!"

        if jointName in self.exposeNodes.keys():
            return self.exposeNodes[jointName]
        else:
            exposeNode = self.actor.exposeNode( None, "modelRoot", jointName )
            self.exposeNodes[jointName] = exposeNode
            return exposeNode
         
    def createIKChain( self, jointNames ):
        
        chain = IKChain( actor=self.actor )

        parentBone = None
        for jointName in jointNames:
            assert jointName in self.joints.keys(), "Joint '" + jointName + "' cannot be added to chain - not found!" 
            joint = self.joints[jointName]
            controlNode = self.getControlNode( jointName )
            newBone = chain.addBone( joint, controlNode, parentBone=parentBone )
            if parentBone:
                controlNode.reparentTo( parentBone.controlNode )
            else:
                controlNode.reparentTo( self.actor )
            parentBone = newBone

        #chain.debugDisplay()

        return chain
