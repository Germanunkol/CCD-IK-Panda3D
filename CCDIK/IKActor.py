from direct.actor.Actor import Actor
from .IKChain import IKChain

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

        # Keep dictionary of joints, accessible by name for convenience
        self.joints = {}
        for j in joints:
            self.joints[j.getName()] = j
            #print(j.getName(), j.getTransform())

        for jointName, j in self.joints.items():
            parentControlNode = self.getControlNode( jointName )
            for c in j.getChildren():
                childControlNode = self.getControlNode( c.getName() )
                childControlNode.reparentTo( parentControlNode )

        # Find all nodes which have no parent yet. Those should be repatented to the actor itself:
        for jointName, j in self.joints.items():
            cn = self.getControlNode( jointName )
            if not cn.getParent():
                print(f"Re-parenting {name} to root!")
                cn.reparentTo( self.actor )

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
            exposeNode = self.actor.exposeJoint( None, "modelRoot", jointName )
            self.exposeNodes[jointName] = exposeNode
            return exposeNode
         
    def createIKChain( self, jointNames ):
        
        chain = IKChain( actor=self.actor )

        parentBone = None
        for jointName in jointNames:
            assert jointName in self.joints.keys(), "Joint '" + jointName + "' cannot be added to chain - not found!" 
            joint = self.joints[jointName]
            controlNode = self.getControlNode( jointName )
            newBone = chain.addJoint( joint, controlNode, parentBone=parentBone )
            parentBone = newBone

        #chain.debugDisplay()

        return chain
