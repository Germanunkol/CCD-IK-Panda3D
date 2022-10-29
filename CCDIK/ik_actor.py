from direct.actor.Actor import Actor
from .ik_chain import IKChain

class IKActor():

    def __init__( self, model ):

        self.model = model

        self.character_node = self.model.find("-Character")
        self.actor = Actor(self.character_node)

        self.parent = None

        self.control_nodes = {}
        self.expose_nodes = {}

        # Retrieve all joints of the actor:
        joints = self.actor.get_joints()

        # Save all joint names for convenience:
        self.joint_names = [j.get_name() for j in joints]

        # Keep dictionary of joints, accessible by name for convenience
        self.joints = {}
        for j in joints:
            self.joints[j.get_name()] = j
            #print(j.get_name(), j.get_transform())

        for joint_name, j in self.joints.items():
            parent_control_node = self.get_control_node( joint_name )
            for c in j.get_children():
                child_control_node = self.get_control_node( c.get_name() )
                child_control_node.reparent_to( parent_control_node )

        # Find all nodes which have no parent yet. Those should be repatented to the actor itself:
        for joint_name, j in self.joints.items():
            cn = self.get_control_node( joint_name )
            if not cn.get_parent():
                print(f"Re-parenting {name} to root!")
                cn.reparent_to( self.actor )

        #print("LS")
        #self.actor.ls()

    def reparent_to( self, parent ):

        self.actor.reparent_to( parent )
        self.parent = parent

    def get_control_node( self, joint_name ):

        assert joint_name in self.joint_names, "Cannot control joint '" + joint_name + "': Not found!"

        if joint_name in self.control_nodes.keys():
            return self.control_nodes[joint_name]
        else:
            control_node = self.actor.control_joint( None, "modelRoot", joint_name )
            self.control_nodes[joint_name] = control_node
            return control_node

    def get_expose_node( self, joint_name ):

        assert joint_name in self.joint_names, "Cannot expose joint '" + joint_name + "': Not found!"

        if joint_name in self.expose_nodes.keys():
            return self.expose_nodes[joint_name]
        else:
            expose_node = self.actor.expose_joint( None, "modelRoot", joint_name )
            self.expose_nodes[joint_name] = expose_node
            return expose_node
         
    def create_ik_chain( self, joint_names ):
        
        chain = IKChain( actor=self.actor )

        parent_bone = None
        for joint_name in joint_names:
            assert joint_name in self.joints.keys(), "Joint '" + joint_name + "' cannot be added to chain - not found!" 
            joint = self.joints[joint_name]
            control_node = self.get_control_node( joint_name )
            new_bone = chain.add_joint( joint, control_node, parent_bone=parent_bone )
            parent_bone = new_bone

        #chain.debug_display()

        return chain
