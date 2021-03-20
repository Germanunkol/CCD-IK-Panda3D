Simple Cyclic Coordinate Decent implementation for Panda3D
===========================================================
Numerical Inverse Kinematics solver with simple constraints

![](CCDIK-2020-12-29_17.31.24.gif)

Features:
---------
- Hinge joints
- Ball joints
- Uses Panda3D's Bone system, so should be relatively easy to use with an existing skeleton. See usage notes below.
- Pure python, no dependencies beyond Panda3D

Not implemented:
----------------
- Other constraints
- Target rotation (only target position is currently taken into account)

Samples:
---------------
In all samples, use WASD to move and middle mouse button to rotate.

### Reacher Sample: ###

The Reacher sample shows how to set up a simple IK chain with constraints which reaches for a moving target point:

```
python3 ReacherSample.py
```

![](IK-2021-03-08_22.14.25.gif)

### Biped Sample: ###

The Biped sample shows a very basic humanoid, where the legs are placed by IK. The basic character setup is: The root is a "torso" node. To this, a hip node is rigidly attached. There are two legs, each is its own IKChain. To let the character walk, the torso node is moved, and everything else moves with it.
While the torso moves, the legs have target points on the ground. The IK makes sure that they stay attached to these points, even when the torso moves. Periodically, the legs are moved to a new target to take a step. This new target point is always a point projected onto the floor in front of the body. How often a step is taken and how far infront of the character the new target position is depends on the movement speed of the character.

```
python3 BipedSample.py
```

Press + and - to speed the character up or slow it down. Note that this is very simplified - the step length should likely be increased for higher speeds.

![](Biped-2021-03-13_22.23.21.gif)

### Tentacle Sample: ###

This sample shows how to set up a Bone chain from an existing mesh. Note that all bones are attached to their parent, each bone has a corresponding vertex group which it controls and all bones point down the Y axis, so their constraints will be set up to rotate over their X or Z axes.

```
python3 TentacleSample.py
```



Usage notes:
------------

The best way to learn is probably to look at the samples. However, here are also rough step-by-step instructions of how to set up and use an IKChain:

### Creating a chain from code ###
- Create a new IKChain, then create bones by calling "addBone", then call "finalize" before using the chain.

- The "offset" parameter is a vector which describes the difference between the new bone's position and that of its parent.
- The "rotAxis" can either be a (unit) vector, or "None". In the latter case, the constraint acts like that of a ball joint (or maybe more like two perpendicular hinge joints). In the former case, the rotAxis is the axis of the hinge joint. **Note that I only tested axis which are perpendicular to the "offset" vector, more specifically I usually use unitX or unitY.**
- After adding all bones, you must call "IKChain.finalize()".

### Creating a chain from a rigged mesh ###
Instead of creating an IKChain manually, you can also use a predefined armature that comes with a rigged mesh. Usually, a chain will be set up for a certain subset of the bones (for example one chain for all the bones in the left arm and one for all of those in the right arm), so you need to specify which bones should be controlled by the IK algorithm (see also the TentacleSample.py):

- First load the model using `model = loader.loadModel("RiggedCharacter.bam")`
- Then find the already existing character using `characterNodePath = model.find("-Character")`
- Create an actor (required for access to the predefined joints): `actor = Actor(characterNodePath)`
- Next we need a list of joints which the IK algorithm should control, along with their constraints. Create an empty list. Then add entries which are dictionaries. Each holds four entries: the "name" (string) of the joint to be used, the "axis" (None, "auto" or LVector3f, see below) to be used for rotation and the "minAng" (float) and "maxAng" (float) constraints.
- The "axis" of each joint can be None (ball joint), "auto" (algorithm tries to determine the axis automatically - only works if the bone is rotated with respect to its parent!) or LVector3f (for instance LVector3f.unitZ() )
- Figure out which node you want to act as the root of the chain. This can be any node in the scene graph, or it can be the parent bone of the first bone in your chain (in this case, get access to it via `root = actor.exposeJoint(...)`, *Note: Untested!*)
- Lastly, create the IKChain using the static method IKChain.fromArmature. There's no need to call "finalize", the fromArmature method does this for you.

Hints:
- When exporting from blender, make sure there is a vertex group for every bone - even if it's empty (i.e. if you have a bone called "Bone.002" there must be a vertex group called "Bone.002"). Otherwise, the bone gets positioned at the model root by Panda3D, and offsets are no longer correct. Hint: in blender, these vertex groups are set up automatically when parenting an armature to the mesh and selecting the automatic weight assignment.
- Make sure every bone is connected to its predecessor.

### Solving IK ###
Whether you created the chain from scratch or from a rigged model, you need to set the chain's target using `IKChain.setTarget( targetNode )`. After that, you should call `IKChain.updateIK()` once a frame, possibly while moving the root of the chain or the target.

### General notes ###

- CCD tends to rotate the last segments (the ones close to the end effector) much more than those close to the root, which can be undesirable. To avoid this, an annealing strategy should be implemented, which weighs the movement of bones depending on their distance to the root.
- Ball joints (rotAxis=None) currently have no way of limiting a bone's "roll" angle. This means your bones may spin uncontrollably around their own axis. A workaround would be to replace a ball joint with two hinge joints (one rotating on LVector3f.unitX() and the other one on LVector3f.unitZ(), for example).

