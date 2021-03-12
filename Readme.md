Simple Cyclic Coordinate Decent implementation for Panda3D
===========================================================
Numerical Inverse Kinematics solver with simple constraints

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

Running Samples:
---------------
In all samples, use WASD to move and middle mouse button to rotate.

The Reacher sample shows how to set up a simple IK chain with constraints which reaches for a moving target point:

```
python3 ReacherSample.py
```

The Biped sample shows a very basic humanoid, where the legs are placed by IK. The basic character setup is: The root is a "torso" node. To this, a hip node is rigidly attached. There are two legs, each is its own IKChain. To let the character walk, the torso node is moved, and everything else moves with it.
While the torso moves, the legs have target points on the ground. The IK makes sure that they stay attached to these points, even when the torso moves. Periodically, the legs are moved to a new target to take a step. This new target point is always a point projected onto the floor in front of the body. How often a step is taken and how far infront of the character the new target position is depends on the movement speed of the character.

Press + and - to speed the character up or slow it down. Note that this is very simplified - the step length should likely be increased for higher speeds.

Usage notes:
------------
- In the current form (state March 2021), the IK Chain creates its own Character and CharacterJoints. Ideally, in the future, it should be able to use an existing Character, loaded with a rigged mesh.
- Create a new IKChain, then create bones by calling "addBone".
- The "offset" parameter is a vector which describes the difference between the new bone's position and that of its parent. Note that I only tested with offsets which are along the Y axis, i.e. are multiples of LVector3f.unitY().
- The "rotAxis" can either be a (unit) vector, or "None". In the latter case, the constraint acts like that of a ball joint (or maybe more like two perpendicular hinge joints). In the former case, the rotAxis is the axis of the hinge joint. Note that I only tested axis which are perpendicular to the "offset" vector, more specifically I usually use unitX or unitY.
- After adding all bones, you must call "IKChain.finalize()".
- Call IKChain.setTarget and IKChain.updateIK to make the chain (try to) reach for a target.
