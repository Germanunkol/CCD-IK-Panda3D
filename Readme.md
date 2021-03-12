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

Running Sample:
---------------

```
python3 IKChain.py
```

Use WASD to move, middle mouse button to rotate

Usage notes:
------------
- In the current form (state March 2021), the IK Chain creates its own Character and CharacterJoints. Ideally, in the future, it should be able to use an existing Character, loaded with a rigged mesh.
- Create a new IKChain, then create bones by calling "addBone".
- The "offset" parameter is a vector which describes the difference between the new bone's position and that of its parent. Note that I only tested with offsets which are along the Y axis, i.e. are multiples of LVector3f.unitY().
- The "rotAxis" can either be a (unit) vector, or "None". In the latter case, the constraint acts like that of a ball joint (or maybe more like two perpendicular hinge joints). In the former case, the rotAxis is the axis of the hinge joint. Note that I only tested axis which are perpendicular to the "offset" vector, more specifically I usually use unitX or unitY.
- After adding all bones, you must call "IKChain.finalize()".
- Call IKChain.setTarget and IKChain.updateIK to make the chain (try to) reach for a target.
