# FRC Robot Interface Code# 

The robot interface code is the part of our setup which sits at the lowest level, closest to the hardware.  This could be real robot hardware.  Or, using slightly different code, various simulated robot hardware.  By isolating this layer of the communications to one bit of the software, we can easily swap between hardware and the various simulators.

This page attempts to document this code.

## Overall Framework## 

The robot interface code is in src/ros_control_boilerplate.



Much of the robot interface code is shared between all implementations.  This is typically code which parses config params and sets up generic data structures.  The rest

## Common Robot Interface Code### 