## Building ROS on Windows Notes## 

Initial instructions for installing ROS is [http://wiki.ros.org/Installation/Windows](http://wiki.ros.org/Installation/Windows).

Changed location of RobotOrient.srv to frc_msgs from teleop_joystick package, and blocked some of the packages from being compiled on Windows.
Packages blocked:
  - behaviors
  - goal_detection
  - laser_targets (subsequently removed from the repo)
  - pixy_get_lines (subsequently removed from the repo)
  - teleop_joystick_control
  - screen_to_world (subsequently removed from the repo)
  - state_listener
  - navx_publisher
  - ros_control_boilerplate (had gettimeofday and clock_gettime, now uses standard C++ clock functions)
  - zms_writer
  - uptime

The pid package, the control_toolbox package, and the px4flow and mavlink_serial_client in the px_ros_pkg all had Linux-specific function calls (gettimeofday for px_ros_pkg, erand48 and `'>>': shift count negative or too big, undefined behavior` for control_toolbox, strange syntax errors in pid).

I just removed the entire ros_controllers directory -- what could go wrong?

Added `if(NOT WIN32)` around the testing part of ddynamic reconfigure

Added not win32 around terabee