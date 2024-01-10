###  Auto Align Documentatision ### 

Theoretically this document should explain the current state of the auto align code with explanations about design decisions and the end goal for this code. We never got auto align fully functional this season, so the goal is the get the auto align code from this year into a clean/commented boilerplate-ish state.

####  Initial auto align goals #### 
This year (2019) we had mechanisms for placing game pieces on two different sides of the robot, and due to the field there were multiple different places to score both of the game pieces. We could place cargo (balls) in effectively two different places(though at three different angles). We could place hatch panels (discs) in effectively 5-ish places. After analysing all of the different places to place the game pieces we ended up with the following sensor setup: On the cargo side of the robot we had a monoscopic camera and two terabee(distance) sensors on the corner of the robot. On the hatch panel side of the robot we also had two terabee sensors on the corners alogn with the Zed (stereoscopic camera). With this sensor setup we planned for three different auto align algorithms. Aligning cargo side at the rocket, aligning cargo side at the cargo ship, and placing hatch panels anywhere.

#####  Aligning cargo side at the rocket ##### 
{{:programming:auto_align_img.png?400|}}

This figure approximately shows the geometry of aligning cargo side at the rocket. The red lines represent the terabee(distance) sensors, and we ended up only using one on each corner. Using these distance sensors we were able to run just a PID loop on their difference because when perfectly aligned the sensors on the corners would have the same value and the further either side we go the greater the distance the sensor on that side would read. This algorithm we did actually get working, but unfortunately we didn't place too much cargo until district champs where for some reason(I can't remember) it didn't seem useful. This algorithm wasn't actually in the final auto align design because we didn't have a good way to distinguish in code between placing cargo on the rocket ship and placing cargo on the cargo ship it didn't make sense to make these two different algorithms. 

#####  Aligning cargo side at the cargo ship ##### 



#####  The parts of the code ##### 
  - The base align servers
    - This year (2019) we had mechanisms on two different

#####  Random Notes ##### 

For anyone who is interested in working on the logic for auto align in the future these are the main files being used for doing it.

**base_align_server.h**

contains the base functionality for auto align and is derived by specific auto align classes to get more specific behavior
https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/behaviors/include/behaviors/base_align_server.h

**base_align_with_vision.h**

derived from base align server but contains logic specific to using vision. This is what was going to be used for actual alignment with two instances of this class being generated for specific alignment in the following two files.
https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/behaviors/include/behaviors/base_align_with_vision_server.h
https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/behaviors/src/align_cargo_cargoship.cpp
https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/behaviors/src/align_hatch_panels.cpp


code for listening to and publishing vision goals https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/behaviors/src/align_with_camera.cpp
code full of old unused stuff that is just used for basic functionality of reading from terabee sensors https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/behaviors/src/align_with_terabee.cpp
around line 192 is where all of the auto align things are launched and topic names are setup https://github.com/FRC900/2019RobotCode/blob/base_align_server_dev/zebROS_ws/src/controller_node/launch/2019_compbot_jetson.launch