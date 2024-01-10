Stage is a relatively simple simulator. While not part of ROS, there is a package which interfaces with it called stage_ros.  This works with stage to provide standard topics to publish and subscribe to.  For example, the robot simulated by stage_ros reads a /cmd_vel topic to move it. And it reports back odom data.

Stage can be used to create a simulation of the field.  Typically a png file is generated to define the field boundaries.  See 2020RobotCode/field_map for opencv code which generates the field.

Additionally, vision targets can be added to the simulated field.  These are called fiducial markers. This normally refers to things like AR or VR tags. It also fits our use case.  Both retro and tensorflow object detection ID a particular target at a given location, just like an AR/VR/QR tag detector might.  

We have a branch of the stage_ros simulator in our repo (currently part of a PR, will be merged eventually) which publishes fiducial target data on the /base_marker_detection topic.  This is a standard ros marker_msgs/MarkerDetection message type - http://docs.ros.org/melodic/api/marker_msgs/html/msg/MarkerDetection.html

The sim environment for stage is created using text files.  These define objects of various types and place them in the simulated world.

A robot is defined in stage as a "position" object - http://rtv.github.io/Stage/group__model__position.html.  See the example in zebROS_ws/controller_node/stage/robots/2020FRC_robot.inc.  This example defines a simple omnidirectional robot which has a single fiducial sensor on it.  That should be enough to to basic testing of the robot, but feel free to improve it as necessary.

A world file is the top level definition for a simulation environment.  Ours is in zebROS_ws/controller_node/stage/2020FRC_robot.world.  This is the file which pulls in a png file to create the basic map. It also creates an instance of a robot, using the definition from the previous paragraph. Finally, it creates a number of object detection targets, using a macro defined to create fiducial targets.

stage_ros is a single node and it takes a single argument - the top level world file. See controller_node/launch/FRC2020robot_in_stage.launch.

This launch file now includes various nodes to translate from the data published by stage_ros into the correct topics needed for PF localization.


