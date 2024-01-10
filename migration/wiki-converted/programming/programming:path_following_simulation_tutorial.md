##  Nodes Used for Pathing ## 

This section gives a quick overview for each of the nodes used in the path planning and execution code.

###  Path Generator Overview ### 

The input to the path generator is a set of waypoints for the robot to drive through.  Each waypoint is an (x, y, orientation) that the robot must drive through in sequence. The path generation code finds a smooth, quick path through these waypoints.  It outputs a path, which is a sequence of timestamped positions the robots should follow.  That is, each position in the path has a time the robot should arrive at each position.  

A much more detailed description of the path generation code is in https://wiki.team900.org/doku.php?id=programming:control_systems_programming
  
###  Path Follower Overview ### 

The path follower code takes a path from the path generator and generates commands to the robot drive base to actually follow this path.  The path follower keeps track of the current time since the path was started and looks up where the robot should be at that time.  It uses position PID controllers to generate velocities which should drive the robot to the desired position.

The code for this node is at https://github.com/FRC900/2022RobotCode/tree/master/zebROS_ws/src/path_follower/src

###  Auto Node Overview ### 

The auto node runs sequences of configurable automated tasks.  In our case, it is just going to trigger sending a path to the path follower as soon as the robot is enabled.  But it is more powerful than that.  For example, a sequence of events could be something like, drive back 1 meter, then fire balls at a target, then activate the input, then drive a path which (hopefully) moves the intake over a set of game pieces.

###  Particle Filter Localization Overview ### 

The particle filter code (PF) is used to figure out where the robot is on a pre-defined fixed map.  It uses visual beacons and attempts to locate where the robot is based on the relative positions of those beacons from the robot's cameras.  

###  Stage Sim Overview ### 

Stage is a simple robot simulator.  It takes simulated robot inputs and generates simulated robot motion and sensor inputs of the user-defined world the robot lives in.  For example, we can send simulated drive base commands, and the robot moves. Stage reports back odometry, which can be fed back into our path following code.  Stage can also report simulated detection of objects in the field, useful for testing things like our particle filter.

###  Rviz overview ### 

Rviz is a ROS tool for visualizing the data published to various topics. In this case, we can use it to view a graphical representation of a path relative to a map. It can also visualize robot odometry, allowing us to see where the simulated robot is.

##  Running Path Following Simulation ## 

Open up a few terminal windows in docker.  Easiest way is to get into a single docker terminal window, using either docker-run if starting from scratch, or docker restart/docker attach to connect to an existing session.  Then, run `terminator &` to bring up a separate terminal window running inside the container.  From there, create new tabs (ctrl-shift-t) or use the right-mouse menu to split the window horizontally and vertically.

In each window, `source ~/2022RobotCode/zebROS_ws/ROSStandard.sh`.

In one window, use `roslaunch controller_node 2022_sim.launch` to start up the simulator and the pathing nodes listed in the previous section.

The stage sim window will appear.  The origin of the map is at 0,0 in the lower left corner, with a little bit of extra margin added in case the robot runs slightly off the field.   Use the left mouse button to drag the map around and the mouse wheel to zoom in and out.  The red box is our simulated robot.  This can be dragged around using the left mouse button.  The right mouse button can be used to rotate the simulated robot.

Stage sim publishes an odom topic which indicated the current position of the robot.  This topic has been remapped to the topic name from the actual robot - /frcrobot_jets/swerve_drive_controller/odom.  By remapping it to the actual topic used on the robot, we can run the rest of the robot code as if it were on a real robot instead of having to modify it specifically for simulation work.

Watch how odom works.  In a command window, run `rostopic echo /frcrobot_jets/swerve_drive_controller/odom` and watch how it changes while dragging the robot around.  This topic is used by the path following code as an input to the PID controllers driving the robot, simulating the output of the odometry calculated by the real robot.

In addition to being dragged around using the mouse, the simulated robot can be driven using a Twist message. This message contains commanded linear and rotational velocity for the robot to drive.  Type `rostopic pub -r 10 /teleop/swerve_drive_controller/cmd_vel` then hit tab a few times to fill in a template for the message.  Set either x or y (or both), hit enter, and watch the robot move.

This topic is published to from the path following code.  Each of the outputs of a PID controller is a velocity in one of 3 directions (x, y, orientation).  A separate node combines all three into a single Twist message which is published to stage (in simulation) or to the robot's drive base controller (in real hardware). The exact same code produces the exact same message in either case, and it is up to the simulator or real hardware to do what it takes to move their version of the robot in response.

Next up, we're going to see how the auto node runs paths.  Open the file ~/2022RobotCode/zebROS_ws/src/behaviors/config/auto_mode_config.yaml in your favorite editor.  This config file defines all of our autonomous sequences.  At the top of the file is a list of auto_mode_- arrays. We run one of these auto modes each time the robot is enabled in autonomous mode, and have a variety defined either for testing or for different match strategies.  Each entry in each of the auto_mode_- arrays is a text label which refers to an action defined later on in the config file.

Each action definition has a type and a goal. The goal typically contains additional data needed to run the action.  For a path type, for example, it is a list of waypoints to drive to. 

We're going to add a new path to follow. First off, make an auto_mode_6 entry near the top of the file.  The only entry in the auto_mode_6 array def should be the string "drive_path_test_1". This label can be anything, it just has to refer to a named section further down in the file.  Create this section later in the file

```
drive_path_test_1:
    type: "path"
    goal:
        apply_offset: false 
        points:
            - [1.0, 0.0, 0.0]
            - [2.0, 4.0, 0.0] 
            - [4.0, 4.0, 0.0] 
```

Indentation is important, and make sure to use spaces.

The first line is a label, the same as the name of the label in the auto_mode_- array we defined.  The type is path, which means drive a path relative to the robot.  The goal contains the path data. apply_offset can be used to shift the path left or right - we used this to adjust the path based on where the robot is placed on the field.  Finally, the points array is an array of [x, y, offset] points to drive through in order.

You'll need to restart the auto node to pick up these changes.  Run `roslaunch behaviors auto_node.launch` to reload config variables and restart the auto node.  You can also call the stage_ros reset_position service to return the robot to its starting position : `rosservice call /reset_positions "{}" `.

You'll have noticed that in addition to the stage simulator, another gui window was started by the launch file. This is a simulated driver station. We can use it to tell our code that the robot has been enabled, and it can also set various information needed to run auto routines.

In our case, we need to set Auto Mode to 6 so that the auto_node code uses our new auto_mode_6 definition.  Change the Auto Mode field from the default of 1 to 6.  Then set the mode to autonomous (upper left corner) and click Enable in the lower left. The robot should drive the path we defined.

###  Visualizing paths with rviz ### 

Run the rviz tool from the command line using `rviz`.  A gui will launch.  In the lower right, select Add. In the window that pops up, select the "By Topic" tab, and scroll down to the map topic. Select it, and the map should appear in rviz. Repeat the process, but this time in the "By Topic" tab select local_plan->path.  This will display the most recently generated path overlaid on top of the map.

###  Selecting a new map ### 

Two files need to be edited to load a new map, one for the stage sim, and a second for rviz.  The stage sim config file is `2022RobotCode/zebROS_ws/src/controller_node/stage/2022_rapid_react.world`. In this file search for the floorplan section and change both the name and bitmap to match the desired map.  To change the map loaded by rviz, look for the map_server command in `2022RobotCode/zebROS_ws/src/controller_node/launch/2022_sim.launch`.  Update the yaml file to point to the desired map.

###  Changing robot starting position ### 

The same two files need to be edited to change the robot's starting position.  In 2021PathFollower.world, edit the pose section of the omnidir_robot section. The first two numbers in the pose list are x and y starting coords, in meters, from the origin of the map.

A similar edit needs to be run on the static_maptobase line in 2022_sim.launch. Again, the first two numbers in the args list are the starting x and y coordinates of the robot.

