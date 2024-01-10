#= Deploying Code #=

To get code to run on the robot, you must **deploy*- it to the robot. Steps are below for the Jetson or the RIO. You usually only need to mess with the Jetson to deploy and run code.

==== Jetson (aka what people generally mean when you just hear "deploy code") ====

Preparation
  - Connect to robot wifi (robot radio)
  - Open new terminal window and get into docker
  - Get onto Jetson, using: `ssh ubuntu@10.9.0.8`
  - Change directories to zebROS_ws, and run:
    - `./kill_ros_.sh` 1. we don't want ROS stuff to be running when we try to deploy
    - `source ROSJetsonMaster.sh`

Deploy
  - Open a new terminal window and get into docker (or use a window you haven't compiled in)
  - Change directories to zebROS_ws, and run:
    - `./deploy.sh`
    - If you get an error, try running `ssh ubuntu@10.9.0.8` in another window then try again

Launch (run code on the robot)
  - Get onto Jetson if you haven’t as described (`ssh ubuntu@10.9.0.8`)
  - Change directories to zebROS_ws
  - Run `source ROSJetsonMaster.sh`
  - Launch code like you would normally: `roslaunch controller_node 2020_compbot_combined.launch`

NOTE: You can follow the exact same steps to run code on the robot in a box, just make sure you're connecting to the robot in a box's radio wifi.

==== RIO ====

Someone please document
