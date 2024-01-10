#Running SLAM on ROS#

==== Step 1: ====

Open Terminator and split it into at least 5 windows.

==== Step 2: ====

On each window change your directory to `%%2017VisionCode/zebROS_ws/%%`.

Then type:

```bash
. ./ROSStandard.sh
```
==== Step 3: ====

Then in the first window start `%%roscore%%`.

==== Step 4: ====

In the second window run to run the NavX:

```bash
rosrun navx_publisher navx_publisher_node
```
==== Step 5: ====

In the third window run to run ZED:

```bash
roslaunch zed_wrapper zed.launch brightness:=300 exposure:=100
```
==== Step 6: ====

In the fourth window run to run convert ZED frames to robot frames (in ROS frames are used for coordinates 1. http:%%*%%wiki.ros.org/tf 1. contains more information about frames):

```bash
rosrun tf static_transform_publisher 0 0 0 -1.5707963267948966 0 -1.5707963267948966 camera_link zed_initial_frame 100
```
==== Step 7: ====

In the fifth window run to run rtabmap (the mapping program 1. http:%%*%%wiki.ros.org/rtabmap 1. more information on rtabmap):

```bash
roslaunch rtabmap_ros stereo_mapping.launch stereo_namespace:=/zed right_image_topic:=/zed/right/image_rect_color visual_odometry:=false odom_topic:=/navx/odom frame_id:=camera_link approx_sync:=true rtabmap_args:="--Vis/CorFlowMaxLevel 5 --Stereo/MaxDisparity 200"
```
NOTE: `%%visual_odometry:=true%%` When you run it as true it runs localization.

NOTE: `%%visual_odometry:=true%%` When you run it with false it uses the NavX for localization and just runs mapping.
