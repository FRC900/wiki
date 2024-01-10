UPDATE AND FINISH
#= How to rosbag #=

# Index #

  - [[#recording-and-replaying-a-rosbag|Recording and Replaying a rosbag]]
    - [[#where-to-save-bags|Where to save bags]]
  - [[#replaying-a-rosbag-in-a-script|Replaying a rosbag In a Script]]
  - [[#accuracy-of-a-rosbag|Accuracy of a rosbag]]

# Recording and Replaying a rosbag #

  - [[http:*wiki.ros.org/rosbag/Tutorials/Recording%20and%20playing%20back%20data|Tutorial on ROS Wiki]]

==== Where to save bags ====

In the home directory of 2018RobotCode there is `%%bagfiles/%%` and `%%bagfiles_temp/%%`. The former is for bags to be used in competition; the latter is for temporary bags to be stored. Bagfiles in `%%bagfiles_temp/%%` will not be committed.


----

Rosbagging will create a file in your current directory. It is recommended to create a temporary directory if you will be creating multiple bags.

  1. Start the roslaunch/run/core that you wish to record.
  1. `%%rostopic list%%` to view all of the available topics.
  1. If recording all topics (becomes a large file quickly):
    - `%%rosbag record -a%%`
    - This will record all of the activity in every topic that is active during the recording; topics with no data published to them will be ignored.
    - This is generally bad.  For example, it will cause every topic from the ZED to be recorded 1. both camera's images, compressed images, point cloud data, etc. Instead, use the following :
  1. If recording specific topics:
    - `%%rostopic record -O subset <TOPIC1> <TOPIC2> ...%%`
  1. To view information associated with a particular bag: `%%rosbag info /path/to/rosbag.bag%%`
  1. To replay a rosbag (**This will execute every movement that happened during the recording!**):
    - `%%rosbag play /path/to/bag%%`

# Replaying a rosbag In a Script #

TODO

# Accuracy of a rosbag #

The paths tracked by some nodes are very sensitive to small changes in timing in the system, and rosbag is limited in its ability to exactly duplicate the behavior of a running system. For nodes like turtlesim, where minor timing changes in when command messages are processed can subtly alter behavior, the user should not expect perfectly mimicked behavior.
