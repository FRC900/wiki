## **ROS-Related Commands**

### [`rosstd`, `rosjet`](#rosstd-rosjet)
- **Description**: aliases to set up/source ROS environment
- **Usage**:
    - `rosstd` is for running robot code on your laptop
    - `rosjet` is for running robot code on the robot/Jetson

### [`hwlaunch`](#hwlaunch)
- **Description**: alias to run robot code (to be executed on the Jetson)
    - if you're curious, `alias hwlaunch='roslaunch controller_node 2023_compbot_combined.launch'`
- **Usage**: `hwlaunch` after you've sourced ROS with `rosjet`

### [`roscore`](#roscore)
- **Description**: the ROS core process/master, this is what coordinates communication between nodes
- **Usage**: `roscore -p 5802 &`
    - FRC teams are restricted to network ports `5800` to `5810`, and the default ROS port is `11311` which is out of that range. That's why we use port `5802`.
    - The `&` at the end runs the process in the background (so you can do other things in your terminal)
- **Example**:
    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ roscore -p 5802 &
    [1] 50905
    ... lots of debug messages ...
    ```

### [`rosrun`](#rosrun)
- **Description**: runs a ROS node
- **Usage**: `rosrun [package] [node]`
    - This runs the ROS node `[node]` in package `[package]`.
- **Example**:
    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rosrun rviz rviz # runs rviz (which is in a package of the same name), the ROS visualization tool
    QStandardPaths: XDG_RUNTIME_DIR not set, defaulting to '/tmp/runtime-ubuntu'
    [INFO] [1693006625.121936170]: rviz version 1.14.20
    [INFO] [1693006625.122026138]: compiled against Qt version 5.15.3
    [INFO] [1693006625.122052834]: compiled against OGRE version 1.9.0 (Ghadamon)
    [INFO] [1693006625.137169295]: Forcing OpenGl version 0.
    [INFO] [1693006625.606206014]: Stereo is NOT SUPPORTED
    [INFO] [1693006625.606308147]: OpenGL device: Mesa Intel(R) Xe Graphics (TGL GT2)
    [INFO] [1693006625.606338795]: OpenGl version: 4.6 (GLSL 4.6) limited to GLSL 1.4 on Mesa system.
    ```

### [`roslaunch`](#roslaunch)
- **Description**: runs a ROS node
- **Usage**: `roslaunch [package] [file.launch]`
    - This runs the launch file `[file.launch]` in package `[package]`.
- **Example**:
    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ roslaunch controller_node 2023_sim.launch # run our robot in a simulated field
    ... logging to /home/ubuntu/.ros/log/18ec89f2-43a0-11ee-a629-6e8188a52783/roslaunch-bean-ThinkPad-X1-Yoga-Gen-6-51011.log
    Checking log directory for disk usage. This may take a while.
    Press Ctrl-C to interrupt
    Done checking log file disk usage. Usage is <1GB.

    started roslaunch server http://192.168.162.15:44361/

    SUMMARY
    ========

    ```



### [`rostopic`](#rostopic)
- **Description**: topic-related ROS commands
- **Usage**: TODO
- **Example**:
    TODO