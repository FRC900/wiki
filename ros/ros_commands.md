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
    ... lots more text ...
    ```

### [`rostopic`](#rostopic)
- **Description**: topic-related ROS commands
- **Usage**:
    - `rostopic list`: lists all topics being published to
    - `rostopic echo [topic]`: prints out all messages being published to `[topic]`
    - `rostopic info [topic]`: prints out information about a topic
    - `rostopic pub [topic] [message type] [message]`: publishes `[message]` (of type `[message type]`) to `[topic]`
      - Optional argument: `-r [frequency]` publishes the message continuously at `[frequency]` Hz
    - `rostopic hz [topic]`: calculates the publish rate (in <ins>Hz</ins>) of `[topic]`
- **Example**:
    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rostopic list
    /align/cmd_vel
    /auto/auto_mode
    /auto/cmd_vel
    ... lots more topics ...
    ```

    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rostopic echo /imu/zeroed_imu
    header: 
    seq: 24
    stamp: 
        secs: 0
        nsecs: 690000000
    frame_id: "imu"
    orientation: 
    x: -0.00038022196806286243
    y: 0.0001624124748701506
    z: 0.9999998834053025
    w: -0.0002494851185626063
    ... more message data ...
    ---
    ... next message ...
    ```

    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rostopic info /auto/auto_mode
    Type: behavior_actions/AutoMode

    Publishers: None

    Subscribers: 
    * /candle_node (http://192.168.227.195:46065/)
    ```

    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rostopic pub /auto/auto_mode behavior_actions/AutoMode "header:
    seq: 0
    stamp:
        secs: 0
        nsecs: 0
    frame_id: ''
    auto_mode: 5" 
    publishing and latching message. Press ctrl-C to terminate
    ```

    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rostopic hz /imu/zeroed_imu 
    subscribed to [/imu/zeroed_imu]
    average rate: 35.417
        min: 0.000s max: 0.060s std dev: 0.01774s window: 35
    average rate: 32.292
        min: 0.000s max: 0.060s std dev: 0.01764s window: 63
    ```

### [`rosservice`](#rosservice)
- **Description**: service-related ROS commands
- **Usage**:
    - `rosservice list`: lists all services being advertised
    - `rosservice info [service_name]`: prints out information about a service
    - `rosservice call [service_name] [arguments]`: calls the service advertised at `service_name` with `arguments` as the request data
- **Example**:
    ```bash
   ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rosservice list
    /auto/stop_auto
    /candle_node/set_leds_green
    /frcrobot_jetson/swerve_drive_controller/brake
    /frcrobot_jetson/swerve_drive_controller/change_center_of_rotation
    /imu/set_imu_zero
    ... lots more services ...
    ```

    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rosservice info /imu/set_imu_zero 
    Node: /imu/imu_zero
    URI: rosrpc://192.168.234.97:39273
    Type: imu_zero_msgs/ImuZeroAngle
    Args: angle
    ```

    ```bash
    ubuntu@bean-ThinkPad-X1-Yoga-Gen-6:~$ rosservice call /imu/set_imu_zero "angle: 0.0"
    (if the service had a response, it would be shown here)
    ```

### [`rosnode`](#rosnode)
- **Description**: node-related ROS commands
- **Usage**:
    - `rosnode list`: lists all topics being published to
    - `rosnode info [node]`: prints out info about `[node]`
    - `rosnode kill [node]`: kills a given node 
    - `rosnode machine [IP]`: prints out all nodes on a given machine IP, if run without IP will print current machine IP

- **Example**:
    ```bash
    ubuntu@dino-desktop:~$ rosnode list
    /align_and_place_grid
    /align_to_grid_closed_loop/align_to_grid_closed_loop_2023
    /align_to_grid_closed_loop/x_position_pid
    /align_to_grid_closed_loop/y_position_pid
    ```

    ```bash
    ubuntu@dino-desktop:~$ rosnode info /align_and_place_grid
    --------------------------------------------------------------------------------
    Node [/align_and_place_grid]
    Publications:
     * /align_and_place_grid/feedback [behavior_actions/AlignAndPlaceGrid2023ActionFeedback]
     * /align_and_place_grid/result [behavior_actions/AlignAndPlaceGrid2023ActionResult]
    ...
    
    Subscriptions:
     * /align_and_place_grid/cancel [actionlib_msgs/GoalID]
     * /align_and_place_grid/goal [behavior_actions/AlignAndPlaceGrid2023ActionGoal]
    ...
    Services:
     * /align_and_place_grid/get_loggers
     * /align_and_place_grid/set_logger_level
    
    contacting node http://10.50.36.231:35239/ ...
    Pid: 452
    Connections:
     * topic: /rosout
        * to: /rosout
        * direction: outbound (48389 - 10.50.36.231:47566) [11]
        * transport: TCPROS
     * topic: /align_to_grid_closed_loop/align_to_grid_closed_loop/goal
        * to: /align_to_grid_closed_loop/align_to_grid_closed_loop_2023
        * direction: outbound (48389 - 10.50.36.231:47598) [10]
        * transport: TCPROS
    ```

    ```bash
    ubuntu@dino-desktop:~$ rosnode kill /align_and_place_grid
    killing /align_and_place_grid
    killed
    ```

    ```bash
    ubuntu@dino-desktop:~$ rosnode machine
    10.50.36.231
    ubuntu@dino-desktop:~$ rosnode machine 10.50.36.231
    /align_to_grid_closed_loop/align_to_grid_closed_loop_2023
    /align_to_grid_closed_loop/x_position_pid
    /align_to_grid_closed_loop/y_position_pid
    ... many more
    ```

### [`roswtf`](#roswtf)
- **Description**: a ROS self-test tool, checks for errors
- **Usage**: `roswtf`

[Home](/README.md)
