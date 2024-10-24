##  Joystick Code - Mapping Actions to Buttons ## 

The joystick data processing section of the code is what controls and triggers the various controllers and ActionLib servers. This system allows actions to be triggered by buttons and movements on a joystick rather than typing in the command line.

PLEASE NOTE: This has been updated recently but the code continues to evolve.  Trust the code if it doesn't match the docs...

##  To Learn: ## 

###  Nodes Involved ### 

Three message types are used within this system: `sensor_msgs::Joy` for raw ROS joystick data and `frc_msgs::JoystickState` & `frc_msgs::ButtonBoxState`, as well as several nodes. To make changes within this system, it is necessary to have an understanding of how each of these nodes is used in the joystick code.

####  1. joy/joy_node #### 

The `joy_node` [http://wiki.ros.org/joy](http://wiki.ros.org/joy) is a ROS package that interfaces a Linux joystick with ROS. This node is used in simulation only. It reads values from a connected joystick and publishes them using the `sensor_msgs::Joy` message type. Certain parameters can be set in the launch file to modify how the node functions.

*a. autorepeat_rate* — When no changes are detected in the joystick, messages will be published at the rate specified (Hz). Without this parameter, the node only publishes when it detects a change in the joystick. As a result, if nothing is being moved, no messages are sent. Having a constant stream of messages at a predictable speed is helpful when testing other aspects of the system, since it more closely matches how the real FRC Driver Station works. An `autorepeat_rate` of 100 Hz works well currently.

*b. coalesce_interval* — The counterpart to `autorepeat_rate`, this parameter limits how fast messages may be published. Only one message will be published within the specified interval (seconds). To avoid missing button presses and releases, the `joy_node` will override the `coalesce_interval` when a button change is detected, publishing a message immediately. The default value is 0.001 sec, which can occasionally cause problems with backed up messages (especially on less powerful machines). The `coalesce_interval` is currently set to 0.01 sec. When paired with a 100 Hz `autorepeat_rate`, messages will theoretically always be published at 100 Hz.

*c. deadzone* — Sets an area around 0.0 on the -1.0 to 1.0 scale used for axes, within which the value is considered 0.0. The default value is 0.05. The `teleop_joystick_comp` node does its own `deadzone` processing, so this value is set to 0.0 (disabling `deadzone`) in the `joy_node`.

*d. dev* — Specifies which joystick to read from. Useful when using multiple joysticks. Default is `/dev/input/js0`.

####  2. ros_control_boilerplate/frc_robot_interface #### 

While this node does many other things, when discussing joystick code its function is relatively simple. It takes the joystick data, converts it to our FRC-specific joystick messages and publishes those messages periodically.  On a real robot, the joystick data comes from the FRC Driver station. For sim data, it is read by the sim interface (see below).

This code converts from `sensor_msgs::Joy` messages into `frc_msgs::JoystickState` / `frc_msgs::ButtonBoxState` messages. The `sensor_msgs::Joy` message type lacks certain information that is extremely helpful for FRC, namely button press and release events. The `frc_msgs::JoystickState` / `frc_msgs::ButtonBoxState` message type includes variables for press, release, and hold events. These fields are filled out by the code in `frc_robot_interface`.

####  3. teleop_joystick_control/joystick_remap #### 

A ROS driver for remapping which button corresponds to which array index in a `sensor_msgs::Joy` message and performing mathematical operations on the values of those indices. This remapping is controlled by a `config` file in the `teleop_joystick_control` package called `joystick_remap.yaml`. It is currently used to transform the trigger axes from reading 1.0 when open and -1.0 when closed to 0.0 when open and 1.0 when closed, as well as to rearrange the order of various buttons and axes in the message. This is the [Original GitHub source](https://github.com/ros-drivers/joystick_drivers/blob/master/joy/scripts/joy_remap.py).

####  4. teleop_joystick_control/teleop_joints_keyboard #### 

The `teleop_joints_keyboard` node reads keystrokes from the command line, then generates and publishes a `sensor_msgs::Joy` message. See Keyboard Inputs below for a full list of what keys correspond to what joystick buttons.

####  5. ros_control_boilerplate/frcrobot_sim_interface #### 

The simulation version of the `frc_robot_interface`. It is responsible for listening to `sensor_msgs::Joy` messages and storing them in frc_robot_interface simulated joystick structures.  Once stored, the path through frc_robot_interface out to the rest of the code is identical to a real robot.

There are two simulation configurations. The first has actual gamepad(s) plugged into the laptop running simulation. There, a ROS joy node(s) reads the raw joystick input. That input is run through a remap node and then into `frcrobot_sim_interface`'.

The second configuration takes input from the keyboard, where the keyboard acts as a replacement for a joystick. In this config, output from `teleop_joints_keyboard` is connected to the sim joystick subscriber in `frcrobot_sim_interface`. Since `teleop_joints_keyboard` publishes the same messages as a ROS `joy` node, the code in the subscriber is the same for both cases. The only difference is which node(s) are launched to publish that data.

####  6. teleop_joystick_control/teleop_joystick_comp #### 

This node acts like a control tower for the rest of the code: it subscribes to `rostopics` publishing `frc_msgs::JoystickState` messages, interprets the joystick data, and sends various commands to other nodes, prompting robot actions. Robot actions are mapped to specific buttons and axes, allowing button presses to trigger actions that would otherwise require typing in the command line.

###  Processing Configurations ### 

There are various ways joystick data can be processed, depending on what arguments are passed in while launching code. It is necessary to point out that if an argument is not entered, it will default to a set value. For `hw_or_sim`, the default is `hw`, and for `joy_or_key`, the default is `joy`. The result is that if no arguments are entered, the code will launch in the normal configuration for the physical robot. Many of these configurations are used quite infrequently, but are helpful in specific situations.

####  1. Joystick code on a physical robot ####  

The code supports up to two joysticks plus an optional button box. The button box acts similarly to a joystick, with the main difference being the contents of the message published after `frc_robot_interface` translates it from a raw joystick message.

There are a few variables which control how many / which joystick inputs are enabled :

`joy_or_key:=joy` - the default, enables joysticks vs. keyboard input. Keyboard input should probably never be used on the real hardware, except maybe for cases where we want to trigger a mechanism from a single button press. But generally, use a real gamepad instead. The FRC Driver Station sends joystick data to the `frc_robot_interface`, which publishes the data as an `frc_msgs::JoystickState` message. This message is then read and interpreted by the `teleop_joystick_comp` node, which triggers robot actions. This is the normal way joystick code is run.

`joy_or_key:=two_joy` - enable a second gamepad. The FRC Driver Station sends data from two joysticks to the `frc_robot_interface`,  which publishes them on two separate `frc_msgs::JoystickState` topics. These messages are then both read and interpreted by the `teleop_joystick_comp` node, which triggers robot actions. This has never been used, but it is good to have the option of using multiple joysticks, especially if controlling the robot gets too complicated for a single driver.

`button_box:=true` - Enable the button box.  The path for getting data from this to the teleop node is similar to the above descriptions, but using an `frc_msgs::ButtonBoxState` message instead.

####  2. Joystick code in simulation #### 

In general, all of the options above are supported.  `hw_or_sim:=sim` enables simulation mode, which changes the dataflow slightly, as described above in the `frcrobot_sim_interface` section.  For all non-keyboard configs, the data flow for each message is : The `joy_node` reads data from the connected joystick and publishes a `sensor_msgs::Joy` message to the `joystick_remap` node. After manipulating the message, the `joystick_remap` node publishes to the `frcrobot_sim_interface`, which converts the `sensor_msgs::Joy` message into an `frc_msgs::JoystickState` / `frc_msgs::ButtonBoxState` message. The new message is then published to the `teleop_joystick_comp` node, which interprets the message and triggers actions in the simulation. This is the closest simulation gets to the normal configuration on an actual robot.

Additionally, there's the option to use the keyboard input described in the `teleop_joints_keyboard` section above. Set `joy_or_key:=key`. `teleop_joints_keyboard` reads keyboard inputs and publishes a `sensor_msgs::Joy` message to the `frcrobot_sim_interface`. This node stores it and translates the message into an `frc_msgs::JoystickState` message then publishes it to the `teleop_joystick_comp` node, which interprets the data and triggers actions in the simulation. This configuration is only useful if one wants to test the code's response to a button press without actually having a joystick connected and set up. It is too complicated and impractical for any other use, given the sheer number of different keys needed to simulate a joystick.

The "Release Buttons" function in the table sets the boolean value of all buttons to false. Normally, releasing a button would do this automatically. However, since `teleop_joints_keyboatd` is reading from characters typed into the command line, there is no way to detect when a button has been released. Therefore, the r key is used to release all buttons that had been previously pressed.

| **Axes:**           | **Corresponding Keys:**  |   | **Buttons:**     | **Corresponding Keys:**  |
| Left Stick Y +0.5   | w                        |   | Button A         | 1                        |
| Left Stick Y -0.5   | s                        |   | Button B         | 2                        |
| Left Stick X +0.5   | d                        |   | Button X         | 3                        |
| Left Stick X -0.5   | a                        |   | Button Y         | 4                        |
| Right Stick Y +0.5  | i                        |   | Left Bumper      | e                        |
| Right Stick Y -0.5  | k                        |   | Right Bumper     | u                        |
| Right Stick X +0.5  | l                        |   | Back             | 5                        |
| Right Stick Y +0.5  | j                        |   | Start            | 6                        |
| Left Trigger +0.5   | q                        |   | Left Stick       | x                        |
| Left Trigger -0.5   | z                        |   | Right Stick      | m                        |
| Right Trigger +0.5  | o                        |   | Center Button    | 7                        |
| Right Trigger -0.5  | ,                        |   |                  |                          |
| Dpad Y +1.0         | up arrow                 |   |                  |                          |
| Dpad Y -1.0         | down arrow               |   | Release Buttons  | r                        |
| Dpad X +1.0         | right arrow              |   |                  |                          |
| Dpad X -1.0         | left arrow               |   |                  |                          |


###  Useful Resources ### 

####  1. `2020_compbot_jetson.launch` and `joysticks_sim.launch` #### 

These are two launch files located at `~/2020Offseason/zebROS_ws/src/controller_node/launch`. Reference them when you are writing your launch file. They demonstrate the proper syntax for launch files, as well as show how to launch each of the nodes you will need for this project. Know that understanding launch files will be a great help as you take on more advanced programming tasks.

####  2. `teleop_joystick_comp_2020.cpp` #### 

This is the file you will have to edit to print a string. The mappings for what to do when a button action happens are in a large if/else if block in the code. Throughout the file there are good examples of how to use a `ROS_INFO_STREAM`. This is another good skill to have, as it can be quite useful when testing/debugging code.

####  3. Other Programmers #### 

Feel free to ask for help from more experienced programmers. If the person you ask doesn't know the answer to your question, it will be a good thing for them to learn as well. In particular, ask for Caleb or @Caleb on #ros in Slack, as he designed this curriculum and project.

####  4. Joystick Connection Problems #### 

In the command line, run `ls /dev/input`. If `js0` is in the outputs, your problem is elsewhere and is likely an error in your code. If `js0` is not an output, follow the steps below to resolve the issue.

*a.* — Restart your Docker container with the joystick plugged in.

*b.* — Add `--privileged` to your command to enter the container. The full command should read `docker exec -it --privileged container_name /bin/bash`. 

*c.* — Run `ls /dev/input` again. If `js0` is now in the outputs, the issue is fixed.

The connection problem stems from your Docker container not having the permissions necessary to use USB devices. Adding the `--privileged` tag gives it the permissions it needs. You will need to add the `--privileged` tag every time you want to connect a joystick.

###  Old Config ### 

Saved for historical purposes, also is still the setup for the nanobot example.

###  Nodes Involved ### 

Two message types are used within this system: `sensor_msgs::Joy` and `frc_msgs::JoystickState`, as well as six nodes. To make changes within this system, it is necessary to have an understanding of how each of these nodes is used in the joystick code.

####  1. joy/joy_node #### 

The `joy_node` is a ROS package that interfaces a Linux joystick with ROS. This node is used in simulation only. It reads values from a connected joystick and publishes them using the `sensor_msgs::Joy` message type. Certain parameters can be set in the launch file to modify how the node functions.

*a. autorepeat_rate* — When no changes are detected in the joystick, messages will be published at the rate specified (Hz). Without this parameter, the node only publishes when it detects a change in the joystick. As a result, if nothing is being moved, no messages are sent. Having a constant stream of messages at a predictable speed is helpful when testing other aspects of the system, since it more closely matches how the real FRC Driver Station works. An `autorepeat_rate` of 100 Hz works well currently.

*b. coalesce_interval* — The counterpart to `autorepeat_rate`, this parameter limits how fast messages may be published. Only one message will be published within the specified interval (seconds). To avoid missing button presses and releases, the `joy_node` will override the `coalesce_interval` when a button change is detected, publishing a message immediately. The default value is 0.001 sec, which can occasionally cause problems with backed up messages (especially on less powerful machines). The `coalesce_interval` is currently set to 0.01 sec. When paired with a 100 Hz `autorepeat_rate`, messages will theoretically always be published at 100 Hz.

*c. deadzone* — Sets an area around 0.0 on the -1.0 to 1.0 scale used for axes, within which the value is considered 0.0. The default value is 0.05. The `teleop_joystick_comp` node does its own `deadzone` processing, so this value is set to 0.0 (disabling `deadzone`) in the `joy_node`.

*d. dev* — Specifies which joystick to read from. Useful when using multiple joysticks. Default is `/dev/input/js0`.

####  2. teleop_joystick_control/joystick_remap #### 

A ROS driver for remapping which button corresponds to which array index in a `sensor_msgs::Joy` message and performing mathematical operations on the values of those indices. This remapping is controlled by a `config` file in the `teleop_joystick_control` package called `joystick_remap.yaml`. It is currently used to transform the trigger axes from reading 1.0 when open and -1.0 when closed to 0.0 when open and 1.0 when closed, as well as to rearrange the order of various buttons and axes in the message. This is the [Original GitHub source](https://github.com/ros-drivers/joystick_drivers/blob/master/joy/scripts/joy_remap.py).

####  3. ros_control_boilerplate/frcrobot_hw_interface #### 

While this node does many other things, when discussing joystick code its function is relatively simple. It takes the joystick data received from the FRC Driver Station and publishes it as a `sensor_msgs::Joy` message. As the name suggests, this node is only used when running code on a physical robot.

####  4. ros_control_boilerplate/frcrobot_sim_interface #### 

The simulation version of the `frcrobot_hw_interface`, this node is only used in one processing configuration: Simulation with Keyboard Inputs. In this configuration, the keyboard acts as a replacement for a joystick. The `frcrobot_sim_interface` reads keystrokes from the command line, then generates and publishes a `sensor_msgs::Joy` message. See Keyboard Inputs below for a full list of what keys correspond to what joystick buttons.

####  5. teleop_joystick_control/translate_joystick_data_node #### 

The purpose of this node is to translate `sensor_msgs::Joy` messages into `frc_msgs::JoystickState` messages. Reading the joystick values into `sensor_msgs::Joy` messages is much simpler and allows us to use pieces of code such as the `joy_node` that already exist, instead of taking the time and effort to create our own. However, the `sensor_msgs::Joy` message type lacks certain information that is extremely helpful for FRC, namely button press and release events. The `frc_msgs::JoystickState` message type includes variables for press, release, and hold events. The `translate_joystick_data_node` compares current and previous `sensor_msgs::Joy` messages, detecting changes to fill in the press and release events in the `frc_msgs::JoystickState` messages it publishes.

####  6. teleop_joystick_control/teleop_joystick_comp #### 

This node acts like a control tower for the rest of the code: it subscribes to `rostopics` publishing `frc_msgs::JoystickState` messages, interprets the joystick data, and sends various commands to other nodes, prompting robot actions. Robot actions are mapped to specific buttons and axes, allowing button presses to trigger actions that would otherwise require typing in the command line.

###  Processing Configurations ### 

There are various ways joystick data can be processed, depending on what arguments are passed in while launching code. It is necessary to point out that if an argument is not entered, it will default to a set value. For `hw_or_sim`, the default is `hw`, and for `joy_or_key`, the default is `joy`. The result is that if no arguments are entered, the code will launch in the normal configuration for the physical robot. Many of these configurations are used quite infrequently, but are helpful in specific situations.

####  1. Joystick code on a physical robot ####  

*a. Single Joystick (`hw_or_sim:=hw`, `joy_or_key:=joy`)* — The FRC Driver Station sends joystick data to the `frcrobot_hw_interface`, which publishes the data as a `sensor_msgs::Joy` message. The message is then read, translated, and published as an `frc_msgs::JoystickState` message by the `translate_joystick_data_node`. This message is then read and interpreted by the `teleop_joystick_comp` node, which triggers robot actions. This is the normal way joystick code is run.

![](../../wiki-resources/programming/physical_robot_single_joystick_horizontal_version.png)

*b. Two Joysticks (`hw_or_sim:=hw`, `joy_or_key:=two_joy`)* — The FRC Driver Station sends data from two joysticks to the `frcrobot_hw_interface`, which publishes the data as `sensor_msgs::Joy` messages on two separate `rostopics`. The messages are then read, translated, and published as `frc_msgs::JoystickState` messages by two instances of the `translate_joystick_data_node`. These messages are then both read and interpreted by the `teleop_joystick_comp` node, which triggers robot actions. This has never been used, but it is good to have the option of using multiple joysticks, especially if controlling the robot gets too complicated for a single driver.

![](../../wiki-resources/programming/physical_robot_two_joysticks_horizontal_version.png)

####  2. Joystick code in simulation #### 

*a. Single Joystick (`hw_or_sim:=sim`, `joy_or_key:=joy`)* — The `joy_node` reads data from the connected joystick and publishes a `sensor_msgs::Joy` message to the `joystick_remap` node. After manipulating the message, the `joystick_remap` node publishes to the `translate_joystick_data_node`, which converts the `sensor_msgs::Joy` message into an `frc_msgs::JoystickState` message. The new message is then published to the `teleop_joystick_comp` node, which interprets the message and triggers actions in the simulation. This is the closest simulation gets to the normal configuration on an actual robot.

![](../../wiki-resources/programming/simulation_single_joystick.png)

*b. Two Joysticks (`hw_or_sim:=sim`, `joy_or_key:=two_joy`)* — Two instances of the `joy_node` read data from two connected joysticks and publish `sensor_msgs::Joy` messages to two copies of the `joystick_remap` node. After manipulating the message, the `joystick_remap` nodes publish to two instances of the `translate_joystick_data_node`, which convert the `sensor_msgs::Joy` messages into `frc_msgs::JoystickState` messages. The new messages are then published to the `teleop_joystick_comp` node, which differentiates between the messages, interprets them, and then triggers actions in the simulation. This is the simulation counterpart to using two joystick on a physical robot.

![](../../wiki-resources/programming/simulation_two_joysticks.png)

*c. Keyboard Inputs (`hw_or_sim:=sim,` `joy_or_key:=key`)* — The `frcrobot_sim_interface` reads keyboard inputs and publishes a `sensor_msgs::Joy` message to the `translate_joystick_data_node`. This node translates the message into an `frc_msgs::JoystickState` message and publishes it to the `teleop_joystick_comp` node, which interprets the data and triggers actions in the simulation. This configuration is only useful if one wants to test the code's response to a button press without actually having a joystick connected and set up. It is too complicated and impractical for any other use, given the sheer number of different keys needed to simulate a joystick.

![](../../wiki-resources/programming/simulation_keyboard_inputs.png)

The "Release Buttons" function in the table sets the boolean value of all buttons to false. Normally, releasing a button would do this automatically. However, since the `frcrobot_sim_interface` is reading from characters typed into the command line, there is no way to detect when a button has been released. Therefore, the r key is used to release all buttons that had been previously pressed.

| **Axes:**           | **Corresponding Keys:**  |   | **Buttons:**     | **Corresponding Keys:**  |
| Left Stick Y +0.5   | w                        |   | Button A         | 1                        |
| Left Stick Y -0.5   | s                        |   | Button B         | 2                        |
| Left Stick X +0.5   | d                        |   | Button X         | 3                        |
| Left Stick X -0.5   | a                        |   | Button Y         | 4                        |
| Right Stick Y +0.5  | i                        |   | Left Bumper      | e                        |
| Right Stick Y -0.5  | k                        |   | Right Bumper     | u                        |
| Right Stick X +0.5  | l                        |   | Back             | 5                        |
| Right Stick Y +0.5  | j                        |   | Start            | 6                        |
| Left Trigger +0.5   | q                        |   | Left Stick       | x                        |
| Left Trigger -0.5   | z                        |   | Right Stick      | m                        |
| Right Trigger +0.5  | o                        |   | Center Button    | 7                        |
| Right Trigger -0.5  | ,                        |   |                  |                          |
| Dpad Y +1.0         | up arrow                 |   |                  |                          |
| Dpad Y -1.0         | down arrow               |   | Release Buttons  | r                        |
| Dpad X +1.0         | right arrow              |   |                  |                          |
| Dpad X -1.0         | left arrow               |   |                  |                          |

##  To Do: ## 

In a new branch, write a launch file to launch the proper nodes for a single joystick in simulation. Add remaps for the proper `rostopics` and edit `teleop_joystick_comp.cpp` to print out a string (with a `ROS_INFO_STREAM`) when a certain button is pressed. A finished project will do something like this: after running the launch file with a joystick connected, pressing button X causes "Hello World!" to be printed on the screen. When you have finished, ask for Caleb at a meeting and show him the end result.

###  Useful Resources ### 

####  1. `2019_compbot_jetson.launch` and `joysticks_sim.launch` #### 

These are two launch files located at `~/2019Offseason/zebROS_ws/src/controller_node/launch`. Reference them when you are writing your launch file. They demonstrate the proper syntax for launch files, as well as show how to launch each of the nodes you will need for this project. Know that understanding launch files will be a great help as you take on more advanced programming tasks.

####  2. `teleop_joystick_comp.cpp` #### 

This is the file you will have to edit to print a string. The mappings for what to do when a button is pressed/held/released are located from lines 272-581. Pick one of these buttons to edit. Throughout the file there are good examples of how to use a `ROS_INFO_STREAM`. This is another good skill to have, as it can be quite useful when testing/debugging code.

####  3. Other Programmers #### 

Feel free to ask for help from more experienced programmers. If the person you ask doesn't know the answer to your question, it will be a good thing for them to learn as well. In particular, ask for Caleb or @Caleb on #ros in Slack, as he designed this curriculum and project.

####  4. Joystick Connection Problems #### 

In the command line, run `ls /dev/input`. If `js0` is in the outputs, your problem is elsewhere and is likely an error in your code. If `js0` is not an output, follow the steps below to resolve the issue.

*a.* — Restart your Docker container with the joystick plugged in.

*b.* — Add `--privileged` to your command to enter the container. The full command should read `docker exec -it --privileged container_name /bin/bash`. 

*c.* — Run `ls /dev/input` again. If `js0` is now in the outputs, the issue is fixed.

The connection problem stems from your Docker container not having the permissions necessary to use USB devices. Adding the `--privileged` tag gives it the permissions it needs. You will need to add the `--privileged` tag every time you want to connect a joystick.