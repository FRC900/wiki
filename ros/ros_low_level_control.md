### **−**Table of Contents

*   [ROS Robot Joints](#ros-robot-joints)

*   [ROS controllers](#ros-controllers)
    
*   [ROS control manager](#ros-control-manager)
    
*   [ROS hardware interface](#ros-hardware-interface)
    
*   [ROS top-level control node](#ros-top-level-control-node)
    
*   [Actual implementation](#actual-implementation)
    
*   [Hardware state data](#hardware-state-data)
    
*   [Talon HW Commands](#talon-hw-commands)
    
*   [Registering Interfaces](#registering-interfaces)
    
*   [Standard ROS Interfaces](#standard-ros-interfaces)
    
*   [Interface vs. Controller](#interface-vs-controller)
    
*   [Talon Controller / Controller Interface Code](#talon-controller--controller-interface-code)
    
*   [Resources](#resources)
        
This page describes the ideas behind the hardware interface and ROS low level controllers. This is the code which is in / used by `ros\_control\_boilerplate`. It is the very lowest level of code - that which talks directly with the hardware, and the stuff which operates just a level above to generate commands to send to the hardware.

### ROS Robot Joints

In ROS-speak, each thing which can move on a robot is called a joint. While we typically use motors to drive wheels, ROS control spends more time worrying about the harder process of controlling arms. These arms have various joints, so the name stuck. In our case, a wheel is also a joint. Or for a swerve wheel, two joints - a steering joint and a speed joint - but that is handled by higher level code which combines two joints into a single wheel.

Note that joints can also be inputs or outputs on the Rio. For example a digital output or solenoid is a bit of hardware which can change, so we create joints mapped to them. Setting the value of these joints triggers changes in the output of the connected hardware. While it seems even less correct than to call a wheel a joint, it does let us reuse a lot of standard ROS code … we'll happily trade effort for a weird naming convention.

In our setup, we define joints in a YAML config file. Every joint on the robot is listed in an array at the top of our main config file. This defines the name, type of joint, joint properties, etc for each.

[Joints](http://wiki.ros.org/urdf/XML/joint) can also be defined in ROS using a URDF (Universal Robot Definition File) file. A joint definition has a name, a type (e.g. continuous or planar), optional movement limits and also parameters for simulation physics. This isn't used heavily by our code yet, but the hope is to eventually allow us to generate a list of joints from a CAD file or similar.

### ROS controllers

A ROS controller is code which exposes joint data from the hardware interface to other ROS nodes. This could mean it reads joint state and publishes that on a topic (typically called a state controller). Or it takes commands published to a topic by another node and uses that to command joints.

A controller can be simple and control just a single joint. Or they could coordinate multiple joints working together along with sensors to control a more complex mechanism. It is hard to put a specific rule on how mechanisms should be grouped into controllers, but in general, if we can reasonably control a system with a simpler setup (i.e. with fewer things grouped into a controller) we should use that approach.

These are written as Linux shared libraries and are loaded dynamically using a ROS command. Multiple control libraries can be loaded at once. Basically, this means that the set of controllers running can be dynamically controlled by launch files. This is useful in our case for debugging and bring up. For example, we could load a simple controller to make sure each wheel moves. Then, when we are satisfied the basics are working, a single controller running all the wheels in a coordinate fashion could replace the individual test controllers. This would just require a change in the launch file.

### ROS control manager

ROS's control\_manager is used to manage access to hardware (motor controllers, encoders, etc). It loads shared libraries containing controllers for particular sets of hardware. These libraries can control things like the drive base (multiple wheels) or a shooter (again, multiple wheels plus a hood) or an intake (just one motor).

When each library is loaded and the controller from that library is started, the control\_manager makes sure the hardware resources needed for the new controller are available and if so, allocates them to that particular controller. This prevents multiple controllers from trying to write to the same hardware.

This is a stock ROS node, so there's not much to do here other than run it with the correct list of controllers.

### ROS hardware interface

The controller\_manager isn't a node in itself. Instead, it is code pulled into a node. This code is normally combined with a so-called ROS hardware interface. The hardware interface is the code which translates between commands to a named joint and the actual code which drives the corresponding hardware. This includes keeping track of the current state (encoder value, input level, etc) of the joint and the next value to write.

Those values are kept in various arrays. Each type of joint has an array of values, and each entry is the value for one particular joint of that type.

This is also where the code which actually reads encoder values from the hardware and writes motor position/speed commands to the Talon, using methods called read() and write().

### ROS top-level control node

We'll call the node containing a ROS hardware interface plus a controller manager the top-level control node.

When this node is started, it runs through the list of hardware defined in the config file and initializes it. This initialization just puts it in a known state.

The launch file then commands it to load a list of controllers. Each controls a subset of the hardware doing a single task (drive base, arm, shooter, intake, analog/digital IO pins, etc). On initialization the library gets \_handles\_ for each of the joints it needs to run. Handles are created by name. These handles could be read only, which allow the library to get at the stored state values for that joint. They may also be read/write, allowing the controller to send commands to the hardware. For the read/write case, the code enforces that only one read/write handle is allowed per joint.

By accessing joints by name, the code abstracts away the need for a controller library to know exactly what hardware it is controlling. For example, we routinely move around CAN IDs, digital IO port numbers, and so on as the robot hardware evolves. The mapping from joint name to these IDs is in one place - the YAML config files. Changing them there automatically makes the controller access the updated joint information without changing any other code.

In more extreme cases, we could change between different types of motors or encoders, or maybe switch from an analog to digital output to trigger some hardware. If the controller library writes using real-world values of angle (radians) or velocity (meters/sec) it can stay the same … so long as the hardware interface converts between those real world units and the commands needed for a given piece of actual hardware.

The initialization also typically starts publishers and/or subscribers in the controller. A subscriber might listen for commands instructing the controller library to move the joint(s) it controls. A publisher might periodically read the state of some joints and publish them as a ROS message so that other nodes could react to those values.

This will be under a namespace named after the control library. For instance, a swerve base would be under /frcrobot\_jetson/steered\_wheel\_base\_controller/cmd\_vel. This way a library can just say it wants to subscribe to cmd\_vel and make sure that there's no conflict with another separate library (or separate copy) that wants to subscribe to the same message.

The top level loop in a controller looks like this ([https://github.com/FRC900/ros\_control\_boilerplate/blob/4cae3deaa4b463263b18d226e8518ae12c9d8808/src/generic\_hw\_control\_loop.cpp#L89](https://github.com/FRC900/ros_control_boilerplate/blob/4cae3deaa4b463263b18d226e8518ae12c9d8808/src/generic_hw_control_loop.cpp#L89 "https://github.com/FRC900/ros_control_boilerplate/blob/4cae3deaa4b463263b18d226e8518ae12c9d8808/src/generic_hw_control_loop.cpp#L89")) :

```c++
// Read HW state from encoders
hardware\_interface\_->read(elapsed\_time\_);
// Control HW, generate another set of commands for motors
controller\_manager\_->update(ros::Time::now(), elapsed\_time\_);
// Output HW state to motors
hardware\_interface\_->write(elapsed\_time\_);
```

As mentioned, the hardware interface is responsible for actually reading and writing to the hardware. In our case the read() will get, e.g. encoders connected to TalonSRXs, line break sensors, limit switches, or even things from the driver station like joystick values or match information.

The writes do the opposite - write output values to each of the joints as needed. The hardware interface is responsible for knowing exactly how to do this. This means converting to correct units or command values, and then calling the correct functions which write to the hardware.

The controller\_manager `update()` method calls the corresponding `update()` method in each loaded controller library. Those `update()` methods use the current state of the hardware (generated by `read()`) combined with the most recent ROS control message received to calculate new commands for the joints it controls. It uses the joint handles loaded in the init phase described above to write those commands to the appropriate variables in the hardware\_interface.

The call to hardware interface's `write()` method loops over all initialized joints, writing each “next command” to the appropriate physical device. As an optimization, the hardware interface remembers which value was most recently written to the hardware. If the next command is same as the last, it skips this redundant write to the hardware.

Note that in addition to the benefits describe above, this architecture provides a clean break for the sim version of the code. The hardware\_interface is replaced with one that can read/write to/from the simulation rather than to actual encoders and motor controllers. No other code would need to change in any of the controller libraries or in any of the other ROS nodes (obviously, how well the simulation models the actual robot will impact how well those other components work).

### Actual implementation

Here's an attempt to describe the ROS control implementation code. As always it is hard for docs to keep up with actual code but the basics should stay the same.

The controller code is based off of so-called “boilerplate” example ROS control code and is stored here : [https://github.com/FRC900/2019Offseason/tree/master/zebROS\_ws/src/ros\_control\_boilerplate](https://github.com/FRC900/2019Offseason/tree/master/zebROS_ws/src/ros_control_boilerplate "https://github.com/FRC900/2019Offseason/tree/master/zebROS_ws/src/ros_control_boilerplate"). The src subdir has a few important components in it. For now, we'll focus on the generic hardware control loop and a base class for an FRC Robot.

The generic hardware control loop class combines a hardware interface with an instance of a controller manager. It has a simple bit of init code and the read/update/write loop described above. The implementation is in [https://github.com/FRC900/2019Offseason/blob/master/zebROS\_ws/src/ros\_control\_boilerplate/src/generic\_hw\_control\_loop.cpp](https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/ros_control_boilerplate/src/generic_hw_control_loop.cpp "https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/ros_control_boilerplate/src/generic_hw_control_loop.cpp").

The [hardware interface base class](https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/ros_control_boilerplate/include/ros_control_boilerplate/frc_robot_interface.h "https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/ros_control_boilerplate/include/ros_control_boilerplate/frc_robot_interface.h") has basic code to interface with hardware. The object holds arrays for objects holding the state of the controlled hardware as well as arrays holding the next command to send out to that hardware. It also has variables to hold handles to each of the entries in those arrays. And it has defined interfaces for each type of joint.

The init() method of this class reads the complete list of joint definitions from a config file. Using that data, it fills in the state and command arrays, and then initializes the handles and interfaces to make them accessible to controllers.

It has pure virtual implementations for read() and write(), which means a derived class must be defined for specific robot hardware. This makes sense - robot hardware varies quite a bit, so read() and write() are going to be very different for each type of robot.

The implementation of such a class is [frcrobot_hw_interface](https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/ros_control_boilerplate/src/frcrobot_hw_interface.cpp "https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/ros_control_boilerplate/src/frcrobot_hw_interface.cpp").

### Hardware state data

The hardware interface stores the most recently read state from the hardware. This is actually an array of state data - 1 entry per physical hardware “thing”. A thing could be a motor controller, a PWM or analog/digital IO pin, and maybe even a joystick.

As an example, we'll use the Talon state. There's a \`std::vector<TalonHWState> talon\_state\_\` member variable in frc\_robot\_interface.h. This is an array of state, 1 entry per Talon. The object itself is defined in [https://github.com/FRC900/2019Offseason/blob/master/zebROS\_ws/src/talon\_interface/include/talon\_interface/talon\_state\_interface.h](https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/talon_interface/include/talon_interface/talon_state_interface.h "https://github.com/FRC900/2019Offseason/blob/master/zebROS_ws/src/talon_interface/include/talon_interface/talon_state_interface.h"). This is a container holding member variables which store position, speed, output voltage and other state for the Talon. There are also basic setters and getters for each member variable.

The code in read() at [https://github.com/FRC900/2018Offseason/blob/544096af0bd9d3af97708088f809a5630e634f33/zebROS\_ws/src/ros\_control\_boilerplate/src/frcrobot\_hw\_interface.cpp#L760](https://github.com/FRC900/2018Offseason/blob/544096af0bd9d3af97708088f809a5630e634f33/zebROS_ws/src/ros_control_boilerplate/src/frcrobot_hw_interface.cpp#L760 "https://github.com/FRC900/2018Offseason/blob/544096af0bd9d3af97708088f809a5630e634f33/zebROS_ws/src/ros_control_boilerplate/src/frcrobot_hw_interface.cpp#L760") gets the current values from the hardware. It uses them to set the appropriate member variables in the talon\_state\_ array. This way any control code which needs to access these values (e.g. using an encoder value as feedback) can get them from the correct entry in the joint\_states\_ array.

A controller doesn't have direct access to the joint\_states\_ array - they're private members of the FRCRobotInterface class. Since the controllers aren't members of that class, we have to provide another way to get them. This method is called a \_handle\_. There's some C++ trickery involved, but the basic idea is that controllers can ask for a joint \_handle\_ of a given type using its name. The handle that's returned includes a way to access the corresponding variable holding state or command data.

Finally is a so-called interface. This is simply a wrapper class which allows arrays of handles to be added and tracked by the robot code. Each type of joint has a different interface. This means that a controller can ask specifically for, e.g. the joint of type CAN Talon with the name front\_right\_steering.

The interface definition also specifies if access to this handle should be unique or can be shared by multiple bits of code. State is typically read only, which means that it can safely be shared among multiple controllers. Read/write handles typically can not be shared - multiple controllers commanding the same hardware at the same time would lead to unpredictable behavior.

### Talon HW Commands

In addition to tracking state, the hardware\_interface also tracks the next command to be sent to hardware. This is done using another vector of objects, 1 entry per physical Talon. This class is named TalonHWCommand (in talon\_interface.h) and the vector of them is \`std::vector<TalonHWCommand> talon\_command\_\`. This data is used in write(). The values stored there by controller code are written to the physical Talon hardware.

Like the Talon State above, controllers access the data in this array using a handle. TalonCommandHandle is defined in talon\_interface.h. This is mostly wrapper code which passes data through to TalonHWCommand using the appropriate methods in that class.

TalonCommandHandle is derived from TalonStateHandle, which means that users of that handle can access Talon state in addition to setting Talon commands. This is a convenience since it eliminates the need for a controller to maintain both a state handle and command handle for each joint. The most common use case will be one where a controller needs the state of every joint it commands so it is included by default in the TalonCommandHandle code.

Finally, there's a TalonCommandInterface defined to wrap the TalonCommandHandle. In this case, since it would be bad to have two sets of code controlling the same Talon, the Interface is marked as non-shared using the ClaimResources tag.

### Registering Interfaces

When a controller is loaded, it has to be able to get handles to the interfaces it wants. This is done by looking for a given joint name of the appropriate Interface type (e.g. TalonCommandInterface, PDPStateInterface, etc). The controller manager maintains a list of valid <name,interface type> pairs and assigns them as requested when each controller is loaded.

The code to set up the list of valid interfaces is in void FRCRobotInterface::init() : [https://github.com/FRC900/2018Offseason/blob/544096af0bd9d3af97708088f809a5630e634f33/zebROS\_ws/src/ros\_control\_boilerplate/src/frc\_robot\_interface.cpp#L384](https://github.com/FRC900/2018Offseason/blob/544096af0bd9d3af97708088f809a5630e634f33/zebROS_ws/src/ros_control_boilerplate/src/frc_robot_interface.cpp#L384 "https://github.com/FRC900/2018Offseason/blob/544096af0bd9d3af97708088f809a5630e634f33/zebROS_ws/src/ros_control_boilerplate/src/frc_robot_interface.cpp#L384"). Pairings of <joint name, CAN hardware address> are loaded when the controller manager is started. The init code loops through each of them creating interfaces to both a state and command handle.

The state handle takes a name and a place its state data is stored. In this case, the places the data is stored in the appropriate entry in the talon\_state\_ vector. Each talon state handle is created and registered (i.e. stored) in a talon\_state\_interface\_ vector. Once filled in, all the entries in talon\_state\_interface are added to the global list of valid hardware resources using a call to registerInterface.

The same happens for talon command handles. Each handle is created using a name and an entry in the talon\_command\_handles\_ vector. The handle is added to the vector of all talon command handles, and when that vector stores all of the command handles, the full list is added to the complete list of robot hardware via the registerInterface call.

### Standard ROS Interfaces

ROS's control stack is designed for a slightly different control scheme than we have. The standard ROS control setup has all of the control code, including things like PID loops, running in controllers in the hardware interface. Thus, they provide basic controllers which provide e.g. position, velocity and effort control of joints.

In our case, the Talon / Victor motor controllers handle PID control. We simply need to configure them correctly and then send setpoints. This isn't easily handled by the default ROS controllers, which motivates our need for the Talon controller / Talon interface described in a later section.

However, we do use other hardware which is a good fit for the standard ROS controllers. For example, a digital input pin is just a single value. To use this with the ROS controllers, we register the pin as a position joint. The standard ROS code for publishing position joint states automatically takes care of the rest - reading it every read() call and publishing its value along with the rest of the position joints.

Likewise, a solenoid is a good fit to ROS's position controller. The solenoid needs a single value to control it. The ROS position joint interface allows writes of a single value. The stock ROS position controller uses that interface and subscribes to a topic which is a single value. Each update() loop it writes the value from the topic to the position joint we've defined for the solenoid.

### Interface vs. Controller

A quick aside on the difference.

In ROS-speak, an interface is a bit of code which lets other code access joints. It'll like have a get and set method, and maybe something more complex for specialized interfaces.

This interface code can be included in other code to create a controller. In that code, the controller would write data read from a subscribed topic to the hardware using the interface. Or it might read data from joint using the interface and publish it on a topic.

A pre-written controller is generally pretty simple. It has to be to be generic enough to be useful. Most pre-written controllers handle a single value.

For more complex cases, a custom controller can be developed. These could contain interfaces to multiple joints, combining them into one mechanism. The message in or out of them would (potentially) be more complex as well.

### Talon Controller / Controller Interface Code

The mismatch between ROS motor control and our actual control scheme motivated the development of the Talon Controller Interface. This code makes available to controllers an interface to a CTRE Talon motor controller. It contains functions for setting the motor setpoint, and also a (pretty) full set of functions to configure and control the various Talon features. It also contains code for reading the assorted status the Talons provide.

The goal here was to provide code similar to what normal ROS controller interfaces give. Thus, a stock ROS controller for e.g. a drive base could be updated to use the Talon interfaces. Using the stock controller code wouldn't give access to any of the unique features that the Talons provide, but it would work as a starting point.

An additional feature of the Talon interface is reading configuration from parameters loaded from a YAML config file. So in the case above where Talon Interfaces replaced stock ROS ones in a standard ROS controller, the talon-specific features could be configured via parameters. This would require no code changes at all in the controller - the Set() method would work to command the talon in whatever mode it was configured with, using the parameters read at startup. And then as needed, talon-specific features could be added to the code as needed. But the initial step of changing the type of a standard ROS interface to a Talon interface would do the majority of the porting work.

Once the interface was written, it was easy to create simple controllers for running the Talons in their various modes. This way, for cases where a user needs to run just a single talon, this pre-written controller would be available. For example, an intake running off a single motor could use this controller as is, without writing any additional code.

A simple controller using these talon resources is in zebROS\_ws/src/talon\_controllers. Much of the code is in include/talon\_controllers/talon\_controller.h. The init() call does the work of getting a TalonCommandHandle from the controller manager. The name of the requested handle is a parameter set in a config file. Using this name, the controller gets the handle corresponding to that name and stores it in the member variable talon\_.

The code starts a subscriber to “command”. The callback for this command takes the value read and stores it in a buffer. The update() call (remember the read/update/write loop described above) takes the most recent value from the buffer. Using the talon\_ handle, it calls the handle's setCommand() method to write that value to the handle object. The handle object will in turn write that to the correct TalonHWComand entry in the talon\_command\_ vector. When the write() part of the (read/update/write) call happens, it will copy the commanded value from that array and actually write it to the hardware.

### Resources

Control manager. Might be able to add an interface to TalonSRX PID or motion profile code : [http://wiki.ros.org/ros\_control](http://wiki.ros.org/ros_control "http://wiki.ros.org/ros_control")

Boilerplate code for ros\_control : [https://github.com/davetcoleman/ros\_control\_boilerplate](https://github.com/davetcoleman/ros_control_boilerplate "https://github.com/davetcoleman/ros_control_boilerplate")

Discussion of boilerplate code : [https://github.com/ros-controls/ros\_control/issues/198](https://github.com/ros-controls/ros_control/issues/198 "https://github.com/ros-controls/ros_control/issues/198")

Various controllers for different drive bases (possibly already merged into ROS) : [https://github.com/Romea/romea\_controllers](https://github.com/Romea/romea_controllers "https://github.com/Romea/romea_controllers")

Another controller for a steered-base robot : [https://github.com/FRC900/steered\_wheel\_base\_controller](https://github.com/FRC900/steered_wheel_base_controller "https://github.com/FRC900/steered_wheel_base_controller")

Basic implementation of a 4WD robot : [https://github.com/grassjelly/linorobot\_4wd](https://github.com/grassjelly/linorobot_4wd "https://github.com/grassjelly/linorobot_4wd")

Ros control block diagram : [https://bitbucket.org/osrf/gazebo\_tutorials/raw/default/ros\_control/Gazebo\_ros\_transmission.png](https://bitbucket.org/osrf/gazebo_tutorials/raw/default/ros_control/Gazebo_ros_transmission.png "https://bitbucket.org/osrf/gazebo_tutorials/raw/default/ros_control/Gazebo_ros_transmission.png")

Complete implementation of ros\_control based robot : [https://github.com/CentroEPiaggio/vito-robot](https://github.com/CentroEPiaggio/vito-robot "https://github.com/CentroEPiaggio/vito-robot")