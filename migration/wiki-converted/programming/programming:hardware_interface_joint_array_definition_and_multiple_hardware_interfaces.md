##  Hardware Interface joint array definition and multiple hardware interfaces ## 

A goal for this year is to run hardware interfaces on both the Rio and Jetson.  The Jetson will be connected to the CAN bus along with the Rio, and in general we expect CAN-based devices to be controlled by the Jetson.  This currently works to read & write Talons, and read PDP and PCM state.

Analog & digital IOs, navX, rumble, etc, will still be driven from the Rio.

Solenoids are an open question. Technically these are CAN-based devices, but given how FRC safety code works it is unknown if we can write to them from the Jetson.

This document describes how to get two or more hardware interfaces to coordinate with each other.

###  Joint Array Overview ### 

The first thing to do when setting up a config file for the hardware interface is to define a joints array.  This array holds information on each bit of hardware on the robot which can change state or be controlled by the hardware interface. A soon to be out of date list of things which can be defined as joints :

  - can_talon_srx
  - nidec_brushless
  - digital_input
  - digital_output
  - pwm
  - solenoid
  - double_solenoid
  - Rumble
  - navX
  - analog_input
  - compressor
  - pdp
  - dummy
  - ready
  - joystick

The function of many should be obvious - they correspond to things which are hooked up to the Rio or driver station.

Things which might be less obvious 

Rumble : allows the hardware interface to write to joystick rumble
Dummy : used to more abstract communicate state between controllers and/or hardware interface. Many time used to set a value which is written to the driverstation dashboard
Ready : used to signal that robot code has finished loading

Each entry in the joints array has a name and type. In addition, each type has zero or more additional fields.  These will define the specifics of the particular named joint type.  For example, it might include a hardware port number, set an invert flag, and so on.  

###  What needs to be coordinated between hardware interfaces ### 

Basically, each hardware interface needs to be configured with knowledge of where each joint is connected and going to be controlled from.  

For read-only hardware, this is (relatively) simple.  We’ve added a local field to each joint array entry.  This is a boolean. It defaults to true, which makes it backwards compatible in cases where just 1 hardware interface is running.

A read-only joint is defined to be local if it is physically connected to the processor a given hardware interface is running on.  For analog and digital in, this means the joint array on the Rio will define these joints to be local. On the Jetson, they’ll be defined as remote.  

For CAN-connected read only joints (e.g. PDP or PCM state) it is expected to read / publish their values from the Jetson. This is part of a general drive to push everything onto the Jetson since it is way faster than the Rio.

Reads of a local joint are simple - just read from the connected hardware.  This is how things work now. The controllers run on the Rio, hardware is connected to the Rio, so just read local hardware and store the result.

Reads of a remote joint are more complex, but probably necessary. Imagine there’s a controller which runs on the Jetson to control CAN-based Talons (again, it is running there for performance reasons).  In many cases, the behavior of this controller relies on knowledge of analog or digital inputs to the rio - a line break or distance sensor, perhaps. 

The point is that there needs to be some way to get local joint data to remote hardware interfaces.  There could be code added to each controller to subscribe to, say, the joint_states message and read from there, but that’s going to add a lot of redundant code. Plus, spoiler alert, doing it differently sets us up to be able to write to remote hardware as well.

Instead, we’ve added a set of state_listener controllers.  These are essentially the opposite of e.g. the joint_state_controller. Instead of reading from internal hardware interface state and publishing data, these subscribe to a state topic, read non-local joint data from those messages, and write them to the hardware interface of the non-local controller.  They sync internal joint state of all hardware controllers on the robot. 

This means that a controller doesn’t have to know if a joint is local or remote - it just reads from the appropriate handle and the hardware interface, combined with appropriate state listener controllers, takes care of the rest.

Writes are a bit more complicated. It turns out you need two different local flags to handle the various cases.  

One flag, local_update, defines where the controller writing to the joint is running. The other flag, local_hardware, identifies which processor the hardware is physically connected to.  As long as the appropriate state

The various permutations :

local_update = true, local_hardware = true.  Here, there’s a controller running on one processor. And that processor is where the hardware is physically connected.  This matches the single hardware interface case.  For example, a controller running on the Rio which drives the Rio’s digital outputs.  Or a controller running on the Jetson driving Talons.  For this, you can just set local to true - it implies both local_update and local_hardware is true.

local_update = false, local_hardware = true.  In this case, there’s hardware connected to a processor.  But unlike the case above, the controller writing to it is running in the hardware interface of a different processor.  Imagine a controller running on the Jetson - for performance reasons - which needs to write to digital output on the Rio.  In both cases, the Rio’s joint array entry would be set with this config for the joints physically connected to the Rio.

local_update = true, local_hardware = false. Basically, the mirror image of the above.  The hardware is connected locally but the controller writing it is running somewhere else.  For the example in the previous section, the joint array entry would be set up like this on the Jetson.

local_update = false, local_hardware = false.  This means that hardware connected to another processor is being controlled by another processor.  Which sounds like it means you could just not define that joint for this hardware interface at all. But there are cases where it matters 
If a local controller needs the state of the joint in question
Very important - the Rio needs at least one Talon set up with these settings.  This allows the Rio to send heartbeat CAN packets which keep all the Talons enabled. Note that this should be a Talon at a CAN ID which doesn’t exist - setting it up for an existing Talon will cause conflicts because both the Rio and Jetson are sending control frames to the device.
Probably best to have the joint list the same across all processors to make it easy to move stuff around if needed
In any case, like the true/true case, setting local = false is shorthand for both local_update = false and local_hardware=false.

Reading between the lines, this means we need a top-level yaml file for each hardware interface.  

###  Various Stuff ### 

Since we’re running hardware interfaces on multiple processors, the way we signal robot code ready has to change slightly.  We’ve added a “ready” type. Each hwi needs to define one of these variables and mark it local = true.  The last controller spawned by each hwi should be robot_code_ready_x, which updates that particular var to a non-zero value (e.g. 900).  The hwi running the Rio needs to have joint array entries for all of the remote ready joints (with local = false) plus one for itself.  The code handles the rest.

There are two new top-level params added to the hardware_interface node handle.  

run_hal_robot - this is a Rio / not Rio flag.  It defaults to true.  Set it to false in the config file for running an hwi on the Jetson.

can_interface - this identifies the linux can device to access for non-Rio hwi instances.  It should be can0 unless something is really weird.

Since there are two (or more, please no!) hwi copies running, we have to split the /frcrobot namespace into /frcrobot_rio and /frcrobot_jetson to avoid publishing the same topic multiple places.  This is going to cause issues for nodes which are hard-coded to /frcrobot/blah.  Consider not using an absolute path for the topic name. Instead, use a relative one, and then launch the node inside a namespace group.

###  Joint State Listener config ### 

This is the controller which reads state from remote hwi and copies them into local ones.  The setup requires a topic to monitor.  This should be a state topic from the other hwi instance. So e.g. the rio config will subscribe to /frcrobot_jetson/joint_states

###  What if we can’t write to Talons from the Jetson ### 

What’s slow about Talon access is reading them.  Writes are relatively quick by comparison.  So we’d set it up so that Talons are read from the Jetson, and then their values are published to a ROS topic via the talon_state_controller. We’d run a talon_state_listener on the Rio which grabs data from that topic and writes it to talon state stored internally in the Rio’s hwi.  

Then we can run Talon controllers on the Rio.  They’d behave just like now - read state, use that to create commands in update(), then the Rio hwi will write those commands to the Talons over CAN.  Only difference is talon state will be coming from the Jetson, where reads of the Talon are way, way, faster.

Hopefully we don’t have to, but the code is mostly there. We’ll have to figure out exactly which Talon state to copy from the Jetson. We’d only want to get data that’s actually coming from the Talon while ignoring talon state that’s actually written from the controller (e.g. we care about encoder values but want to not overwrite PIDF or mode set from the Rio).

Running the controller on the Jetson would be a bit more complicated. We’d need a way to publish talon commands from the Jetson, which currently doesn’t exist. Then we’d also need a way to listen and copy those commands to the Rio.  It is possible but more difficult than the above, where a talon state publisher exists and a talon state listener is at least hacked together.

###  Joint Type Documentation ### 

All joints have a name and type field which are required. The following types are supported

can_talon_srx

This is used to run Talon SRX motor controllers connected via CAN.  Parameters
can_id - the device ID of the Talon
local / local_update / local_hardware : described above

nidec_brushless

A NIDEC brushless motor connected to the RIO. Parameters :

pwm_channel - The PWM channel on the RIO the motor is connected to
dio_channel - the digital input/output channel on the Rio the motor is connected to
local / local_update / local_hardware : described above
invert - invert the output of the motor. Optional, defaults to false.

####  Digital input #### 

A sensor or other input device connected to the digital IO pins on the Rio. 

dio_channel - identifies the Rio digital channel to use for input for this device
invert - invert the input. Optional, defaults to false
local : described above (note, no local_update / local_hardware since this is a read-only device)

####  Digital output #### 

A device driven from one of the digital IO pins on the Rio

dio_channel - identifies the Rio digital channel to use for input for this device
invert - invert the output. Optional, defaults to false
local / local_update / local_hardware : described above


####  PWM #### 

A device connected to the PWM output of the Rio. Warning : Largely untested

pwm_channel - identifies the Rio PWM channel to use for input for this device
invert - invert the output. Optional, defaults to false
local / local_update / local_hardware : described above

####  Solenoid #### 

A solenoid connected to a PCM module.


pcm - identifies the the ID of the PCM the solenoid is connected to
id - which port on the PCM the solenoid is connected to
local / local_update / local_hardware : described above

####  Double solenoid #### 

A double-solenoid setup connected to a PCM module

pcm - identifies the the ID of the PCM the solenoid is connected to
forward_id - which port on the PCM the forward solenoid is connected to
reverse_id - which port on the PCM the reverse solenoid is connected to
local / local_update / local_hardware : described above

####  Rumble #### 

Sets up a joint to write to the rumble output on a given USB controller attached to the driver station

rumble_port : identifies the port number of the device to rumble
local / local_update / local_hardware : described above







