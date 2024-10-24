We support a variety of different type of hardware, but each year there's always more.  This page attempts to document the process for adding low level support.

Typically this hardware will have support from either WPIlib or a vendor library. Those libraries will have functions to create, read and write to the hardware.  The code here will bridge the gap between those libraries and ROS controllers.

##  Basic Outline ## 

Code needs to be added to :

   - Create data structures (classes) to hold the state and commands for the hardware
   - Read config values from the joint list
   - Using that config, create buffers of the state and command classes 
   - Initialize the hardware
   - Read from the hardware into the state buffer
   - Write from the command buffer to the actual hardware
   - Publish state information (a state controller)
   - Simulate the hardware to some degree

The rest of this is a tutorial on what to do. For the hows, review https://wiki.team900.org/doku.php?id=programming:ros_low_level_control


##  State Data Structures ## 

One of the jobs of the ROS hardware interface is to keep track of the state of all the hardware on the robot. It does this by reading values from the actual hardware and storing them in a buffer in memory.  The data stored by this buffer are specific to the type of hardware. One of the first tasks when adding new hardware is to define the data structure which holds all of the state needed by the hardware.

The exact contents of this are hardware-specific. For an encoder, for example, state might be position and velocity.  For a color sensor, it might be the color values last read.  

Hardware state can also include config values used to set up the hardware.  These would be stored in the state so they can be displayed for debugging purposes.

Our convention for these files is to put them in a <device name>_interfaces package.  If it is something supplied by WPIlib, use the frc_interfaces directory.  For Cross the Road Electronics, use the talon_interfaces directory.  spark_max_interfaces for Rev products.  And if it is a new type of device, create a new _interface package (as was done for the as726x_interface).

Note that the existing files in those directories will also be very useful references for adding new hardware.

The class used to hold state data is, by convention, DevicenameState, or perhaps DevicenameHWState. Ending in State makes it obvious what it is used for.

The class should be in the hardware_interface namespace, and probably best to create a device_name namespace under that since the hardware_interface namespace is getting crowded.

The class should have private members for each piece of state data.  The constructor should set them to reasonable defaults.  Read-only state for data which never changed (e.g. the CAN address of a device) can also be passed in via the constructor.

The rest of the members should have setBlah and getBlah public methods defined.

Although some of the first-written classes had this code defined inline, it is really better practice to have the .h file be a definition of the class and a .cpp class to implement the code.

Finally, a few lines of boilerplate code are needed to define a StateInterface and a RemoteStateInterface.  Easiest thing to do there is copy-paste from an existing file and change the types in the typedef StateHandle lines.

#  Command Data Structures # 

The command data structure is used by the hardware interface to queue up commands written to the hardware. Controllers have access to write to this data structure in their update() method.  The write() function in the hardware interface takes the values in the data structure and actually writes the requested values to the hardware.

The contents of the command data structure could be config items. Or perhaps commands to immediately set the state of the hardware.  Like the state class above, it will vary from device to device.

The command data structure should be named something like DevicenameCommand.  And it should be in the same namespace as the State structure above.

The command data structure should have member vars for each command.  It also should have a corresponding _changed_flag_ for each of those variables. This will be used by the hardware interface to force the code to only write values to the hardware when they have been modified, hopefully saving time by avoiding redundant write operations.

Note there might not be a changed_ variable for each config item. Instead, there should be one changed_ per function in the hardware library code. For example, if a single hardware call writes three values, in the command data structure there would be member vars to hold the three values and one changed_ var shared by all three.  An update to any one of those would require the hardware interface write() call to write to the hardware, so there's no point in having individual changed_ flags for each.

The constructor should set sane defaults for all member vars.  To force those values to be written to the hardware the the first update loop, the corresponding changed_ var can be initialized to true.

Each member var should have a set and get method.  get returns the current value.  

setBlah should

  - check to see if the new value is different from the old value
  - if so, set the member var to the new value and set the corresponding changed_ flag to true
  - if not, do nothing

Each changed_ var should have two methods.

blahChanged takes as arguments all of the member vars associated with is as references.  The function copies the member var values into the arguments.  If changed_ is true, set it to false.  Return the initial value of changed_.  See the many examples in other interface code for details on this.

resetBlah sets the changed_ value to true.  This is used by the hardware interface to force a re-write to the hardware in the event of an error.

There's also boilerplate CommandInterface declarations.  Like the state code, easiest here is copy-paste from a working example and change the types and names to match the device being created.

##  Reading Config / Param Values ## 

This is code added to the ros_control_boilerplate frc_robot_interface code to load data from rosparms about the hardware.  

First off, create some data structures to read the params into.  Look at frc_robot_interface.h, search for names_.  This will lead to the section of definitions of config data read for each type of hardware. New hardware will probably be similar - a name, a hardware address(s) of some sort, perhaps some other info.  Typically, aside from the name, look at what sort of info has to be passed to the constructor of the function used in the vendor library to create the hardware.

Additionally, add a local_hardwares_ and local_updates_ vector.  This is used to coordinate data between the rio and jetson.

Finally, add a num_devices_ var to match everything else.

##  Create State and Command Buffers ## 

In frcrobot_interface.hpp, create vectors of the state and command data structures.  For examples, search for State and Command, you'll find vectors of the already existing ones for previous hardware.

Additionally, create state, command and remote state interface member vars for the device.  These are variables used to pass state and command buffers between the hardware interface and controllers. Search for state_interface to find the block of previously defined interfaces for existing hardware.  

Then, in frc_robot_interface.cpp, add code to fill in these values.  The class constructor has a large if/else if block for reading data. Easiest thing is to find an existing device similar to the one being added and copy-paste and start editing.

Next up, is the code to create buffers of the state and command classes written in the previous few sections. Again, copy-pasting previous code is probably best.  In init(), look for num_<example device> and copy that to your device. Change the types and it should be good.  

Finally, at the bottom of init, call registerInterface for the state, command and remote_state interfaces added in the previous paragraph.  Use the existing code as an example.

##  Initialize the Hardware ## 

This has two parts : creating an object for the hardware and actually initializing it.

###  Hardware Data Structure ### 

Typically the low-level code will provide some sort of way to identify the hardware being accessed.  For example, for CTRE motor controllers it is a Talon C++ object. For certain WPIlib code, it is an integer index or perhaps an frc:: object used to control a particular type of hardware.  

Where the value is an integer (or wpilib handle, which is typically an int), the code uses a vector of ints to store the list of hardware.  For cases where objects are necessary to access hardware, our code uses vectors of shared_ptrs or unique_ptrs to that object type to hold a list of the hardware.

That vector is stored in frc_robot_interface.hpp.

###  Init Code ### 

Look in frc_robot_interface.cpp, init().  Here, you'll see code which loops over each case of num_devices_ and does a push_back of newly created hardware objects to the vector for each device type.

Note the code which checks for device_local_hardware_ - if this is false the hardware isn't physically connected to the CPU (jetson or rio) the hardware interface code is running on.  In that case, a nullptr or invalid handle is pushed onto the vector as a placeholder. Since that particular hardware isn't accessed by the hardware interface there's no point in creating an object for it.

##  Read Code ## 

The code here is also in frc_robot_interface.cpp. Look in the read() function.

The code should loop over each device instance for the new device type. For each device, read each state from the actual hardware. Using the value read, call device_state[i].setBlah(blah's value).

Note that for performance reasons, this should only be done for values which can change outside the control of the code. It shouldn't be used to update config values. That is, a config value written to the device should be what sets the devices state entry for that value. This saves time versus reading, say, every config value for every device.  This also holds true for outputs which are set by writing them to the device from code.

On the other hand, sensor values are set from the changing physical state of the sensor and need to be read each time read() is called - there's no other way to read them.

####  Read Threads #### 

If reads take a long time, the code to read them should be moved into a separate thread. This thread will read from the hardware and update a buffer shared between the thread and read.  The read() code will simply copy the last update from this buffer. This means that if the thread takes a long time (relative to read() ) to update that shared buffer, the read function won't be blocked waiting for new data. Instead, it will just repeat the data from the most recent finished read.

Look at the pdp code for an example. This section can be expanded as needed to document in more detail as it becomes necessary.

##  Writing to Hardware ## 

Like the read code, the write code should loop over each instance of the new device. The goal is to write and commands which have changed in the command buffer to the hardware. The general pattern is

```
variable blah
if (device_command_[i].blahChanged(blah))
{
    if (actual_hardware->SetBlah(blah) #####  SUCCESS)
    {
        device_state_[i].setBlah(blah);
        ROS_INFO_STREAM("Set device blah to " << blah);
    }
    else
    {
        device_command_[i].resetBlah();
    }
}
```

The outer-most if block will check to see if blah has been updated in the command buffer since the last time write() was called.  If so, it attempts to write blah to the actual hardware.  The inner if checks if that write was successful, and if so updates the device's state to reflect the change.  

If the write to the hardware fails, resetBlah is called. This will force blahChanged() to return true on the next time through the write() function. This forces a retry of the hardware write.

#  Publish State Information # 

This involves creating a state controller.  The purpose of a state controller is to read the device_state and publish it on a topic. This lets nodes outside the hardware interface use that state.

There are a number of state controllers already written - these should be used as a template for new code.

###  Create a Message ### 

The message will contain fields that correspond to each member var in the state data structure.  Since it is possible to have more than one of a given device type, each field will be an array of the correct type.

Messages should be in a package separate from the controller itself.

###  Create the Controller ### 

Init will grab all of the names of the devices of the new type.  For each device, it will allocate space in each message field array - pushing back default values to them.

Update will loop over each device instance. It will call the get() method on the device state for each state field and use that to set the message output.

##  Simulation ## 
For most cases, wpilib has sim support built in for new hardware. In that case, the wpi code handles simulation automatically. For cases where values are read from real hardware, code is added to frcrobot_sim_interface to create subscribers which allow us to provide simulated input values.  The callback for such a subscriber is responsible for calling HAL_SIM functions which set the value of the simulated hardware.

For hardware without sim support, the following outline allows us to implement basic sim code.

Basic support is simply copying the init() and write() code from frcrobot_hw_interface into frcrobot_sim_interface, then cutting out the calls to the low-level hardware function. And since the low level call isn't being made, the reference to resetBlah() can be removed as well. Basically, just 

```
variable blah
if (device_command_[i].blahChanged(blah))
{
    device_state_[i].setBlah(blah);
    ROS_INFO_STREAM("Set device blah to " << blah);
  
}
```

If it helps debug code, support can be added to set the state of the device.  This would likely involve a service call to set the state of the device.  See the linebreak or limit switch examples currently in the sim interface.
