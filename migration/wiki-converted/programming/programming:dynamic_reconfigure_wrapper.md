This is obsolete - use ddynamic_reconfigure code instead

In ROS, dynamic reconfigure is used to set ros parameters from a GUI. The code sets up a list of parameters to expose to the gui via a config file. A callback is created which is called when those parameters are changed. The callback is responsible for copying updated values into variables in the program to implement the change.

The dynamic reconfigure wrapper simplifies setup for this process. The code is in src/dynamic_reconfigure_wrapper/include/dynamic_reconfigure_wrapper.h. Code using this includes the file, creates a DynamicReconfigureWrapper variable (as either a global or, if posible, a member variable), and calls init(). The Wrapper is responsible for creating a dynamic reconfig server, hooking it up to the correct config values and setting up the server to call the correct callback.

Steps for using this -

Create a config file. This is the same process as a standard ROS dynamic setup - http://wiki.ros.org/dynamic_reconfigure/Tutorials/HowToWriteYourFirstCfgFile.

Create a config object in your code. Include the .h file generated from the cfg file (typically something like <my_package/BlahConfig.h>). The config object type will then be BlahConfig - create a variable of that type. For example : `BlahConfig config_`

In the init section of your program, add code to read from ROS parameters into the members of the config_ object you created above. This is similar to reading them into individual global or member vars using `nh.getParam`, but reading them into the config item will make it possible to keep the params read from config files in sync with params changed using dynamic reconfigure.

Create a dynamic reconfigure object and make it part of your code. This is a templated object, so it needs to have a type included. It should be a member variable, or if not part of a class, a global. Code will look something like

```DynamicReconfigureWrapper<BlahConfig> dynamic_reconfigure_wrapper_;```

In your program's init code, near the end of init, call `dynamic_reconfigure_wrapper_.init()`. init takes 2 arguments

    the programs node handle
    the name of the config object created above

```dynamic_reconfigure_server_.init(controller_nh, config_);```

Last step is to use the config_ member vars in your code, in place of individual global or member vars. Typically, anything in older code which was a global or member var read from `getParam` should use the replacement config_ member var instead.