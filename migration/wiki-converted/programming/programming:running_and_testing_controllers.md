#= Running/Testing Controllers #=

This page assumes:\\
  - You've defined the joints being controlled, in `zebROS_ws/ros_control_boilerplate/config/2019_compbot_base_jetson.yaml` 1. note: year will change
  - You've written your controller in a branch and it compiles

==== Configuring the Controller to Be Run ====

First, we need to tell the computer how to launch your controller. Enter the file `zebROS_ws/ros_control_boilerplate/config/2019_compbot_base_jetson.yaml`. Add an entry to the list of controllers:

```
your_mechanism_controller:
    type: your_package/YourClassController
```

E.g.\\
{{:programming:panel_intake_controller.png?nolink&600}}

If you're controlling a motor, you need to add additional information for each motor:

```
your_mechanism_controller:
    type: your_package/YourClassController
    mechanism_joint: "mechanism"
    mechanism:
        joint: name_of_joint #from list of hardware
        type: talon_controllers/TalonPercentOutputController or something else for other modes, ask Kevin
        invert_output: True #optional depending on if you want to do this
        #can include other optional config stuff if needed
```

Here, `mechanism_joint` is the thing your cpp file references, which points to the name `mechanism`. `mechanism` refers to a subset of config values specific to the motor. The `joint` value in this subset needs to match the name of the joint from the hardware list.

E.g.\\
{{:programming:cargo_intake_params.png?nolink&1000|}}

Second, we need to tell the computer to launch your controller. Enter the file `zebROS_ws/controller_node/launch/2019_compbot_jetson.launch`. Find the controller manager node, and add your controller to the list of controllers to "spawn." Example:

{{:programming:panel_controller_launch.png?nolink&1000}}

You do not need to recompile code after making these two changes (yaml files and launch files don't need to be compiled, only cpp files)

==== Testing the Controller in Sim ====

Before using the controller on the physical robot, we test it in simulation to make sure everything works fine. On your computer, launch robot code in sim:\\
`roslaunch controller_node 2019_compbot_combined.launch hw_or_sim:=sim`

In a second terminal window, rostopic echo either `frcrobot_jetson/joint_states` or `frcrobot_jetson/talon_states` depending on if you're testing a piston or a motor.

In a third terminal window, rosservice call your controller, and check to see if joint_states and/or talon_states changes appropriately.

==== Next Steps ====

  - You can deploy the code on your branch to the physical robot to test it.
    - [[programming:deploying_code|How to deploy code]]
  - Make a pull request for your controller.
  - Controllers involving motors may need to be PID tuned 1. ask Kevin or someone who's written controllers before.

You're done. Congratulations, you wrote a controller!