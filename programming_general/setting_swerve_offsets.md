
    Set the wheels so that they're facing the front of the robot with the bevel gear facing right.

    Zero all 4 angle motor cancoder magnet offsets. This can be done by setting them to 0 in the swerve_cancoder_hw.yaml file or using Phoenix Tuner X.

    Call the service "/frcrobot_jetson/dump_offsets" with no arguments

    On the Jetson, in ros_control_boilerplate/config/, that service has created a file named "offsets_numbers.yaml." Move that file to something_swerve_cancoder_hw.yaml (for example, 2024_compbot_swerve_cancoder_hw.yaml), depending on which robot being configured (for example, 2024_compbot_swerve_cancoder_hw.yaml) Don't forget to also copy the file onto your local machine and commit to git.

    Restart code for the changes to take place.
