###  Resetting Offsets ### 

1. Set the wheels so that they're facing the front of the robot with the bevel gear facing right. 

2. Call the service "/frcrobot_jetson/dump_offsets" with no arguments

3. On the Jetson, in ros_control_boilerplate/config/, that service has created a file named "offsets_<unix timestamp>.yaml." Move that file to 202_compbot_offsets.yaml, depending on which robot. Don't forget to also copy the file onto your local machine and commit to git.

4. Restart code for the changes to take place.