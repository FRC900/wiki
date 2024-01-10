##  Setting up the DS Stream ## 
###  Networking ### 

  - Refer to [All Things Networking](programming:all_things_networking)

###  Setup ### 
  - Connect to the robot wifi and ssh into the pi using `ssh pi@10.9.0.7`
  - Open `stream.sh` -- After the `udpsink` command, make sure the host is set to 10.9.0.7
  - Shutdown the pi using `sudo shutdown now`
  - Make sure the DS's IP is set to 10.9.0.5 by running `ipconfig` (should already be set)
  - Make sure overlay.png is in `C://Pictures\Overlays\`
    - run [ generate_image.py](https://github.com/FRC900/2019RobotCode/blob/master/ds_stream/generate_image.py ) to generate and overlay file
      - add a drawCross(res, <your x>, <your y>) for each crosshair to be generated
###  Instructions ### 
  - The Pi is set up to run `stream.sh` on boot up
  - Double-click the gstreamer icon on the DS desktop

