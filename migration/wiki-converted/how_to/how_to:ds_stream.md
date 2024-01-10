# Setting up the DS Stream #
==== Networking ====

  - Refer to [[programming:all_things_networking|All Things Networking]]

==== Setup ====
  1. Connect to the robot wifi and ssh into the pi using `ssh pi@10.9.0.7`
  1. Open `stream.sh` -1. After the `udpsink` command, make sure the host is set to 10.9.0.7
  1. Shutdown the pi using `sudo shutdown now`
  1. Make sure the DS's IP is set to 10.9.0.5 by running `ipconfig` (should already be set)
  1. Make sure overlay.png is in `C:\Pictures\Overlays\`
    1. run [[https:*github.com/FRC900/2019RobotCode/blob/master/ds_stream/generate_image.py | generate_image.py]] to generate and overlay file
      1. add a drawCross(res, <your x>, <your y>) for each crosshair to be generated
==== Instructions ====
  1. The Pi is set up to run `stream.sh` on boot up
  1. Double-click the gstreamer icon on the DS desktop

