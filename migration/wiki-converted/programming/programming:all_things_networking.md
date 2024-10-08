#  All Things Networking # 

This is a page for all networking information such as IP addresses, networking best practices...

  - [Network Configuration](Network Configuration)
  - [Networking Helpful Info](Networking Helpful Info)
  - [FRC Event Networking](https://wpilib.screenstepslive.com/s/4485/m/24193/l/319135-ip-networking-at-the-event)
  - [Intro to Networking](https://www.youtube.com/watch?v=FM169QUIQco)

###  Configuration Settings for Devices and Common IPs### 

IP: [IP]

Netmask: 255.255.255.0 OR /24

Gateway: 10.9.0.1

DNS: 8.8.8.8

RoboRIO: 10.9.0.2

Jetson: 10.9.0.8

Second Jetson: 10.9.0.9

DS: 10.9.0.5

Raspberry Pi: 10.9.0.7

###  Reserved Ports ### 

FIRST reserves ports 5800-5810 for "Team Use". Current port assignments for our team are:
  - 5801 - SSH
  - 5802 - ROS
  - 5804 - GStreamer

###  How to Connect roboRIO to Internet ### 

Plug in laptop with USB connection to the roboRIO, change the network settings to DHCP in webdash. Reboot the robot. Plug in Ethernet cord from roboRIO to network switch, with another laptop, plug in with ethernet to network switch. Using the laptop still connected of USB, find IP address of RIO, and ssh into RIO from the laptop connected to the switch. You should now have internet on the RIO.


Note: not sure if that works? ssh onto the RIO failed in the lab when attempted 1/4/2020, though ping to the RIO worked
  

Alternatively in the lab, after you changed the roboRIO network settings to DHCP, you can disconnect the roboRIO from the radio, then run an ethernet connection from the roboRIO to the black box on the lab wall near the wooden table with desktop computers. The black box has internet, so the roboRIO will have internet. Reboot the robot, then use the webdash to find out the roboRIO's assigned IP address.
###  How to Connect Jetson to Internet ### 

Unplug Jetson from robot, plug it into power station, monitor, keyboard, and mouse, Click network button in top right to DHCP, then plug into the network switch. You should now have internet on the Jetson.

###  How to use a laptop to get the robot on the internet ### 

This requires a laptop with both a wired connection and wifi.  A usb<->ethernet dongle might work, but hasn't been tested.

Disconnect the robot radio - unplug it.
Connect the laptop to the robot with an ethernet cable.  

Configure a wired network connection on the laptop for a static IP of 10.9.0.1, netmask 255.255.255.0, gateway 10.9.0.1, DNS 8.8.8.8.  Enable that network connection.

Connect the laptop wifi to the internet.  eduroam in the lab, tether to a cell phone, whatever works.

Go into the top level robot code directory (2019RobotCode as of preseason 2019).  Edit robot_ip_masq.sh. The lines which need to be changed are INTERNET_HW and PRIVATE_HW.  These can be found by running ifconfig.  For PRIVATE_HW, look for the name of the adapter connected to the 10.9.0.1 address.  For INTERNET_HW, look for the wifi adapter name connected to the internet.  See [here](https://www.cyberciti.biz/faq/linux-list-network-interfaces-names-command/) for examples.


Run `sudo ./robot_ipmasq.sh`.  This will set the laptop up to route traffic from the internet, through the laptop, to the robot 10.9.0.x network.

When finished, run `sudo ./robot_ipmasq_off.sh` to disable.  Note - I've seen this confuse docker - might need a `sudo systemctl restart docker` to reenable networking inside containers.

###  Radio Configuration ### 

To configure radios, download the zip file for the Radio Configuration Utility from the internet. Do all the things until you have the app on your desktop. Then connect the radio to a power cable. Also, connect it to your computer/drivestation using an ethernet cable. The ethernet cable has to go into the port next to the port for the power source. After that, you need to put in your team number and what you want to name the radio. Finally, you update the firmware and then configure the radio. Then you're done!

When you're plugging radios into a robot, the orange cable (power over ethernet) goes into the port which says 18-24v POE, while the black cable goes into the port which says 802.3af POE.