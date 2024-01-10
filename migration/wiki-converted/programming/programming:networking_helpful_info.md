## Networking Helpful Info## 

Are you trying to connect to a robot? Maybe even move it? Well, you're in the right place!

#### Connecting#### 
You can get access to the processors on the robot in a few ways. 
  - You can connect to its radio, which is as simple as connecting to the correct WiFi network. Its name will start with 900 and will probably have something to do with robots.
  - You can connect over an ethernet cord. You should plug into the switch, not the Jetson or the RIO itself.
  - You can connect over USB, using a USB A to B cord (normal USB on one end, square on the other end). Plug the square end into the RIO. This will only give you access to the RIO, and is really only good for connecting drivers stations or troubleshooting problems.

Over ethernet or WiFi, you can check if you're connected with the 'ping' command. Ping just checks to see if you can send and receive messages to a certain IP address.

Jetson: 10.9.0.8
roboRIO: 10.9.0.2
Radio: 10.9.0.1

So try 'ping 10.9.0.8' to see if you can reach the Jetson.

#### SSH#### 

#### Programming radios#### 