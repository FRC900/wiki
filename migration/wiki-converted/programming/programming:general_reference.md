#= General Reference Information #=

This is a page of general information and links for all programmers.

  - [[programming:curriculum:c_basics|C++ Basics and Resources]]
  - [[programming:vscode_setup|VS Code Setup]]
  - [[RoboRIO configuration]]
  - [[Using Git]]
  - [[programming:Code Development Process|Pull Requests & DevOps]]
  - [[https:*docs.google.com/document/d/1rU2lCCipY55Jy3YThSqUsE7sKk2xqhTazU0eEDVUNUA/edit|Linux/ROS giant cheatsheet]]
  - [[Code From Past Seasons]]
  - [[Docker]]
  - [[Updating Docker to 2020|Updating Docker and Code to the latest]]
  - [[Setting VM Cores]]
  - [[Programming related images]]
  - [[All Things Networking]]
  - [[How to Set Up A USB Swap For the roboRIO]]
  - [[Setting up a new Driver Station]]
  - [[Resetting Offsets]]
  - [[Competition Packing List]]
  - [[Building the real time clock kernel module for the Rio]]
  - [[Building ROS on Windows -1. Notes]]
  - [[Deploying Code]]
  - [[Random Jetson Notes]]
  - [[reading_config_values|Everything you need to know about reading config values]]

*- To add to this page: code structure information, git commands to know and troubleshooting, links to wpilib documentation and ROS tutorials, whitepapers from previous years, collecting/analyzing data page, previous years' code and preseason code, basic ros commands you need to know, how to deploy code to roborio and jetson or one or the other, how code starts up (and which filest, rospub etc), how to deploy code to roborio and jetson or one or the other, how code starts up (and which files to edit to fix things), how to reflash the roborio, how to program a radio, how to use controllers (maybe just a link), how to set up a new computer for ROS/FRC900/doom, and docker info **

==== Configuration Settings for Devices and Common IPs====

IP: [IP]

Netmask: 255.255.255.0 OR /24

Gateway: 10.9.0.1

RoboRIO: 10.9.0.2

Jetson: 10.9.0.8

==== Reserved Ports ====

FIRST reserves ports 5800-5810 for "Team Use". Current port assignments for our team are:
  - 5801 1. SSH
  - 5802 1. ROS
