formatting and update

This page is intended to show how to set up a development environment for building code which will run on the RoboRIO.  We'll be using a process called cross-compiling.  Here, code is written and compiled on a laptop or desktop machine running Linux.  After it successfully compiles it is copied over the the Rio for testing.  

###  Installing ARM FRC toolchain ### 

Skip this - use docker instead.  Skip directly to the building code section


FIRST provides a compiler and related tools for cross compiling code for the RoboRIO.  Enter the commands below to install it on your Linux laptop :

```bash
sudo apt-add-repository ppa:wpilib/toolchain
sudo add-apt-repository ppa:wpilib/toolchain-beta
sudo add-apt-repository ppa:webupd8team/java
sudo apt-get update
sudo apt-get install git libc6-i386 curl jstest-gtk gradle oracle-java8-installer frc-toolchain meshlab cmake libprotobuf-dev libprotoc-dev protobuf-compiler ninja-build sip-dev python-empy libtinyxml2-dev libeigen3-dev libpython2.7-dev
```

###  Installing wpilib ### 

wpilib is a set of code needed for the RoboRIO to communicate with hardware attached to the RIO, and also with the driver station.  We're using the 2018 beta code which is behind a password-protected web site right now.  So until it is released to the general public, download it from here : https://drive.google.com/open?id=1iE9qeX3LgzQwM-7DyfdPmJw2Q-jv1Slz

Extract it to your home directory :

```bash
cd
tar -xjf <path to file>/wpilib_2018.tar.bz2
```

This should create a wpilib director tree under your home directory.  You shouldn't have to do any additional setup - the code expects it to be there and should find it automatically.

###  Installing ROS and assorted other code ### 

Since we're using ROS, you'll have to install various other code which is used to build ours. This is mainly ROS code plus the libraries which ROS needs.  Download it from here : https://drive.google.com/open?id=1zqutiqgwnfmUT0i98mocCAp5ytk5t7Mv

Once it is downloaded, extract it : 

```bash
cd /
sudo tar -xjf <path to file>/ros_cross.tar.bz2
```

As with the wpilib code above, our build environment will know where to find these files once they've been extracted to the correct location.

###  Building Code ### 

Setup for building using the following commands (only needed once per terminal session) : 

```bash
source /usr/arm-frc-linux-gnueabi/opt/ros/kinetic/setup.bash
cd ~/2018RobotCode/zebROS_ws
```

To actually build code : 

```bash
./cross_build.sh
```
This will use catkin_make to generate code that should run on the Rio.  To transfer the code over

```bash
tar -cjf ~/rio.tar.bz2 install_isolated
scp ~/ros.tar.bz2 admin@<rio IP address>:.
```

On the Rio : 

```bash
cd ~/2018RobotCode/zebROS_ws
tar -xjf ~/rio.tar.bz2
```

Then on the Rio, run ROS as normal (first line only needed once per terminal)

```bash
source rio_bashrc.sh
rosrun/roslaunch whatever you want
```