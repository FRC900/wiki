# Setting up a VirtualBox workspace with ROS and OpenCV

## Overview

If you plan on working with ROS  or with OpenCV on either the Vision or Control Systems teams, please follow this tutorial to set up an Ubuntu virtual machine to house all of your work!

## Mac and Windows Users

### Install VirtualBox

**Mac Users:*- Please download VirtualBox at this [link](http://download.virtualbox.org/virtualbox/5.2.2/VirtualBox-5.2.2-119230-OSX.dmg). This link will start the download automatically. Follow the installation process to completion.

**Windows Users:*- Please download VirtualBox at this [link](http://download.virtualbox.org/virtualbox/5.2.2/VirtualBox-5.2.2-119230-Win.exe). This link will start the download automatically. Follow the installation process to completion.

### Set Up A New Virtual Hard Drive

The following images are from a Mac running macOS High Sierra (10.13.1). The process should be similar on other versions of macOS and on Windows.

#### Click on "New" to make a new virtual image.

[Files/Images/VB_1.png](Files/Images/VB_1.png)

#### Fill in the following window matching the example below. Click Create.

[Files/Images/VB_2.png](Files/Images/VB_2.png)

#### Again, follow the below example and click Create.

[Files/Images/VB_3.png](Files/Images/VB_3.png)

#### You now have a fully set up virtual image. Now to start it up and install Ubuntu! Click Start.

[Files/Images/VB_4.png](Files/Images/VB_4.png)

In the meantime, download Ubuntu 16 [here](https://www.ubuntu.com/download/desktop). (Ubuntu 16! Not Ubuntu 17!) You will be prompted to donate to Ubuntu before the download, but there should be a link somewhere at the bottom to take you directly to the download without paying. This may take a while depending on your internet speed. Someone in the lab may already have it on a USB stick, so ask around before you slow down the internet for everyone else.

#### When you click Start, this window appears. Use the file search function to locate wherever you saved the Ubuntu 16 image. Then click Start.

[Files/Images/VB_5.png](Files/Images/VB_5.png)

#### Wait about a minute for the system to start up. Then click Install Ubuntu.

[Files/Images/VB_6.png](Files/Images/VB_6.png)

#### Don't select either of the additional checkboxes and just click Continue.

[Files/Images/VB_7.png](Files/Images/VB_7.png)

The next window prompts you to erase all files on all disks. Select that top option and click Continue. Have no fear, VirtualBox does not have access to your actual OS, so nothing will get deleted. Continue through the prompts, inputting your language, region, name, username, password, etc. Ubuntu will now install itself. This may take a while. Once the setup is complete, you will be prompted to restart. Click Restart Now and the VirtualBox will restart. Press Enter if prompted. Congrats, you now should have a beautiful version of Ubuntu 16 installed! Continue to the "ROS and OpenCV Setup" section.

## Linux Users

If you already have Linux dual-booted or used as your only OS, you're ahead of the game! Your sole job is to convert everyone else to a Linux user. :)

If you have Ubuntu, go to the "ROS and OpenCV Setup" section and follow the instructions. If you have some other Linux distro, the instructions should be similar but you might have to do some Googling.

## ROS and OpenCV Setup

Open up Terminal and install vim and Terminator. Enter your password when prompted and type "y" when prompted.

`sudo apt-get install terminator vim`

### Install ROS

Now we begin following the ROS Kinetic Kame installation. Setup your computer to accept updates from packages.ros.org:

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

Set up your keys:

`sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`

Run an update to make sure Ubuntu is up-to-date:

`sudo apt-get update`

Install the desktop version of ROS:

`sudo apt-get install ros-kinetic-desktop`

Initialize rosdep:

`sudo rosdep init`

`rosdep update`

Finally install these tools to help when building packages.

`sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential`

### Install OpenCV

Installing OpenCV is very easy! 

`sudo apt-get install libopencv-dev python-opencv`

To test that it worked, type `python` into the terminal, and then `import cv2`. If no errors appear, all is well!

### Clone Most Recent Repo

The final step is to clone this year's repo. When this tutorial was made, that was the 2017Preseason repo, but that will change in January. Whatever the most recent repo is, get the link to it in Github, and run this command wherever you want your code to live on your machine.

`git clone <link to repo>`