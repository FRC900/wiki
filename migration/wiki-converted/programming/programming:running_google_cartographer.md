## Running Google Cartographer## 


Cartographer is Google’s mapping software. Here’s how to run it.

There are a few entries in ~/2017VisionCode/setup_ROS.sh to install cartographer and dependencies. Run them.

Cartographer expects to be installed into a directory tree under the base zebROS directory called install_isolated. The command to build & install it :

```bash
catkin_make_isolated --install --use-ninja
```
If you get errors about previous builds using cmake, delete the old build and devel subdirs and try again. .\\
Once this is complete, run

```bash
source install_isolated/setup.bash
```
After this you may have to manually reset your local IP :

```bash
export ROS_IP=127.0.0.1
```
Download the example bag files : https:%%*%%drive.google.com/open?id=0B8hPVHrmVeDgUWlHWGxUSzVFSUU.

Launch ros in demo mode :

```bash
roslaunch controller_node demo_zv_2d.launch bag_filename:=~/RPLidar_2017-03-10-11-11-53_0.bag 
```