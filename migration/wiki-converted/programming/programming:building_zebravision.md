#  Prerequisites # 

To install the prerequisites for Zebravision a laptop or desktop computer (not on the Jetson TX1), see the notes below.

##  Installing Jetpack ## 

To re-image a Jetson from scratch, use nVidia’s JetPack. This will download the latest Linux operating system for the Jetson and install it. Note that this will wipe out anything else on the Jetson - make sure there’s nothing important there.

[Flashing Latest JetPack on TX1 and TX2](Flashing Latest JetPack on TX1 and TX2)

Once JetPack is installed, clone the 2017Preseason repository script as follows from a terminal window:

```bash
cd
git clone https://github.com/FRC900/2017Preseason.git
cd 2017Preseason
```
Once this is done, run `%%setup_environment.sh%%`. It will automatically download and install the first set of requirements for Zebravision. After that, run through setup_ROS.sh. This will install ros and pull in code the ROS side of Zebravision depends on.

If running on a Jetson TX1, try using

```bash
sudo /home/ubuntu/2016VisionCode/set_freq_tx.sh
```
first to max out the clocks on the Jetson board for a small speed up. For the TX2, run the following to do the same (yes, repeat the commands twice) :

```bash
sudo nvpmodel -m 0
sudo /home/ubuntu/jetson_clocks.sh
sudo nvpmodel -m 0
sudo /home/ubuntu/jetson_clocks.sh
```
If you hit problems, contact a vision leadership student or a mentor.

TBD - set up a cross-root environment for the Jetson? We might need to build an aarch64 hosted arm-frc-none-linux gcc toolchain for this to work…

#  Laptop installs # 

We have standardized on using Ubuntu 16.04. This matches up with the Jetson and should minimize the difficulty of moving code from Laptop to Jetson and back. Students can choose to either install a VM or dual boot.

Help! Fill in details here!

The two scripts above should work reasonably well on a laptop which has an nV GPU. For laptops without them (probably the vast majority) there are going to be issues installing the ZED driver. Probably the easiest thing to do is just skip installing the ZED SDK.

TBD - we’ll need to install a specific set of tools to build code for the RoboRIO on a laptop.

#  Building the code for Jetsons # 

For code running a Jetson, we’re doing a native build. That is, the code is built on the same Jetson system it will run on. This will also work fine for building code to test on a laptop (as long as we successfully filter out code which needs access to actual robot hardware unique to the RoboRIO).

```bash
cd ~/2017Preseason/zebROS_ws
source /opt/ros/kinetic/setup.bash
source devel/setup.bash   # Note this won't be there the first time you build. Don't panic! It will be created by the first catkin_make call. Be sure to re-source the file once it exists
cakin_make --use-ninja
```
#  Building code for the RoboRIO. # 

Since the RoboRIO is very slow and memory-limited, we’re going to build code for it on another system. This means compiling on a laptop or maybe someday a Jetson (tbd) and the