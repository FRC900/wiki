#=  READ THIS DOC #=
[[https:*docs.google.com/document/d/13AoeIFF3kKoX3-IzGBw2naoREyNXJA3SDAEPcFOD_VM | cool doc]]


#= Prerequisites #=

  - Jetson TX1 or TX2
  - Laptop or desktop computer running Ubuntu 14.04 or 16.04
  - Micro USB cable for attaching the computer to the Jetson

#= Install Steps #=

  1. Connect your laptop to the Jetson via USB. The micro-USB connector connects to the micro-usb port on the Jetson, the big USB port to your laptop
  1. Download latest JetPack version from https:%%*%%developer.nvidia.com/embedded/jetpack (registration required) on your laptop or desktop computer you are planning to use Save it in a known location (i.e., Downloads directory).
  1. Open a terminal window and cd to where the JetPack was downloaded to (Downloads directory).
  1. Run `%%chmod a+x JetPack-L4T-2.3.1-linux-x64.run%%` (or whatever the file name of the JetPack version you downloaded) to make it executable.
  1. mkdir ~/Jetpack
  1. cd ~/Jetpack
  1. Run `%%~/Downloads/JetPack-L4T-2.3.1-linux-x64.run%%` to start the JetPack installer.
  1. Next, Next to skip the first two info screens
  1. Select either TX1 or TX2 then pick next
  1. Enter root password if asked, then next
  1. On the JetPack L4T Component Manager screen, choose the action “uninstall” on the “Host 1. Ubuntu” Line. Pick “Keep & Apply” in the dialog that pops up. This will prevent the installer from adding a bunch of software to your laptop.
  1. Next
  1. Select all licenses and Accept
  1. Click through the important notice
  1. Then click next to proceed when prompted
  1. Make sure the Jetson is plugged into the lab network, click next
  1. Pick the network interface on your laptop which is connected to the lab network. If you’re connected via wifi, this will probably start with a “w”. If wired, it is probably an “e”. Click next
  1. Click next again
  1. Power on the jetson
  1. Follow the instruction in the text window, then hit enter
  1. Go grab lunch, come back, hopefully it is done.