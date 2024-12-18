(Page describing the process for recompiling the kernel for the TX1.)


----

#  Recompiling the TX1 Kernel # 

This was done on a ubuntu 16.04 system running as a virtual machine. You need 50GB of freespace, FYI. It’s a cross-compile environment and makes this whole process a lot faster.

  - Start by pulling down the sources and the L4T documentation from Nvidia (see links below).


  - Read the “Building the NVIDIA Kernel” section.


  - It will point you to downloading and installing toolchains for ARM64.


  - You need BOTH toolchains.


  - Before you run the setup scripts for the toolchains, run this to get the extra packages: “apt-get install gawk texinfo automake libtool g++”.

Clean install of latest L4T on the Jetson. Dont change it other than that.

You have to export some configurations (The first one isn’t incomplete, it’s a prefix)

  - `%%$ export CROSS_COMPILE=/home/ubuntu/Desktop/workspace/toolchain/toolchain-build-aarch64/install/bin/aarch64-unknown-linux-gnu-%%`
  - `%%$ export CROSS32CC=/home/ubuntu/Desktop/workspace/toolchain/toolchain-build-armhf/install/bin/arm-unknown-linux-gnueabi-gcc%%`
  - `%%$ export TEGRA_KERNEL_OUT=/home/ubuntu/Desktop/workspace/sources/kernel_out/%%`
  - `%%$ export ARCH=arm64%%`
  - `%%$ cd <myworkspace>/<kernel_source>%%`
  - `%%$ mkdir $TEGRA_KERNEL_OUT%%`
  - `%%$ make O=$TEGRA_KERNEL_OUT tegra21_defconfig%%`
  - Edit the .config file to enable NVME (CONFIG_BLK_DEV_NVME=y) and possibly set a CONFIG_LOCALVERSION extension for the kernel
  - `%%$ make O=$TEGRA_KERNEL_OUT zImage%%`
  - `%%$ make O=$TEGRA_KERNEL_OUT dtbs%%`
  - `%%$ make O=$TEGRA_KERNEL_OUT modules%%`
  - `%%$ mkdir /home/ubuntu/Desktop/workspace/sources/kernel_out/modules_out%%`
  - `%%$ make O=$TEGRA_KERNEL_OUT modules_install INSTALL_MOD_PATH=/home/ubuntu/Desktop/workspace/sources/kernel_out/modules_out%%`
  - `%%Copy DTB files from here: /homt/ubuntu/Desktop/workspace/sources/kernel_out/arch/arm64/boot/dts/%%`
  - `%%Copy Image and zImage from here: /homt/ubuntu/Desktop/workspace/sources/kernel_out/arch/arm64/boot/%%`
  - `%%Copy /lib/modules/<uname -r> from /home/ubuntu/Desktop/workspace/sources/kernel_out/modules_out%%`

Verify that the DTB file you are replacing in /boot/extlinux/extlinux.conf is correct.

Reboot and deal with the inevitable errors the first time you do this.


----

Useful links:

  - L4T Documentation: http://developer.nvidia.com/embedded/dlc/l4t-documentation-24-2-1 (also has the source_sync script and the scripts for building the toolchains)
  - https://devtalk.nvidia.com/default/topic/929186/jetson-tx1/jetson-tx1-kernel-compilation/
  - http://developer.ridgerun.com/wiki/index.php?title=Compiling_Tegra_X1_source_code
  - https://devtalk.nvidia.com/default/topic/930642/how-to-compile-tegra-x1-source-code/
  - https://devtalk.nvidia.com/default/topic/917386/usb-3-0-port-unstable-on-jetson-tx1-/
  - https://devtalk.nvidia.com/default/topic/973034/jetson-tx1/problem-with-intel-600p-nvme-ssd
  - http://codegists.com/code/cross-compile-windows-dll-in-linux/