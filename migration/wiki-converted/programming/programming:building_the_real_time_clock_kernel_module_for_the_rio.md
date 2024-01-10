This only needs to be done when an image with a new kernel version is released for the roborio. This is typically once per season.

Log into the rio (ssh admin@10.9.0.2)

```bash
mkdir build
cd build
wget https:*raw.githubusercontent.com/FRC900/rtc-bq32k-rRIO/master/Makefile
wget https:*raw.githubusercontent.com/FRC900/rtc-bq32k-rRIO/master/rtc-bq32k.c
opkg install gcc
source /usr/local/natinst/tools/versioning_utils.sh 
setup_versioning_env
versioning_call make
cp rtc-bq32k.ko /lib/modules/`uname -r`/kernel/
cd ..
rm -rf build
opkg remove libmpfr4 libmpc3 gcc cpp
```


Also, copy (scp) the file back into the root of the git repo and commit it. This way, the next time a rio is imaged the updated kernel module will be used instead of having to do this entire process again.