Note : this might be obsolete. The Jetson can remotely launch nodes on the Rio as part of it starting up.

#  Introduction # 

This page covers roboRIO boot, the services that are started by default and how to disable them, and how to configure our code to start on boot.

#  Init System # 

The roboRIO doesn’t use the new-fangled [systemd](https://freedesktop.org/wiki/Software/systemd/) system for initialization that most recent Linux distributions have moved to but instead uses the good old fashioned [System V init scripts](https://en.wikipedia.org/wiki/Init#SysV-style).

#  Default roboRIO init # 

The roboRIO starts in runlevel 5 by default. (This is strange because most System V init-based Linux distributions use runlevel 5 for graphical, multi-user mode with networking. Runlevel 3 would be more appropriate, but it is what it is.)

##  Services started in runlevel 5 ## 

  - S01networking
  - S01nicgroup1createcpusets
  - S01nicgroup2createcpuacctgroups
  - S01nisetupdump
  - S02dbus-1
  - S03nisetupkernelconfig
  - S04nisetprimarymac
  - S04nisetupirqpriority
  - S05niauth
  - S09sshd
  - S19syslog
  - S20busybox-ifplugd
  - S20fuse
  - S20nipal
  - S20nisvcloc
  - S20ntpd
  - S20setupClock
  - S21avahi-daemon
  - S22NiRioRpcServer
  - S24nimdnsResponder
  - S25nibds
  - S28niwifibledd
  - S31nirtmdnsd
  - S80vsftpd
  - S85niembcan
  - S88FRCNetComm
  - S90crond
  - S90nivisaserverd
  - S98nivalidatestartup
  - S99rmnologin.sh
  - S99stop-bootlogd

#  Launching our services # 

  All code is started remotely by the Jetson - no setup is needed on the RIO itself.

#  Disabling Unnecessary Services # 

There are several services that run by default on the roboRIO that are unnecessary when running ROS. These services are `%%nilvrt%%`, `%%systemWebServer%%`, and others. This saves memory and CPU time that can be used for more useful purposes.

Note : disabing the systemWebServer will prevent the WebDashboad from working. This will prevent us from being able to e.g. set IP addresses so make sure those tasks are completed and working before disabling the service.

To stop these services and prevent these services from running on boot, run:

```bash
update-rc.d -f nilvrt remove
update-rc.d -f lvrt-wrapper remove
update-rc.d -f systemWebServer remove
/etc/init.d/nilvrt stop
/etc/init.d/systemWebServer stop
```
If these services need to be started later, for WebDashboard, for example, run:

```bash
/etc/init.d/systemWebServer start
```
For LabView support run:

```bash
/etc/init.d/nilvrt start
```
```
S01networking            S05niauth       S20setupClock   S85niembcan
S01nicgroup1createcpusets    S09sshd         S21avahi-daemon     S88FRCNetComm
S01nicgroup2createcpuacctgroups  S19syslog       S22NiRioRpcServer   S90crond
S01nisetupdump           S20busybox-ifplugd  S24nimdnsResponder  S90nivisaserverd
S02dbus-1            S20fuse         S25nibds        S98nivalidatestartup
S03nisetupkernelconfig       S20nipal        S28niwifibledd  S99rmnologin.sh
S04nisetprimarymac       S20nisvcloc         S31nirtmdnsd    S99stop-bootlogd
S04nisetupirqpriority        S20ntpd         S80vsftpd
```
