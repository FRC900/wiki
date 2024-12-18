##  To setup ntp on the rio: ## 

`%%opkg install ntp ntp-tickadj%%` on the rio restart the rio and ntp will be running in the background `%%vi /etc/ntp.conf%%` and edit the contents to be the ntp.conf server file linked [ntp.conf server](here) it will allow a client with the ip of your choice to sync with the rio `%%/etc/init.d/ntpd restart%%` to restart the service after any changes to the ntp.conf file

##  To install ntp on the jetson you will need to download these two .deb files onto a flash drive. ntp.deb libopts.deb ## 

plug the flash drive into the jetson and ssh in. on the jetson (port: 5801 for sshing in) find the correct drive using `%%sudo fdisk -l%%` you should be looking for something like /dev/sda1 `%%mkdir /media/usb%%` to create an empty directory to mount the flash drive in `%%mount /dev/sda1 /media/usb%%` mount the flash drive `%%cd /media/usb%%` go into the directory `%%dpgk -i package%%` You need to do this for both the ntp and the libopts .deb files `%%vim /etc/ntp.conf%%` edit the contents to be the ntp.conf client file linked [ntp.conf client](here)

[this](https://serverfault.com/questions/806274/how-to-set-up-local-ntp-server-without-internet-access-on-ubuntu) is a good resource for ntp.conf files

##  NTP Configuration ## 

In order to simplify NTP configuration, set up `%%/etc/ntp.conf%%` to operate in broadcast mode. The advantage of broadcast mode is that the devices syncing from the roboRIO (or whatever device we decide to make the time master) do not need to know the IP address of the server.

The roboRIO should have an `%%/etc/ntp.conf%%` file that looks like:

```
# DON'T PANIC!
# Prevents NTP from giving up and exiting when time is more than
# +/- 1000 seconds off from the servers we are synchronizing against.
tinker panic 0

# The driftfile must remain in a place specific to this
# machine - it records the machine specific clock error.
driftfile /var/lib/ntp/ntp.drift

# Specify 4 servers from ntp.org to use for synchronization when we can
# get to the Internet. "iburst" allows for faster synchronization when
# these servers are first contacted.
server 0.pool.ntp.org iburst
server 1.pool.ntp.org iburst
server 2.pool.ntp.org iburst
server 3.pool.ntp.org iburst

# Configure undisciplined local clock when external servers are
# unavailable.
server 127.127.1.1 iburst minpoll 3 prefer
fudge  127.127.1.1 stratum 12

# Broadcast time to anyone listening on the 10.9.0.x network.
# "minpoll 3" makes the server broadcast every 2^3 seconds for faster
# synchronization when the robot is powered up.
broadcast 10.9.0.255 minpoll 3

# Disable authentication. It's a trusted network and authentication
# setup is a pain.
disable auth
```
Any other device syncing off the roboRIO should have an `%%/etc/ntp.conf%%` file that looks like:
```
# DON'T PANIC!
# Prevents NTP from giving up and exiting when time is more than
# +/- 1000 seconds off from the servers we are synchronizing against.
tinker panic 0

# The driftfile must remain in a place specific to this
# machine - it records the machine specific clock error.
driftfile /var/lib/ntp/ntp.drift

# Synchronize directly with the roboRIO at 10.9.0.2 (substitute correct
# IP address if available). Use "iburst" to speed initial synchronization.
server 10.9.0.2 iburst minpoll 3

# Listen for time being broadcast as a backup.
broadcastclient

# Disable authentication and believe any server broadcasting time on the
# subnet so we don't have to mess with exchanging keys.
disable auth
```
##  These instructions to set up rtc on the rio. ## 

Create a startup script called `%%setupClock%%` with the following code.

```bash
#!/bin/bash
NAME="Setup Clock"
USER=admin
depmod
echo bq32000 0x68 | tee /sys/class/i2c-adapter/i2c-2/new_device
retries=0
hwclock.util-linux --utc --hctosys
while [ $? -ne 0 && $retries < 5 ]( $? -ne 0 && $retries < 5 ); do
    sleep 5
    retries+=1
    hwclock.util-linux --utc --hctosys
done
```
and type `%%chmod +x setupClock%%` to make the script executable

and do `%%cp setupClock /etc/init.d%%`

`%%/usr/sbin/update-rc.d -f setupClock defaults%%` to install the startup script

if you need to remove a startup script do `%%/usr/sbin/update-rc.d -f setupClock remove%%`

##  Force Jetson to sync ## 

`%%ntpd -gq NTPSERVERIP%%` -q forces it to sync with the NTP server -g makes it work even if there is a massive offset

##  NTP Commands ## 

`%%sudo service ntp stop%%` stop ntp

`%%/etc/init.d/ntpd restart%%` to reset ntp

`%%sudo service ntp start%%` start ntp

`%%ntptime%%` to see what time ntp is set to

`%%ntpq -p%%` to check connection

`%%ntpq -c as%%` to check status

`%%ntpq -c pe%%` to see offset and other useful info

`%%hwclock.util-linux -r -f /dev/rtc%%` or `%%hwclock --show%%`to check the hw clock

## If the date is wrong on the roboRIO, how to fix it## 
```bashdate -s 7/5/2018 #or the correct day
date -s 19:11:00 #or the correct time
cp /usr/share/zoneinfo/America/New_York /etc/localtime #fix time zones if it's not EDT
hwclock -w #write to the hardware clock```
