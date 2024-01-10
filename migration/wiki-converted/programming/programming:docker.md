#= Docker development environment #=


# Introduction #

[[https:*www.docker.com/what-docker|Docker]] is a framework for packaging applications into containers for execution in their own isolated environment. It's not strictly accurate to call it a lightweight virtual machine (VM), but the analogy is close enough. The advantage for using it for our development environment is that we can package our toolchain for building for ROS both natively and for the roboRIO.

To get an idea of what Docker is and what it does, reading the [[https:*docs.docker.com/engine/understanding-docker/|Docker Overview]] document will give a good idea of how it works.

# Installing Docker #

**NOTE: THIS INSTALLATION INFORMATION IS OUTDATED. FOR UPDATED INFORMATION, SEE EITHER __[[programming:curriculum:ubuntu|THIS PAGE]]__ OR GO DIRECTLY TO THIS GUIDE:**\\
https:*docs.google.com/document/d/1Tatn_AO20QY5a7Z8FREisJILmsiPbGAA3VSFKOOGNMU/edit
\\

*Frankly, most of this page is either outdated or documented elsewhere. See the curriculum for better information.*\\
\\

Docker containers run directly on any Docker host and do not have to run inside a VM. Sometimes VMs are used to easily bring up a Docker host but Docker container can run directly on any Linux system with Docker installed. For running Docker containers on Windows or Mac, virtualization is used under the covers to start a Linux VM as a Docker host.

==== Linux Install ====

[[https:*docs.docker.com/engine/installation/|Installing Docker on Linux]] is straightforward. There are packages available for Ubuntu and every major Linux distribution.

For Ubuntu 16.04, Docker installation is as simple as running the following commands:

  - ```bashsudo apt install docker.io```
  - ```bashsudo usermod -a -G docker <userid>```

Where `<nowiki><userid></nowiki>` is your user ID in Ubuntu and log out and log back in.

==== Windows and Mac Installation ====

Docker versions for [[https:*docs.docker.com/docker-for-windows/|Windows]] and [[https:*docs.docker.com/docker-for-mac/|Mac]] are available. Docker for Windows uses Microsoft Hyper-V to start a Linux VM while Docker for Mac uses HyperKit. Another option for getting Docker running on Windows or Mac is [[https:*kitematic.com/|Kitematic]] although Docker [[https:*docs.docker.com/kitematic/userguide/|now recommends]] using Docker for Windows or Docker for Mac in place of Kitematic.

Personally, I haven't tried running Docker on Windows or MacOS so if you have any issues, you're on your own. =)

# Pulling the ZebROS Docker Development Image #
ZebROS Docker images are available in the [[https:*​hub.docker.com/​u/​frc900/​|Docker Hub]] for download. Once you have Docker up and running on your system, you can run ```bash​docker pull frc900/zebros-2020-beta-dev```​ to download the latest version of our ROS development environment.	 
 
The Docker image is large (around 10 GB) so be sure to run the `<nowiki>​docker pull</nowiki>`​ command while connected to a fast internet connection or it will take a long time. The image pull only needs to be done once. When the development image changes, only incremental changes need to be pulled so it will run much faster.

# Building and Running ZebROS Code #
Before  you start a new ZebROS container by executing the `%%docker-run%%` command, you should run the following commands to clone and initialize a new git repo:

  - ```bashcd```
  - ```bashgit clone https:*github.com/FRC900/2019RobotCode.git```
  - ```bashcd ~/2019RobotCode```
  - ```bashgit submodule update --init --recursive```
  - ```bashcd ~/2019RobotCode/zebROS_ws```
  - ```bashwstool update -t src```

Unfortunately, these steps are needed while our GitHub repository is private, as we do not want the Docker image to include our private code at this time.

# Starting a ZebROS Container #

Once the image is pulled, you can start a container by using `<nowiki>docker run</nowiki>`. There is [[https:*github.com/FRC900/2019RobotCode/blob/master/docker/docker-run|a docker-run script]] in the 2019RobotCode repository in the docker directory that you can run to start a new container.

When you execute the docker-run script, you will be inside a bash shell inside the container. If you run `docker ps` in another shell, you will see something like:

```
# docker ps
CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS              PORTS               NAMES
b8d9d374887f        eblau1/zebros-dev   "/bin/bash"         43 hours ago        Up 52 minutes                           lonely_keller
```

This shows the container you're running, identified by the hex container ID, and the image that the container is using. When you exit the shell you are in, the container itself will stop and will not show up in `%%docker ps%%`. To see exited containers, you must use the -a argument (`%%docker ps -a%%`).

```
# docker ps -a
CONTAINER ID        IMAGE               COMMAND             CREATED             STATUS                    PORTS               NAMES
b8d9d374887f        eblau1/zebros-dev   "/bin/bash"         43 hours ago        Exited (0) 43 hours ago                       lonely_keller
```

Once a container has exited, you can remove it by running `%%docker rm <containerId>%%` or you can restart the container by running `%%docker start <containerId>%%` and attaching to the bash shell again by running `%%docker attach <containerId>%%`. If you want to start another shell in the same container (i.e., you want to use multiple terminals), you can run `%%docker exec -it <containerId> /bin/bash%%`.

# Updating to a new image #

Our docker image will be updated at various times during the year.  When this happens, use the following steps to grab the latest copy.

Exit from all containers running in terminal windows.

```bashdocker pull frc900/zebros-2020-beta-dev:latest
~/2019Offseason/docker/docker-run
```

This will download the new image and start a new container using that image.

See below for getting rid of older containers and images.

# Troubleshooting #

If you want to start any graphical programs inside your Docker container, run `%%xhost +%%` on your Linux Docker host outside of the container to allow windows from inside the container access to be displayed. If this is not done, you will get errors like the following:

```
Gtk-WARNING **: cannot open display: :0
```


==== Cleaning up ====

Eventually, most people have a bunch of old containers that are no longer needed.  

Use `docker ps -a` to list all of the containers on your machine.  Typically, only the most recent one is needed.  Containers can be removed using `docker rm container_name`.  Running containers can not be removed 1. stop them with `docker stop container_name`.

Old images take up disk space and can also be removed.  First, stop and remove all containers using those images using the steps from above.  Then, do a `docker image list`.  The most recent image will have the tag "latest".  Others will have a different tag, typically <none>.  To remove those older containers, use `docker image rm image_id`, where the image ID is the list of numbers and letters in the "IMAGE ID" column from `docker image list`.


