## **Robot Code Setup**

### [**Install Docker**](#install-docker)
- `sudo apt update`
- `sudo apt install docker.io`
- `sudo gpasswd -a $USER docker`
- Restart your VM (for WSL, close and reopen your terminal)

### [**Set up GitHub SSH key**](#set-up-github-ssh-key)
- `ssh-keygen -t ed25519 -C "your_email@example.com"`
    - Replace `your_email@example.com` with your actual email
    - Hit enter until you get back to a command line
- Copy the output of `cat ~/.ssh/id_ed25519.pub`
- Go to [https://github.com/settings/keys](https://github.com/settings/keys) and click "New SSH Key"
- Paste what you copied into the "Key" field
- Name the key whatever you want

### [**Clone robot code**](#clone-robot-code)
- `cd ~`
- `git clone git@github.com:FRC900/2023RobotCode.git`

### [**Set up Docker container**](#set-up-docker-container)
- `cd ~/2023RobotCode/docker`
- `./docker-run`
- Wait for things to download

### [**How to enter Docker**](#how-to-enter-docker)
The general Docker reference is [here](/tools/docker.md).
- `docker ps -a`, find the name of your container
- `docker start [container name]`
- `docker exec -it [container name] /bin/bash`

### [**Set up submodules/.rosinstall**](#set-up-submodulesrosinstall)
**Inside Docker:**
- `cd 2023RobotCode/zebROS_ws`
- `git submodule init --update`
- `wstool update -t src`

### [**Build code**](#build-code)
- `natbuild`, this will take a while