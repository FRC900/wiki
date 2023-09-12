## **Robot Code Setup**

### [**Install Docker**](#install-docker)
- `sudo apt update`
- `sudo apt install -y curl`
- `curl https://get.docker.com | sudo sh`
- `sudo gpasswd -a $USER docker`
- Restart your VM (for WSL, close and reopen your terminal)

### [**Make Docker autostart**](#docker-autostart)
Note, only one of these options will match your particular OS.  Pick the one that makes the most sense.
- For native linux systems :
  - `sudo systemctl --now enable docker`
- For Windows 11 WSL2
  - add to /etc/wsl.conf
      ```
	  [boot]
      systemd=true
	  ```
- For Windows 10 WSL2
  - add to ~/.profile
    ```
	if service docker status 2>&1 | grep -q "is not running"; then
      wsl.exe -d "${WSL_DISTRO_NAME}" -u root -e /usr/sbin/service docker start >/dev/null 2>&1
    fi
	```

### [**Install Git Large File Support**](#install-git-lfs)
- `sudo apt install -y wget`
```
   cd &&\
   wget https://github.com/git-lfs/git-lfs/releases/download/v3.3.0/git-lfs-linux-amd64-v3.3.0.tar.gz &&\
   mkdir git-lfs-install &&\
   cd git-lfs-install &&\
   tar -xzf ../git-lfs-linux-amd64-v3.3.0.tar.gz &&\
   cd git-lfs-3.3.0 &&\
   sudo ./install.sh &&\
   cd &&\
   rm -rf git-lfs-linux-amd64-v3.3.0.tar.gz git-lfs-install &&\
   git lfs install
```

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
- `./docker-run` (if on an Apple silicon Mac, use `./docker-run-mac`) 
- Wait for things to download

### [**How to enter Docker**](#how-to-enter-docker)
The general Docker reference is [here](/tools/docker.md).
- `docker ps -a`, find the name of your container
- `docker start [container name]`
- `docker exec -it [container name] /bin/bash`

### [**Set up submodules/.rosinstall**](#set-up-submodulesrosinstall)
**Inside Docker:**
- `cd 2023RobotCode/zebROS_ws`
- `git submodule update --init`
- `wstool update -t src`

### [**Build code**](#build-code)
- `natbuild`, this will take a while
