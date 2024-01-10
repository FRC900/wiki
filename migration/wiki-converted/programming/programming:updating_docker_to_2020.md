## Updating Docker to 2023## 

First, you want to exit all of the docker containers that you're currently running. Do this by running Ctrl-D or `exit` from inside the docker container.

From your local machine (NOT inside the docker container), you want to delete all your frc docker containers.  **NOTE** : If you have tensorflow containers do not remove those. Run `docker ps -a` to list all of the current frc containers. For each of these containers, run `docker stop <CONTAINER ID>` then `docker rm <CONTAINER ID>`. 

Then, you want to delete your frc docker images. **NOTE** : Again, if you have tensorflow docker images do not remove them.  Run `docker image list` to list all of the current images. For each of the frc images, run `docker image rm <IMAGE ID>`.

From your home directory, run `git clone git@github.com:FRC900/2023RobotCode.git`. If the 2023RobotCode already exists, change directories into 2023RobotCode and run `git pull -r`.  Change directories into 2023RobotCode/docker, then run `./docker-run`. Wait for this to download -- it'll take a while.

When that's done and you're inside the docker container, go into the 2023RobotCode/zebROS_ws/src directory. Run `git submodule init`, `git submodule update`, and `wstool update -j 4`. Then, run a ./native_build.sh.

### Master changed to main in 2023### 

The `master` branch has been renamed to `main` in 2023.  Anywhere master was used previously use main in its place.

###  Dealing with errors from wstool ### 

Sometimes there will be errors thrown by wstool. The way to fix this : 

Note what directories / packages are failing.
`cd ~/2023RobotCode/zebROS_ws/src/<package_name>` then `git pull -r`

###  Moving branches from previous repos ### 

Branches can be moved to a the new repo if needed.

First off, add the old repo as a remote for the new one

`git remote add 2022 git@github.com:FRC900/2023RobotCode.git`

This will add the old 2022 code as a remote repo accessible from the 2023.  This only needs to be done once.

Then, to get the latest 2022 code, do

`git fetch -a 2022`

This will pull in the most recent info (commits, branches, etc) into the 2023 repo.

To check out a branch from 2022, do something like

`git checkout -t 2022/my_branch_name`

It can then be pushed to the 2023 repo usin

`git push -u origin branch_name`

From there on, any additional pushes from that branch will update the 2023 repo instead of the 2022 code.

