# Quick Start Guide
This section contains (hopefully) all the information you need to get started with working on robot code. As always, if you are stuck on anything, ask in mattermost and someone will help you out. 

## How to start on a task
1. Go to [prog_gen_tasks](https://github.com/FRC900/prog_gen_tasks/issues) and choose any of the recent tasks (ones that say good first issue are also probably good). You should be able to assign yourself to the task if no one is working on it.
2. You are going to want to make a new branch for your task. First make sure you have the latest code by running `git checkout main` and `git pull -r` in 2023RobotCode. Then you can run `git checkout -b [branch name]` to create a new branch and switch to it. The branch name should be something related to the task you are working on. 
3. Now you can go into docker a

## Common Testing Commands
### Building Code
You need to build your code before you can run it. Depending on the task you may have made a new package, in this  
- `natbuild`: builds all code
- `natbuild [package_name]`: builds just one package (faster than building everything)

### Running Code 
- `simlaunch` This launches hardware simulation along with the sim driver station. If you are testing something that uses motors it is a good idea to use this and make sure everything looks right. 
- `roslaunch controller_node 2024_sim.launch` This launches the non hardward simulation which brings up stageros (2d driving simulation). This is useful for testing anything related to driving or vision (stage also has apriltags and objects). 

## Common "gotchas"
- If you are getting an error about a package not being found, make sure you have run `git submodule update` in 2023RobotCode (in docker) as well as `wstool update` in 2023RobotCode/zebROS_ws (in docker).
- 