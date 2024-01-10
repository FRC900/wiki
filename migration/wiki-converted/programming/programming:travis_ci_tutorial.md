#  Travis CI # 

Travis CI is a service for automatically testing changes to a GitHub repository. Whenever someone pushes to the repo, Travis runs a set script to make sure the the code was not broken. This script can run unit tests, compile code or anything else that is necessary.

##  GitHub Credentials ## 

The environment variables `%%GITHUB_USER%%` and `%%GITHUB_PASS%%` are set by Travis when a build is started. !! These are Will (rivermont)'s username and password. They are hidden in the Travis CI settings and cannot be viewed. They are used to pull from the private 2018RobotCode repo.

##  Repositories with Travis: ## 

Anyone with access to a private repo can see its Travis status.

  - [2018RobotCode](https://travis-ci.com/FRC900/2018RobotCode) - {{https://travis-ci.com/FRC900/2018RobotCode.png?token=T6hJWxFcNyyzxifXQqW5&branch=master&.png?&nocache}}

###  2018RobotCode YAML ### 

Travis is configured with a `%%.travis.yml%%` file in the root directory of a repo.
Here is the YAML file from `%%2018RobotCode%%`, line by line.

<code yaml>
sudo: required
```

This line is needed when using docker.

<code yaml>
language: cpp
```

The language of the project. Travis looks for certain settings given this.

<code yaml>
compiler: gcc
```

The C++ compiler too use when compiling. Travis supports GCC and Clang.

<code yaml>
services:
  - docker
```

Extra services to setup before running the script(s).<br>
We use the FRC900 Docker container that comes preinstalled with ROS and all associated packages.

<code yaml>
before_install:
  - travis_wait docker pull frc900/zebros-dev:latest
```

Download the Docker container. This is done in the `before_install` section because that's how Travis is set up.

```bash
docker run frc900/zebros-dev:latest /bin/bash -l -c "cd ~ && git clone https://$GITHUB_USER:$GITHUB_PASS@github.com/FRC900/2018RobotCode/ && cd ./2018RobotCode/ && git checkout $TRAVIS_COMMIT && git log -n1 && git submodule init && git submodule update && cd zebROS_ws && wstool update -t src && source /usr/arm-frc-linux-gnueabi/opt/ros/kinetic/setup.bash && ./cross_build.sh && echo \"FInished RoboRIO compile.\""
```

Clone the repo and compile our ROS packages (located in `%%2018RobotCode/zebROS_ws/src/%%`) for a regular CPU architecture.

```bash
docker run frc900/zebros-dev:latest /bin/bash -l -c "cd ~ && git clone https://$GITHUB_USER:$GITHUB_PASS@github.com/FRC900/2018RobotCode/ && cd ./2018RobotCode/ && git submodule init && git submodule update && cd zebROS_ws && wstool update -t src && source /usr/arm-frc-linux-gnueabi/opt/ros/kinetic/setup.bash && ./cross_build.sh"
```

Clone the repo and compile the ROS packages for the RoboRIO architecture.

<code yaml>
notifications:
  email: false
  slack: ...
```

Send out notifications about the status of the build after completion.

<code yaml>
template:
  - "Build <%{build_url}|#%{build_number}> (<%{compare_url}|%{commit}>) of %{repository_slug}@%{branch} by %{author} %{result} in %{duration}. If it failed it was probably <@U0S1T88TG>."
```

Format for the Slack notification. More information can be found on the [Slack documentation](https://api.slack.com/docs/message-formatting).

##  Images ## 


**Running Job**

[Files/Images/travis_running_job.png](Running Job) FIXME

**Sample Slack Notification**

[Files/Images/travis_slack_notification.png](Slack Notification) FIXME
