#  Planning for Object Detection Work # 

One of the goals this year is detecting objects on the field using neural networks.

Specifically, we'd like to use an object detection model (probably [SSD](https://arxiv.org/abs/1512.02325)-based) written in Tensorflow running on the Jetson. This document is a work in progress to plan out how to get from here (zero) to there.

##  High Level Overview of Work ## 

This includes everything needed to develop, train test and run the models. I can think of a few basic parts

  - The development environment - [Tensorflow](https://github.com/tensorflow/tensorflow) itself, the [tensorflow object detection models](https://github.com/tensorflow/models/tree/master/research/object_detection), tools to generate datasets ([labelImg](https://github.com/tzutalin/labelImg)?).  
  - Our object detection data. This would be our FRC-specific data, tools to process the data, tools to train the model, any customization to the model itself. Basically, things which we expect to change way more frequently than the list above
  - Actual ROS code which will run on the robot.

##  Development Environment ## 

I think the best place for the first point bullet is in a container, similar to our normal ROS container. Not sure if it should or shouldn't be combined with the already-existing container. I'm thinking no, simply because it is so different. This needs lots of deep learning tools that have nothing to do with ROS, and likewise, this needs basically zero ROS stuff.  

What we need to make this container - basically a list of shell commands to install/build/configure whatever tools we need to include.  This is probably the top priority since everything else we do will depend on it.

##  Our FRC-specific object detection data ## 

Likely a git repo, since it will be changed fairly frequently, we'll want version control on scripts, and so on.

Since images and videos will be large, probably need git-lfs support.

This requires the dev environment to be ready, so slightly lower priority.

Need to come up with a directory structure that makes sense. In particular some way to keep track of changes in training data - that's going to be more difficult to track than code in many way. Naming conventions and directory structure will be very important to keep us from overwriting things or losing track of what is what.

Tentative repo is set up in the tensorflow_workspace repo. To start a container with tensorflow, the tensorflow models and labelImg installed, run 

```bashcd
git clone https://github.com/FRC900/tensorflow_workspace.git
cd tensorflow_workspace
./setup.sh
./docker-run```

##  ROS object detection ## 

This will probably be added to the main repo / container.  The container will get additions for the bare minimum needed to run a model, and similar stuff will need to be added to the jetson setup code.

Checkpoints / frozen models will also be copied over here from the main object detection repo as they are tested and improved on.

The main dev work in here will be hooking some version of the tensorflow object detection code into something which reads and writes ROS messages.

Another bit of work will be optimizing the model using TensorRT.  Maybe. That might also happen in the other repo and just the results moved over here.  Not sure.

Work here can start pretty quickly.  A first goal would be to get a generic pre-trained object detection model running using ROS input and output.  This can happen without custom trained models, with those added later as they are available.