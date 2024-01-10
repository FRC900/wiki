
[[https:*www.youtube.com/watch?v=aUkBa1zMKv4|Introductory Video]]
[[http:*cecas.clemson.edu/~ahoover/ece854/lecture-notes/lecture-pf.pdf|Bit more rigorous discussion of the math]]

Localization is the task of using robot sensors to figure out where the robot is located. That is, given the sensor reading, give me my coordinates on the field.

In robot lingo, the location is called the state of the robot.  Technically the state also includes things like velocity, but we don't really care about those.  For us, we just need x,y and orientation. And luckily, we have the navX IMU which gives us accurate orientation.  So the real problem is just x and y.  

The particles in particle filter are individual guesses at the robot location / state. They hold both the estimated state and the probability the estimate is correct.  The filter part uses other information (camera data, imu data, commanded velocity) to iteratively pick more and more likely states to narrow down the robot's estimated position.

Multiple particles are tracked, each with different guesses about possible robot state.  The initial population of particles is randomized over the possible locations the robot could exist in.  Over time these guesses evolved and (hopefully) converge to a consistent estimate of the real robot state.  

We take advantage a few different sensors to update the estimate robot state.

====The basic outline of a particle filter operation====

  - drive base command velocity measurements are used to update each particle's location.  Since commands aren't followed exactly, noise is added to each particle's updated location. This attempts to model the inaccuracy in commanded versus actual velocity.
  - Camera data provides a range and bearing to various targets.  Each particle's weight is updated by comparing the observed range and bearing against the range and bearing expected if the particle was the location of the robot.  The probability of getting the observation given the particle's estimated position is used to update that particle's weight.
  - Weights are normalized so they sum to 1.  The weighted sum of the particle's states is the estimate of actual robot state for this iteration.
  - Particles are resampled 1. a new set of particles is drawn.  The probability of drawing a particle is its weight.  Particles with higher weights are more likely to end up in the new sample. This helps the estimate converge to more likely solutions.

This process is repeated for each sensor reading.

====Some interesting issues arise from our implementation====

We update the drive base commanded velocity more often than we get camera data.  Thus the first step may happen multiple times before updating particle weights.

We have IMU data which gives an accurate orientation for the robot.  This is used to update the orientation of all particles for each incoming IMU message. 

To track camera data, we define a set of targets (or beacons, depending on the context).  The code has a list of locations of retro-reflective targets on the field 1. x,y and normal direction from the wall (for filtering out ones which would require us to look through walls to see them, see below).  

====Camera data presents many other challenges====

The targets on the field are (currently) retroreflective tape. These are all the same shape 1. we don't have a way to know for certain which target corresponds to which actual target on the field.  Instead, we just get a list of distance and bearing pairs to the targets we can see.  The process for a best guess at the real ones has a few steps

  - Use our known robot orientation to rule out beacons which are outside of the field of view of the camera.  In other words, targets too far to the sides or behind the robot obviously can not be seen so exclude them.
  - Use knowledge of the facing of the surface the targets are on to rule out anything which would have us looking through walls to see them.
  - For the remaining, use an optimal assignment algorithm to match detected targets with actual beacon locations.  Each <detection, actual target location> is given a cost based on the difference between the actual and expected location of each actual target given the particle's predicted robot state.  The algorithm takes that matrix and creates a mapping from each detection to an actual target which minimizes the sum of the costs.  

We do have an advantage that the possible robot locations are bounded by the field, so we can exclude guesses which put us outside the field.  I mean, it is possible to end up there but if so, localization is the least of our problems.

====Work to do====

The code is in src/pf_localization. On master, there's a working but slow python version. In the pf_cpp branch is the start of porting the code to C++.  An obvious first step is to finish porting the rest of it.

Generate some test cases for various parts of the code.  The C++ code is relatively object oriented, so test cases could create simple programs with known inputs and expected output and start testing with them.

We need a plan for better beacons to use. 2019 was nice because it had a lot of retroreflective targets all over the field.  There's no guarantee that 2020 will be as kind. My general idea is to use the work done on neural-net based object detection to identify and track various landmarks on the field. This could be anything which we can detect and that doesn't move ... graphics on field elements, the gates for getting onto the field, anything.  

This will provide several (potential) benefits
  1. Different classes of beacons will reduce some of the "all targets are the same type" problem listed above.
  1. If we find dead spots 1. that is, places were we see no targets 1. we can potentially train the object detector to find new targets which do exist there
  1. With enough targets, we might be able to use a non-depth camera. This would give us just a bearing to each target. But with enough targets, even that might be enough information to localize ourselves in space.

====Testing====

This will obviously need to be tested extensively. We don't have access to a field, so that's not an option.

Luckily, though, for the 2019-based code we just need vision targets at appropriate places.  If we can clear enough space in the lab, we can create something which looks similar to a field just by mounting retroreflective tape on cardboard backing at the correct locations.  Even a 1/4 field mock up would get us lots of useful data.

The other convenient feature is that our "map" of the field is just a list of coordinates of targets. If we don't have enough room, we could simply move the targets where we do have room and update the coordinates in the code.  It should still localize within the new "map".  

Another option would be to create a simple gazebo camera model 1. model the robot location, and using the field of view of the camera report coordinates of beacons which could be seen from that location.  Ideally, we'd add some noise to the sensor readings to make it more realistic.

====Resources====

C++ implementation, not something to duplicate but some things are good references 1. e.g. resample() 
https:*github.com/mithi/particle-filter-prototype/blob/master/kidnapped-vehicle/src/particle_filter.cpp

Another implementation 1. https:*www.cs.utexas.edu/~teammco/misc/particle_filter/particleFilter.js

https:*pdfs.semanticscholar.org/f79f/d5ab86e21eaedfe3c9f9fe1e83df307c3a93.pdf

[Real Time Particle Filters 1. handing cases where update rate of the filter is lower than update rate of the sensor data|https:*papers.nips.cc/paper/2305-real-time-particle-filters.pdf]

ROS package 1. http:*wiki.ros.org/mrpt_localization
http:*wiki.ros.org/bfl/Tutorials/Example%20of%20using%20a%20particle%20filter%20for%20localization%20by%20bfl%20library
