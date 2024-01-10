This page documents the code which generates and optimizes path for the robot to autonomous follow.

Most of the code for path generation is in https://github.com/FRC900/2020RobotCode/tree/master/zebROS_ws/src/base_trajectory

#  Running the Code # 

Like most of our code, the goal is to eventually hook it up to other code to automate the entire process.  However, for testing and understand it can be run as a stand-alone ROS node.

The node can be started using ```rosrun base_trajectory base_trajectory_node```

This will run the code with default values for a number of parameters.  If you'd rather use the values from the config file (zebROS_ws/src/base_trajectory/config/2020_robot_base_trajectory.yaml) run the controller_node path_follower.launch file.

###  Running using rosservice from the command line ### 

The node expects a service call.  The input would look something like this 

```rosservice call /base_trajectory/spline_gen "
header:
  frame_id: "map"
  stamp: {secs: 390, nsecs: 0}
points:
- positions: [0 , 0, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
- positions: [1, -0.05, 3]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
- positions: [3, 1, 0]
  velocities: []
  accelerations: []
  effort: []
  time_from_start: {secs: 0, nsecs: 0}
point_frame_id:
- data:""
- data:"intake"
path_offset_limit:
- {min_x: 0.0, max_x: 0.0, min_y: 0.0, max_y: 0.0}
- {min_x: 0.0, max_x: 0.0, min_y: -1.0, max_y: 1.0}
- {min_x: 0.0, max_x: 0.0, min_y: 0.0, max_y: 0.0}
optimize_final_velocity: false
"```

The header identifies which reference frame the input points are relative to. Internally, all paths are calculated relative to the robot's reference frame "base_link".  But for convienence, users can tell the code that the points being passed in are from another reference frame. This examples uses map as the header frame_id, which means that all points are relative to a fixed map.  The code will transform those into robot-relative points before generating a path.  Aside from being used to generate map-relative paths, this feature could also be used to generate a path relative to the coordinates of objects detected by a camera.  

If the points being supplied to base_trajectory are robot relative already, the header can be left empty and the path gen code will use the points as-is.

For this feature to work, time must be relatively close to the current time. When calling this from another node, either pass through the header timestamp from the data used to generate the path gen service call, or just use ros::Time::now().  When running from the command line using rosservice, attempt to use a time no older than 10 seconds before the current ros time. The current ROS time can be found from the stage sim GUI or through rviz.

The next argument is a list of waypoints the robot should drive through. The positions field is used with 3 entries, for x, y (in meters) and orientation (in radians) in that order.  The first point must be 0,0,0.  Leave velocities, accelerations and effort arrays their default value of empty.  Time from start is calculated by the path gen code, so it can also be left at 0 sec, 0 nsec.

point_frame_id is an array of strings.  If defined, the string defines an additional reference frame for the corresponding entry in the points array.  This is intended to be used to drive something other than the center of the robot over a waypoint.  In this example, the 2nd points entry will be transformed such that instead of the robot base driving over the 2nd waypoint, the intake of the robot will be instead.

path_offset_limit is an array of tuples as shown in the example.  Each entry corresponds to a waypoint from the points array.  Setting these values to non-zero lets the point gen code move the waypoints around to try and find a quicker path.  Note that these values are the min/max that the optimizer will add to the specified waypoint, not a min/max value of that waypoint's coordinate.  In other words, if a point's Y position is 2 and min_y = -1 , max_y = 1, the range of possible values for Y will be between 1 and 3.

Note - use caution setting non-zero values for the first point.  The first point is intended to be the robot's actual starting position, which has to be 0,0,0.  Changing this for a real path will lead to weird results, since the robot will not start where it thinks it should.  The only real use for this would be to experiment with moving the starting position of a path : allow the first point to move, see what the path optimizer finds as the best starting point for the robot, then actually put the robot there and regenerate the path with 0,0,0 and no path_offset defined to run the actual path.

optimize_final_velocity allows the path gen code to vary the speed and direction of the robot's velocity at the last waypoint. This is useful for cases where the robot needs to pass a certain point as quickly as possible with no regard for where it eventually stops once meeting that goal.  In many cases this will mean the robot will be driving at full speed through the last waypoint - make sure there's room for the robot to come to a complete stop afterwards.


The service call returns several things. First off, it has a field for the coefficients for each of the various spline segments. This was used by the serve_point_gen package. Based on testing, this might be phased out if the spline optimization in base_trajectory_node is good enough.

The other field is a ROS [nav_msgs:Path](http://docs.ros.org/melodic/api/nav_msgs/html/msg/Path.html) describing the position of the robot at various points. In this case, there's one entry every 30cm along the arclength.

Additionally, three sets of matlab code are genereated. Look for 3 .m files generated in the directory you ran the base trajectory node in. The first is the initial guess at the spline parameters. The second is the optimized spline information.  The third is the final robot path (position versus real time).  These can be opened in GNU octave (sudo apt install octave) to see various charts.  For the spline plots, The upper left shows the arclength spline - x vs y location.  The top right shows orientation position and velocity.  The remaining graphs show x&y position vs. time, velocity vs. time, acceleration vs. time and jerk vs. time.  The third set of plots shows robot x, y, and theta position, velocity and acceleration vs. time.

####  This is taking forever! #### 

Note - there should be a patch in place to fix this problem. If you still see it, let us know.

Where forever means it takes more than a few seconds to generate a path. What is likely happening is that the path can't be solved given the constraints laid out in the code. The usual suspect is a constraint which tries to keep the robot from straying too far from a straight-line path.  For some sets of waypoints, this makes it impossible to find a path which works. Look for this in particular where paths reverse direction abruptly.

The limit there is a ROS config which can be changed dynamically.  Do `rosrun rqt_reconfigure rqt_reconfigure`, and expand the base trajectory menu item. Look for path_limit_distance, this is the max distance the robot is allowed to deviate from a straight line path between each waypoint.  Change it to something bigger and rerun - the units are in meters, so pick something reasonable.

####  Changing other constraints #### 

The other interesting config values :

  - max_linear_acc : how quickly the robot can accelerate in a straight line (meters/sec^2)
  - max_linear_dec : how quickly the robot can slow down in a straight line (meters/sec^2)
  - max_cent_acc : max centripetal / lateral acceleration, controls how fast a robot can drive in a curved path (meters/sec^2, where 9.8 m/s^2 is 1 g of lateral acceleration)
  - max_vel : max strafing velocity - sqrt(xvel^2+yvel^2) (meters/sec)

#  Theory of Operation # 
##  Motivation ## 

There are times we want to drive the robot to a new place on the field.  If we just needed to drive in a straight line, this would be easy. Particularly easy with our swerve drive setup - just go in a straight line from A to B.

Real life problems aren't quite so simple. In some cases we need to avoid obstacles.  To handle that, we insert intermediate waypoints that need to be hit along the way.  Or we might need to have the robot facing a specific orientation at certain times along the path.  

Additionally, while it is good to go from A to B successfully, the next step is doing it efficiently. FRC tasks are generally cycle -based - that is, the more a task can be repeated during a match the higher the score.  Getting from A to B as quickly as possible improves our chances of completing more cycles per match.

These goals motivate our development of the tools described in this document.

##  Straight-line Paths ## 

As mentioned above, the simplest way to travel between two points is a straight line.  For paths with no intermediate waypoints, this is also the most efficient. 

However, imagine a path with one intermediate waypoint. There, traveling in straight lines will create problems when the path transitions from the first segment to the second.  Specifically, it requires a really sudden change in direction.  In technical terms, this means the velocity is not continuous - there's a discrete jump in the velocity just before and just after the middle waypoint.

In reality, the robot could accelerate really hard and kind of smooth out the turn. That's especially true if the velocity before and after the transition are similar - typically this requires slowing down quite a bit before the turn.  

But that creates several problems. Asking the robot to do more than it physically can will throw the robot off the desired path.  And it adds the potential for tipping or sliding.  And slowing down goes against the idea of getting through the path quickly.

##  Splines ## 

Instead, a common approach is to use *splines* to describe the path. 

Splines are polynomial equations.  They are named according to the degree of the polynomial. Two of the most common are cubic splines, use terms up to $x^3$, and quintic (fifth-order) splines up to $x^5$.

There are many types of splines. The ones we use are called [B-Splines](https://en.wikipedia.org/wiki/B-spline).  The quintic B-spline equation has the form $$ a - x^5 + b - x^4 + c - x^3 + d - x^2 + e - x + f$$  

Typically the $x$ term for splines is really more like $t$, for time. That is, it describes the evolution of the robot's path at various times while driving through that path.

a-f are constants selected to control the characteristics of the curve.  These include things like the start and end point.  Also, depending on the order of the B-spline, they can also fix the velocity (1st derivative) and acceleration (2nd derivative) of the spline at each endpoint.

That last feature helps solve the infinite acceleration problem mentioned above.  Instead of using straight lines between points, the path instead uses a spline for each segment.  The starting & ending position of each spline is specified to match that of the previous / next one. Same is true for the velocity and acceleration.  This leads to a smooth transition between waypoints, at least up to the 2nd derivative. 

The technical terms for this are C<sup>1</sup> and C<sup>2</sup> continuity, meaning the first and second derivatives are continuous.  This is mainly useful for interpreting the papers linked in the resources section.

Another nice feature of B-splines is the simplicity of calculating the velocity and acceleration at any point along the path.  The derivative of the equation has a straightforward analytical solution. Dropping in a given t to that analytical solution gives the velocity and acceleration at that time.

Interested readers can experiment with interactively drawing splines using the web page [here](https://observablehq.com/@mbostock/d3-spline-editor).  The curveCardinal or curveCatmulRom options are conceptually similar to the splines we use - the splines pass through control points as we'd like.
##  Spline-based Paths ## 

Going back to the top, a path is supposed to go from start to end, driving through each intermediate waypoint in order.  This creates a number of path *segments*, one segment between each waypoint.

In a spline path, each segment is described by an individual spline.  The end position, velocity, acceleration values of one segment are the same as the start values for the next segment.  This leads to a smooth path through all of the waypoints.  Aside from lining up at each waypoints, the splines are relatively independent - e.g. they each have a separate set of constants.  This means that varying the set of constants for one segment doesn't change the rest of the path. This becomes important later.

Also note that typically, splines are set up so that each segment starts at time $x=0$.  This makes it such that the same equations work for each segment, and avoids introducing weird $(x-x_{start})^N$ terms into the equation.  Each segment has an associated "start time in the overall path" with it used to translate from total path time to time-within-segment.

Finally, the splines are initially set to run from $x=0$ to $x=1$.  So the mention above of x being a time are kinda true, but they're an arbitrary time through the spline rather than wall-clock time through it.  Calculating an arbitrary to actual time mapping is part of the optimization process described later.

A spline is a function of one variable which returns another single variable, meaning a single spline describes a location along a 1 dimensional path. Astute readers will notice that the FRC playing field is not one-dimensional.  

So in reality, a path is really a collection of 3 splines, one for x position, y position and rotation.  For some parts of the code, these can be treated independently.  For others they can't.  An example of the latter - when figuring max linear velocity, the x and y velocity of the robot need to be combined into one linear velocity. But separating the splines makes things easier in many cases so that's done consistently throughout the code.

##  Spline Heuristics ## 

Picking the start and end positions of splines is simple - it is just the coordinates we want the robot to drive through.

Initial guesses for velocity and acceleration are more complex.  These depend not only on how fast the robot can move and accelerate, but also where in the path they are.  The velocity for a waypoint close to the start of a path might need to be slower simply because the robot can't get up to speed in time.  Same with one near the end of the path - the robot might need to be slowing to hit the final desired velocity.  The velocity and acceleration in the middle of the path is going to depend on how much the direction of travel changes.

Given this complexity, people have developed *heuristics* which give a reasonable starting point for the desired values.

####  What are heuristics? #### 

A heuristic is an educated guess at a solution for a problem. Or sometimes a measure of how good a guess a solution is.  These guesses are based on experience and testing but generally not rigorously proven to be 100% totally accurate for all cases. But they're really effective for getting a simple, reasonably accurate estimated answer to a complex problem - exactly what we have here.

We need to pick velocities and accelerations for each of the intermediate waypoints, so that means two heuristics.

####  Velocity heuristics #### 

For velocity, the direction is chosen to be the bisector of the angle created by the straight lines connecting the previous, current, and next waypoint.  The intuition here is that the direction of the robot at the waypoint is half-way between the directions of the incoming and outgoing straight lines connecting the previous, current and next waypoint. Put more simply, the waypoint is the middle of turning between the previous and next path.

The magnitude of the velocity at this point is based on the distances from the last waypoint and to the next one.  Typically, some percentage of the minimum of those two values is chosen.  Picking a higher velocity will make the turn wider. The robot will be going faster but it will move further away from a straightline path.  Picking a lower velocity will do the opposite. There, the robot will stick closer to a straight line path.  Which is better depends on how close the robot needs to be to the straight line path to avoid running into stuff.

####  Acceleration heuristics #### 

Picking acceleration is less intuitive.  Is is fully described in the papers listed in the resources.  But essentially, the code creates two cubic splines - one between the previous and current waypoint and a second between the current and next waypoint. It then uses a weighted average (weighted based on prev/next path segment length) of the accelerations of both splines at the current point.  

Cubic splines have fewer constants, meaning fewer degrees of freedom.  They can hit specific position and velocity at the start and end, but the acceleration is fixed given those two inputs.  So the cubic splines between the waypoints is curvy but with determined accelerations rather than ones which can be freely chosen in a quintic spline.  The curve of the cubic spline is a reasonable guess at the curve of the quintic spline, so using the fixed acceleration of the cubic splines as a guess for the final quintic splines is reasonable.

Yes, this is hand-waving.  Heuristics are like this sometimes.  There's more rigor in the papers in the reference.

Anyway, following this process we now have positions, velocities and accelerations for each waypoint along the path.  The steps to figure out the constants for each spline connecting the waypoints is well-known (e.g. [here](https://www.rose-hulman.edu/~finn/CCLI/Notes/day09.pdf)). And even better, it is included in pre-written ROS code. All we need to do is pass in an array of < position, velocity, acceleration > tuples and the code generates a spline-based path.  See the initSpline() method in base_trajectory.cpp for details.

#  Code Description # 

##  Basic Operation ## 

The high level goal of the code is to generate a path given the coordinate waypoints input to it. The path should satisfy the goals mentioned in the motivation section above - be drivable given the robot physics, be a reasonably time-optimal way through the waypoints and not diverge too much from a straightline path to avoid obstacles outside the path corridor.  

This is accomplished through the steps described below.

##  Initial Path Generation ## 

This part of the code takes the desired waypoints and generates an initial guess at a reasonably good path.  This function is performed in the code in generateSpline().  The input is a vector of JointTrajectoryPoints. Each JointTrajectoryPoints entry holds the position, velocity and acceleration desired for the x,y and orientation of the robot, and the vector of points holds the requested x,y, and orientation values for each waypoint of the path.

The function is called with all of the velocity and acceleration set to empty arrays. The majority of the function is code responsible for filling in those entries with good initial guesses.  It first iterates through each waypoint filling in the velocity. The velocity is calculated using the method described previously in the Spline Heuristics section. The code then loops through and fills in the acceleration entries for each waypoint, again using the method described in that section.

It then calls initSpline to actually generate the spline segments which fit the position plus guesses for velocity and acceleration.  This data is stored in a Trajectory object, which is a vector of vectors. The outer vector is indexed by x, y or orientation (in that order), and for each particular trajectory[i], the inner vector holds information for each spline segment making up the path.

##  Path Optimization ## 

The initial guess provides a starting point for the rest of the code.  The remaining code tries to optimize the path to improve two things. First, it tries to minimize the time required to drive the path.  Second, it works to keep the path within a corridor not to far from the straightline path between waypoints. The second constraint is there to prevent the robot from moving too far off that straight-line path and into obstacles on the field.

There are several steps involved in optimizing the path.

###  Spline Parameterization ### 

Much of the rest of the path analysis requires knowledge of the distance traveled along the arc-length of the x-y splines versus time.  We don't have this initially. Instead, it has to be derived from the separate x and y splines.

This function is performed in the getPathLength function.  The function performs two major processes. First, it generates a collection of spline segments which approximate the x-y arclength as a function of time.  That is, inputting a time to the spline returns how far along the linear x-y path the robot has moved.  

The second function is using that information to create an array of equally-spaced points along the arc-length.  This is actually an array of times, set up so that each entry in the array is a time used to grab a point from the arc-length spline. Each such point is (approximately) equidistant from the previous point.

Arclength splines are generated by a process called subdivision.  Points are sampled from the x-y arc length and connected using cubic splines between them.  The points are chosen such that the error from the estimate is low enough that it accurately represents the actual x-y curve.

The code using Simpson's rule to estimate the length of part of the x-y path length. It also generates the error bound for that estimate. If the error bound is low enough, the end points is used for the next sample in the path estimate.

If the error is larger than the allowed max error, the segment is cut in half.  Each half is then evaluated by recursively applying the same procedure to the first half, then the second half.  

When this process is complete, it yields a set of points along the x-y arclength that, when connected with cubic splines, will closely approximate the actual x-y arclength.  The code calls initSpline on these points to actually generate the arc length spline function.

The next step is to find the times generating equally-spaced times along that arc.  Here, a modified [binary search](https://en.wikipedia.org/wiki/Binary_search_algorithm) is used.  

The code loops over the position it is trying to find, adding a constant amount each time to generate a list of equally spaced points to search for. The initial binary search for the first arclength searches over the entire path length and returns a time corresponding to that distance.  

Subsequent searches can be sped up a bit. Since we know that the search location is an increasing distance along the spline, the next time can be no less than the time of the previous location.  And since the points are equally spaced, the resulting times found will also be (very roughly) equally spaced. Thus, the initial mid time for the search is the delta between the previous two times found plus some fudge factor.  The goal here is to narrow the search space to the expected location of the next distance being searched for without narrowing it down too much that the time is outside of the initial window.  

Note that the end time is left at the end of the spline.  This means that the desired arc length / time will be found no matter what. But by moving the midpoint closer to the expected location, it (in most cases) bypasses the first few iterations of the binary search and speeds up the entire process.

###  Spline Evaluation ## 

The optimization algorithm operates by trying to reduce the \\cost\\ of the overall spline path.  In this case, the cost is basically a measure of how good it is at matching our requirements. The two requirements described above are time to drive and deviation from a straight line path. The evaluateSpline() function is used to calculate the cost given those requirements.

Both are evaluated on a point by point basis. The points in this context are the equally-spaced arclength points described above.  Sampling at equal arclengths gives a bit more weight to the curved parts of the path, which (handwaving!) is where more interesting things happen. 

####  Distance Cost Function #### 
The distance requirement is relatively simple to describe.  For each sample point, the minimum distance to the straight-line connecting the start and end of the segment is calculated. For most sample points this will be the normal (perpendicular) distance between the point and line segment. In cases where the point goes past the end of the segment, it is just the L2 distance to the end of the line segment $\sqrt{\Delta x^2 + \Delta y^2}$.

The cost from these distances is calculated using a function which treats anything within the desired distance as basically zero cost, and exponentially increases once it is higher than the distance. This means that cost-wise, and path contained in a corridor within the desired distance is more or less equivalent.

The actual function is $e^{25(\frac{|d|}{dMax} - 0.9)}$.  This is summed up for all of the distances measured for each sample point.  Essentially, distance is normalized to a percentage of max distance, then 0.9 is subtracted.  This number times 25 is used as the exponent.  Any numbers much less than 0.9 make this equation close to 0, anything more than 100% of the max distance turn this into a huge number. This way, all distances within dMax of the straightline path add basically nothing to the overall cost.

####  Time Cost Function #### 
The process for figuring out time to traverse the path is a bit more complex. Using defined maximum linear speed along with linear and centripetal acceleration the maximum velocity at each point is calculated.  The minimum time to drive between two sample points is calculated from this maximum velocity.  The total time is the sum of the time between each point.

The max velocity at each point is calculated in three steps.

The first pass takes the minimum of two values. The first value is the maximum velocity minus the velocity needed to rotate the robot at the speed requested from the orientation spline.  The other term is the maximum velocity allowed which given the curvature of the path at the sample point will keep the centripetal acceleration within limits.  

This first pass generates limits based on how fast the robot can move but ignores the time taken to get to those limits. That is, it assume infinite acceleration.  The next two passes over the path refine that using actual acceleration limits.

The second step overall (first of the acceleration-limiting passes) starts at time 0.  It uses the initial velocity defined at the start of the path.  It takes the minimum of the velocity computed in the first pass and the velocity achievable given the linear acceleration limit added to the initial velocity. It iterates through the rest of the sample points in the path in order and continues to model acceleration, each sample point using the previous sample point's velocity plus the robot's acceleration to find the max achievable velocity at every step.

This will generate a set of velocities per sample point taking acceleration into account.  The final step applies that approach in reverse, starting from the end of the path with a specified final velocity. It works backwards through the path modeling the deceleration needed to end up at each minimum velocity.

The final velocity profile array will have the max velocity at each sample point given the constraints passed into the function.  The total time is just the time taken to drive each arclength at that velocity.

The overall cost of the path is then the sum of the time taken to drive the path and the distance cost function.

####  Kinematic Constraints #### 

The time cost function requires that we know how quickly the robot can drive and how much it can accelerate. These values are known as kinematic constraints.  The code supports setting these in several different ways.

The first is a set of global constraints, set via config values.  These are intended to provide absolute limits based on physical limits from the robot itself. For example, the absolute max speed of the robot would be defined here.

The other set of constraints allows users to add constraints to parts of the path. For example, imagine we wanted to limit the robot's speed at in a certain part of the path.  The user can pass in extra local constraints as part of the path.  These constraints include a set of coordinates where the constraints apply along with the values for those constraints. 

Multiple local constraints can be specified, and they may overlap if needed.

The actual constraints used by the time cost function will be the minimum of all that apply given the current position in the path.

###  Optimization ### 

The last step of the code is to change the path to find the minimum cost possible (or at least a good enough cost - there's always a tradeoff between code run time and the optimization quality).  

It does this by varying some of the initial parameters of the code.  Based on the papers listed in the reference section, there were three which had the largest impact

  - offset in the x direction from a waypoint (currently not used)
  - offset in the y direction from a waypoint (currently not used)
  - offset to the guessed tangent length (velocity magnitude) at each waypoint
  - offset to a "curviness" factor applied to the guessed tangent length

Offsets in the x and y direction allow the code to reduce the curvature of the path at those points, potentially allowing more speed through them.  Note that this offset doesn't move the waypoint coordinates used to calculate the distance offset cost. This means that there are still limits enforced to how far off the ideal straightline path the robot can drive. It just relaxes the requirement that the robot drive directly over each waypoint along the way.  Note that this is currently disabled in the code - we want to test hitting each waypoint exactly for now.

The last two bullets allow the optimization code to vary how curvy the path is.  This can either pull the path back within the straight-line distance range if needed, or alternatively push it out a bit closer to the limit to reduce the curvature at each waypoint.  This allows a tradeoff between distance traveled and the speed at which the robot goes around corners.

The code is not allowed to modify anything for the start and end waypoints.  The starting one is fixed because the robot starts where it starts - that's a hard constraints on the actual location of the robot.  The end point is fixed so the robot arrives at an exact point rather than a range of points somewhere near the desired one.

This means if there are N total waypoints including start and end, there are $4*(N-2)$ total parameters available to optimize - 4 each for everything except the start and end waypoints.

A modified path is created by passing in an array of offsets values to the generateSpline() function. Those offsets are added to the appropriate parameters when the spline is generated.

The optimization method used is called [RPROP](https://florian.github.io/rprop/).  This is a gradient descent algorithm which uses only the signs of the gradient to optimize the cost.  A gradient descent algorithm uses the change in cost resulting from a change in an offset to generate future changes for those offsets.  By tracking the change in output related to the change in a specific input, it is roughly approximating the [partial derivatives](https://www.khanacademy.org/math/multivariable-calculus/multivariable-derivatives/partial-derivative-and-gradient-articles/a/the-gradient) of the cost function w.r.t the input offset values.  This is too complex a function to solve analytically so a rough numerical approximation is required.

The optimization code loops over each of the optimization parameters.  It applies an initial offset to that parameter and looks at the sign of the cost function versus the previous cost.  If the cost is improved, it adds more of an offset. If the cost gets worse, some of the previously added offset is removed.  

If at any point the cost is better than the previous best, the algorithm saves that particular parameter's value and then moves on to optimizing the next value. This helps prevent over-optimizing a single parameter and getting the process stuck in a [local minimum](http://mathonline.wikidot.com/local-maxima-and-minima-and-absolute-maxima-and-minima).  

The optimization of a single parameter will also stop and move to the next if the change in cost goes below a certain limit. This might be the case where a certain parameter is near the best value it can be given the rest of the parameters. But at the same time, the overall combination of parameters doesn't produce and absolute best cost.  At this point, continuing to fine tune that one parameter is likely pointless - no matter how good it gets the overall cost will still be sub-optimal.

RPROP continues to loop through all the parameters as long as the cost keeps getting better. That means that parameters will be revisited over and over so long as the cost is still getting better.

Our code adds a small twist to the original RPROP algorithm. If a pass over the entire set of parameters doesn't give any improvement to the cost, the change in cost function required to move on to the next parameter is reduced. The hope here is that initial passes with a large value will get somewhat close to an optimal value, and then subsequent passes through the parameters will fine tune the cost even more.  At some point, this will have diminishing returns. Once the deltacost threshold gets below a certain value, the optimization process is stopped. 

##  Final Path Generation ## 

The path we want the robot to actually follow is described by a series of positions combined with the time the robot should pass those positions. This final path is generated using a data similar to that used for the time cost function. The max velocity of the robot is generated at fixed time intervals using the various velocity and acceleration constraints. Then the desired robot position at each of those times is calculated by simulating the robot moving at those velocities for each time step. 


##  References ## 

Most of the code was written from the description in these papers. Note that these use Bezier Splines, which are different from B-Splines. The overall motivation, theory and practice doesn't really change despite that.

[Sprunk2008](http://www2.informatik.uni-freiburg.de/~lau/students/Sprunk2008.pdf)

[Lau, Sprunk, Burgard 2009](http://www2.informatik.uni-freiburg.de/~lau/paper/lau09iros.pdf)

[Gloderer Hertle 2009](http://ais.informatik.uni-freiburg.de/teaching/ws09/robotics2/projects/mr2-p6-paper.pdf)





