No, not a typo. This is about planning for future additions to path planning.

Current path planning is robot-centric - that is, paths are defined using a robot-centric coordinate system.  There's no awareness of where we are on the field, just that the robot needs to move some defined distance from wherever it is right now.

In the future, we could improve this.  The first necessary step would be to know where we are on the field. Localization should provide this. It will give our location in a predefined fixed map coordinate system.  

Using this location plus the map itself, we can gain the ability to path plan around fixed obstacles on the field.  And to go to specific points on the field given an arbitrary starting position.  

The general idea - use ROS's global planner to get an initial guess at an obstacle-free path through the map from start to end position.  Then optimize this path using a modified version of the current spline-based code.

ROS's global planner generates a path on map grid that avoids obstacles. It won't be anywhere near optimal. Among other problems, the global planner will tend to add more waypoints than necessary.

The paper we used as a basis for our spline optimized paths gives hints to connect the two.  The paper says to first take the global path and iteratively remove waypoints.  If removing a waypoint doesn't change the path such that it intersects field obstacles, then leave the waypoint out. Otherwise replace it. Continue doing so, looping over all waypoints until a complete pass through all the waypoints fails to remove any of them.  This should leave only the waypoints necessary to avoid obstacles.

ROS maps are basically just an array. Each entry in the array corresponds to a given x,y location on the map. The value held shows how occupied each coordinate is.  We could use https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm (https://rosettacode.org/wiki/Bitmap/Bresenham%27s_line_algorithm#C.2B.2B) to walk the grid from start to end, checking the occupancy value of each coordinate along the way.  If any cells are occupied, the removed waypoint leads to a collision and must be replaced.

Open question - does the order of removing waypoints matter?  Ideally we want the fewest number of waypoints remaining after this step. Compare that with the complexity of the permutations of removing them in different orders?

Once we have reasonable set of waypoints from the global planner, they need to be run through the spline-based optimizer to generate a path for the robot to follow. The current spline code has no concept of obstacles so that will need to be added.  More specifically, we need to add a penalty to the cost function used by the optimizer for curves which get close to obstacles. We already have something similar used to keep the curve close to a straight line between waypoints. A similar exponential function is likely the way to go for distance to obstacles as well - i.e. near-zero penalty until a path gets really close to an obstacle, at which point the cost should increase rapidly.

Specifically, we need to make sure there's no point on the path that is closer than half the robot width (or robot hypotenuse?) to any obstacle.  Since the path is a polynomial, this should be similar to the question asked here - https://stackoverflow.com/questions/2742610/closest-point-on-a-cubic-bezier-curve.  I'm not sure how slow this would be, so we might to investigate things like https://www.researchgate.net/publication/2393786_A_Fast_Algorithm_for_Computing_the_Closest_Point_and_Distance_Transform?  

Another idea is from https://users.soe.ucsc.edu/~elkaim/Documents/SplineVTC07.pdf - turn the spline into a set of coordinates on that spline and check each coordinate against the map at that coordinate. They "inflate" each obstacle by the half the size of the robot to account for the fact the robot isn't a point. That means that instead of worrying about distance to an obstacle, this simply becomes a binary check for the path crossing the inflated obstacle or not. This is closer to the approach described above for optimizing the global path. We have code to generate a list of equally-spaced points on the spline. The spacing distance would just have to be appropriate for the scale of the map.

I'm not clear if the code will need to add new spline waypoints to work around obstacles. Papers reference it, which means the answer is probably yes. This is frustrating because the first step after getting a global plan was to remove unneeded waypoints.

See https://users.soe.ucsc.edu/~elkaim/Documents/SplineNTM07.pdf
https://www.researchgate.net/publication/286573246_Real-time_obstacle_avoidance_using_subtargets_and_Cubic_B-spline_for_mobile_robots


Concerns

The biggest issue here is if we get this working, to test it we need an accurate field to drive on. That includes a good field map and localization targets equivalent to the actual field.  It doesn't require actual obstacles - if done correctly, the robot will drive around obstacles from the map even if they're not present on the practice field.


