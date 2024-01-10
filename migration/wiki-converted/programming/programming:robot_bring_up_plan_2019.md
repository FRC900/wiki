#=Robot bring up plan#=

This assumes talon IDs have been set for all motors.  Refer to TBD sheet for CAN IDs for each of them.  Also, update firmware on all of them at that time (plus check PDP, PCM, etc firmware versions).
#=Swerve base#=

What’s needed : 
  - at least 1 working swerve module
  - Angle motor controllers
    - Percent output
    - Velocity PID
    - Motion magic
    - Position PID
  - Speed motor controllers
    - Percent output
    - Velocity PID
    - Position PID
  - A way to lock angle motors in place
  - For tuning speed motors
    - all 4 corners working
    - Lots of floor space to run the robot

#Angle motors#

Make sure battery voltage stays above 12.5V
For all controllers, set voltage comp on so measurements reflect how we’ll have the robot configured on the field
Reset any conversion factors to 1:1, which might be different than last year’s bot

====Set sensor phase, get motion magic constant estimates (1 hour)====
  1. Put the robot on the floor, make sure it is loaded up with weights if all mechanisms aren’t installed yet.  Put on bumpers, too (or add more weight to simulate them being there).
  1. Load a percent output controller with sensor_phase: false set
  1. Run the motor forward at a reasonable speed
  1. Make sure sensor values are increasing. If they’re going down when % output is positive, sensors are out of phase with the motor output, change sensor_phase to true
  1. Verify sensors increase with positive output
  1. Copy this setting to all other angle motor config fields
  1. Set inverts on drive motors 
  1. Run the motor at 40%, 60%, 80% and read the steady state velocity for each 1. we’ll use this for the next step.
  1. Run the motor at 100%, read the steady state velocity. This will be used as an upper bound for motion magic cruise velocity 1. but we’ll probably want to set it closer to the 80% number from above to give us some margin to work with.
  1. Use either rqt_plot to record a transition from 0% to 100% output.  Calculate the slope of the velocity plot to estimate motion magic acceleration
    - Note 1. this would be something that could be coded up. 
      1. Stop the motor
      1. Set the motor to 100% output, record the start time
      1. Keep track of velocity, when it stops increasing (probably want some %error allowed here) record the time. Calculate acceleration.
      1. Set motor back to 0%, again record the time, get the decel number. Hopefully it is similar to the accel number. If not, we’ll need to use the lower of the two
      1. Repeat a few times.  We should probably pause a few seconds between this to avoid overheating the motors?

====Get Velocity feed forward term (45min)====

Note 1. we have had reasonable luck getting a feed forward term directly in motion magic mode. Set PID to 0, adjust F to get a good match between requested and actual peak speed.  

  1. Put the robot on the floor, make sure it is loaded up with weights if all mechanisms aren’t installed yet.  Put on bumpers, too.
  1. Load a velocity PID controller for the motor.  Make sure sensor phase was copied from the previous step. Also make sure dynamic_reconfigure: true is set.  Rosrun rqt_reconfigure.
  1. Launch rqt_plot, add setpoint and velocity for that talon channel to the chart. The goal is to use feed forward to get the desired speed to match actual speed.
  1. Verify PID slot 0 is selected, and make sure all PIDF constants are set to 0
  1. Command the motor to run at the velocity read for 60% above.  Since all PIDF constants are zero, the motor won’t move
  1. Using rqt_reconfigure, increase F term until the reported speed == setpoint.
  1. Repeat for motor speed 40% and 80%.  Hopefully very similar Kf terms will work for all 3.  TBD which speed to tune for if not 1. keep track of them, and we’ll figure out what the cruise velocity is for the angle motors to try and pick which Kf values is closest to the actual operation of the robot.

====Tune motion magic (1.5 hours)====
  1. Update the motion magic controller with Kf calculated previously
  1. Set accel, cruise velocity to something conservative to start 1. half of the calculated values?
  1. Put the robot on the floor, make sure it is loaded up with weights if all mechanisms aren’t installed yet.  Put on bumpers, too.
  1. Start the controller and rqt_plot.  Useful values include setpoint, active trajectory position and velocity, and encoder position and velocity.
  1. Start tuning PID.  I’d guess we’d want to tune steady state behavior first 1. so setting the motor to rotate 10 or 20 times would be a good start.  Hopefully steady state behavior will be reasonable if Kf is correct, so a simple PD controller would get us very close.
  1. Increase config items for cruise velocity and acceleration.  Find the point where the motor can’t keep up, then back down both values by 10-20% to give some margin for error.
  1. After the PIDF values look reasonable for steady state behavior, test them over short rotations.  This will be the more common case, but it might be harder to tune using them because we might not ever get to steady state.  If so, this may point to needing a different gear ratio, trading accel for top speed (a top speed we’ll never hit isn’t a useful one)
  1. Copy the correct PIDF and motion magic constants into the main talon config for the swerve drive controller angle motors.  Figure out which PID slot is used by the controller for motion magic mode, use that one

====Tune position PID (1 hour)====
Note 1. position mode for the drive angle motors is only used for our build-in motion profiling code, which isn't actually used. We can skip this unless we decide to bring it back to life.

  1. Load up a position PID controller. Make sure voltage compensation is enabled at 12.5V.  Also have dynamic_reconfigure set.  
  1. Load rqt_plot. Useful signals might be setpoint and position
  1. Tune position PID.  I don’t think Kf is used for this, so it’ll be a pure PD controller, maybe with a small I term?
  1. As above, might be easier to tune using large motions, then later testing on smaller transitions
  1. Copy the resulting PID tune to the correct slot in the main talon swerve drive config.
  1. Drive motors
  1. Get drive motor offsets (30 minutes)
  1. Put on blocks / on the cart
  1. By hand, turn all the wheels to point to the same direction
  1. Dump the swerve offsets into a yaml file
  1. Set invert, phase (45 minutes)
  1. Put on blocks / on the cart
  1. Load a controller which locks all 4 angle motors with the wheels pointing the same way.
  1. Run each motor in turn. Or make use of the “dont_set_angle_mode” service in the swerve drive controller, assuming tuning isn’t too terrible for PIDF values.  Make sure positive drives the robot forward for each.  If it doesn’t, then set invert: true in the config for that motor
  1. Verify that the encoder values for all wheels increases when running with a positive % output setting.  If not, reverse the sensor phase setting for the offending motors

====Tune velocity PID (1 hour)====
  1. Put the robot on the floor, make sure it is loaded up with weights if all mechanisms aren’t installed yet.  Put on bumpers, too.
  1. Load a velocity PID controller for all 4 drive motors.  Make sure sensor phase, invert was copied from the previous step. Also make sure dynamic_reconfigure: true is set.  Rosrun rqt_reconfigure.
  1. Launch rqt_plot, add setpoint and velocity for a talon channel to the chart. The goal is to use feedforward to get the desired speed to match actual speed.
  1. Verify PID slot 0 is selected, and make sure all PIDF constants are set to 0
  1. Command the motor to run at the velocity read for 60% above.  Since all PIDF constants are zero, the motor won’t move
  1. Using rqt_reconfigure, increase F term until the reported speed == setpoint.
  1. Repeat for motor speed 40% and 80%.  Hopefully very similar Kf terms will work for all 3.  TBD which speed to tune for if not 1. keep track of them, and we’ll figure out what the cruise velocity is for the angle motors to try and pick which Kf values is closest to the actual operation of the robot.
  1. Then start tuning PID.  If feedforward is close, a simple P or PD controller might be good enough.

====Tune position PID (1 hour)====

Note 1. position mode for the drive speed motors is only used for our build-in motion profiling code, which isn't actually used. We can skip this unless we decide to bring it back to life.

  1. Load up a controller which sends the same position to all 4 motors. Make sure voltage compensation is enabled at 12.5V.  Also have dynamic_reconfigure set.  
  1. Load rqt_plot. Useful signals might be setpoint and position
  1. Tune position PID.  I don’t think Kf is used for this, so it’ll be a pure PD controller, maybe with a small I term?
  1. As above, might be easier to tune using large motions, then later testing on smaller transitions
  1. Copy the resulting PID tune to the correct slot in the main talon swerve drive config.

Maybe tune navX position PID?

#Elevator controller#

    Would be useful to have a velocity PID controller and %% output controller ready for testing

====Test sensor phase (30 min)====
This can be done with the robot disabled
Monitor talon states. Move the elevator up by hand.  Make sure talon position value increases.  If not, reverse the phase: setting in the config

====Test zeroing (1 hour)====
  1. Pull motor power leads 1. this can be tested by moving the elevator by hand, hopefully
  1. Put the elevator at the bottom, enable the robot (test 1)
    1. Should see a “hit limit switch” ROS info message
    1. Verify that the talon is now in motion magic mode
    1. YES works yeah woo
  1. Restart robot code, move the elevator up a bit.  Enable the robot, push the elevator down. Should see the same message at the bottom (test 2)
  1. Have a rostopic echo of talon_state running, see that there’s a time before the elevator hits the bottom that output is -X% (see the config file for the /value).  
    1. Again check that talon is in motion magic mode
    1. YES this works wooo
  1. Restart robot code, move the elevator up a bit.  Enable the robot, make sure the elevator doesn’t move
    1. After a timeout (see config file) the Talon should be in Percent output mode but with a setpoint of 0. This is to prevent the motor burning out if the elevator gets stuck or the limit switch fails
    1. Pushing the elevator down to trigger the limit switch should still zero the encoder and switch to motion magic mode 1. test this
    1. All of this works
  1. Plug the motor wires back in one by one, repeat test 1 and 2 above. 
    - Note : since there are 4 motors, this should be tested with each motor individually.  It is possible motors are wired backwards, the ones on different sides of the robot can move in different directions, etc.  Setting them individually will ensure that none of the motors are driving against each other.
    - Note 2 : you might have to increase the % out config setting to get the elevator to move at all.  Do this in small steps.
  1. Again, verify the ROS info messages and talon states.  Be sure to disable as soon as the elevator hits bottom, just in case there’s a problem.
    1. For test 2, the elevator should move itself down. If it moves up, invert is set incorrectly. 
    1. This will also test what an appropriate %output is to drive the elevator down to zero it. Conservative is good, for now.
    1. All right motors are invert = false, all left motors are invert = true
  1. Set conversion factor (30 min)
    - Idea here is to set the conversion factor so that the setpoints and position readings are in meters rather than radians. The conversion factor from mechanical is in the config file.  
    - This can be done with the robot disabled
    1. Set it, move the arm by hand, make sure sensor values look like meters (possibly put a piece of tape 1m up or something)
    1. Might have to invert that value to get it correct 1. should be obvious by looking at the position sensor value
    1. This is 0.02959
====Set top softlimit (45min)====
  1. Let the controller zero the sensor.  Disable the robot.
  1. Move the elevator by hand to the top softlimit position. Read the position value, use that to set the softlimit config items.
  1. Use a percent output controller to slowly run the elevator to the softlimit position. Make sure it stops
  1. This is 1.6675558772 btw
====Grab estimate of arb FF (30min)====
  1. Possibly add a small bit of weight to simulate a game piece?
  1. I’m not really sure about this one. If the elevator naturally moves down when no power is applied, it should be similar to the other arm 1. find a % output which holds it steady ( this would require changing the code to add the ff term all the time instead of just when it is moving up?)
  1. If the arm doesn’t move down when the motor is disabled, perhaps find the % output needed to go from 0 motion to barely creeping up?
  1. Arbitrary feed forward is 0.041
====Get motion magic velocity and acceleration (1 hour)====
  1. General idea is the same as other mechanisms 1. run the motor at 100%, measure velocity, acceleration.  Problem is the limited amount of space to run it. Start small 1. 20% or so and work up slowly.
  1. Conservative is fine here to start with 1. if we can get reasonable numbers at 50%, stop there. We can always increase them in the later parts of tuning motion magic, but here we’re just trying to get something that’s a reasonable starting point.
  1. Remember at this point, measurements will be in m/s or m/s^2 due to the conversion factor. The rest of the code should be consistent with units, but verify if possible in future steps
  1. Maximum velocity = 1.7 meters/sec, we ran max velocity at 1.2
  1. Maximum acceleration = 7.5 meters/sec^2, we ran max accel at 6
====Get Kf (45 minutes)====
  1. Use a velocity PID controller.  Idea here is the same as other controllers, but with the limitation that the elevator can only move so far.
  1. F term is 0.18
====Tune motion magic (1.5 hours)====
  1. Go back to the regular elevator controller
  1. Follow the steps for tuning motion magic PID
  1. Here, I’d start conservative.  Get it working reasonably well over small distances, then gradually increase the range of motion.  The goal here is not to have it slam into either the top or the bottom.
  1. And realistically, we only have to make it work between 4 different points...
  1. My thinking 1. I’d love to have the arm move at peak accel / velocity between any setpoint. But if in the process of getting there we break it two or 3 times, we’ve lost way more than we’d ever gain compared to it being 75% of optimal.  So conservative is good, at least until we understand how it behaves.

#Cargo Intake Controller#

The first pneumatics-related system we test is also going to check for leaks, the compressor running correctly, etc.  Plan extra time for working through those issues.

Main things to test, aside from “does anything move when we command it”
  - Verify both Rio and Jetson yaml files are set up correctly for the solenoid.  Both should have the same name and PCM Id / solenoid channel.  Jetson needs “local_update: true, local_hardware: false”, while the Rio need the opposite values.
  - Intake spinny direction is actually in.  Easiest to test if motor speed is way lower than 100%. I’ve made a task to make the % output a config item.  If the intake spins the wrong way, set invert :  true in the talon config subsection
  - Which clamp direction corresponds to true and false.  Probably good to document so that actionlib code using it knows what’s up

#Cargo Outtake Controller#

  - Verify YAML files for both pistons, as described above
  - Verify kicker, clamp direction makes sense.  I’d run them separately to start, then send service calls with trigger both at once. The last part is more of a test to make sure there’s no mechanical issues we need to code around 1. e.g. parts running into each other


#Panel Intake Controller#

Very similar to cargo outtake controller. Again, verify that convention of true vs. false makes sense

#Climber Controller#

Despite the complexity of the mechanism, from a low level control side this is surprisingly similar to the previous two controllers