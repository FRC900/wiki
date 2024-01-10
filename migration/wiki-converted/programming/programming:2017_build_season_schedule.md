# Build Season Schedule
**Kept for historical interest - either as a monument to what a build season looks like or to how tough it is to plan a build season before seeing a game (most of the stuff in this schedule never actually happened).**

Let's try to plan out build season. This is going to be a rough outline since we have no idea what the game is.  Still, it helps to get something written down - it is always easier to edit stuff than create it from scratch.

# Key Dates 
- Build Season Starts Jan 7th
- Stop Build is Feb 21st - this is close to irrelevant for programming, though
- CHS District - Southwest VA - Blacksburg, VA, Mar 3-5th (plus travel days?) 
- 1st NC district competition - Greensboro, Mar 9-11th
- 2nd NC district competition - Raleigh Mar 23-25th
- State Champs - Mar 31-Apr 1. Add a day or two for travel
- Houston - Apr 19-22 plus some travel days

# Week by week key work items

This assumes 2017 is similar to previous years in that there's an "easy" vision goal and a hard one.  The easy one will have retro-reflective targets of a known shape and can typically be done using hand-coded detection algorithms.  The harder one is anything else and requires neural nets.  

That being said, who's to say that we can't do the easy one with neural nets as well?

Things get more and more fuzzy the further out we go in time.  1st week is probably a reasonable guess. Later weeks become outright fantasy.  But hey, everyone likes a good fairy tale now and then.  So, Once Upon A Time In Build Season ...

## Week 1 : Jan 7 - 13
- Read and understand game manual
- Work with crystal ball team to define & prioritize vision tasks
- Work with mechanical/CAD to build mock ups of vision targets if needed
- Get green-screen videos of important game object(s)
- Use python and/or existing vision code to prototype retro-reflective tracking 
- Create 2017 Vision Code github repo

## Week 2 : Jan 14-20
- Refine goals & priorities for rest of season
    - Get tasks into Trello
    - Update this page also?  OR leave it as a testament to our optimism?
- Figure out HW requirements (# of Jetsons & cameras, LED ring(s), power, networking, etc), feed them back to CAD/mechanical team
    - This may require a semi-solid robot design which could easily be pushed back a week or so
- Continue prototyping retro-reflective detection ideas
- Start generating datasets for training object detection
    - If we need to detect multiple objects, start to update the code
- Latency compensation work 
   - Mostly this will mean getting an accurate timestamp from the NavX
   - Need a per-input "latency offset" - how much time there is between when a frame is captured and when the code can grab a timestamp.  This will be figured out empirically, which means moving sliders until stuff works.
   - Rest of the work should be on the control side, maybe

## Week 3 : Jan 21-27
- Start "real" implementation of retro-reflective detection
- In parallel, generate testing videos for retro-reflective detection
    - Would be nice to have automated testing here, kinda like nnet ground truth
    - The code for this is sorta-there in zv but needs work
- Start training nnets, iterate on net architecture
- Grab videos of objects in real lighting from various places around the lab & mall. 
    - Generate ground-truth data for these videos (TODO : document this, make it work with ZMS files)
    - Start by-hand image grabbing of tough scenes to generate more training data
    - But don't collect data from a few random videos to hold them back as validation
- Continue NNet code updates for multiple objects, if needed
- Figure out what data to pass back to control system.  
    - Safe bet is ZMQ like last year. ROS is the stretch goal
    - Figure out time stamp format. Right now it is nSec which seems like overkill
- Finish up latency compensation code
    - As much as it can be without having a robot to test on
- Maybe a live demo of something for the sponsor presentation
   - Which is when, exactly?

## Week 4 : Jan 28-Feb 3
- Continue implementing and testing retro-reflective detection
- Hopefully have at least some success detecting objects using nets at this point. 
  - Continue training. 
  - First pass of hard negative mining should be happening here to improve our false-positive rejection
- If we can find a spare controls programmer and drivetrain, maybe practice running into a 2016 ball as prep for object pickup code?
   - Depends on our needs, of course

## Week 5 : Feb 4 - 10
-  Retro-reflective testing should be mostly done
    - Work on breaking it by testing in various conditions
- Neural net code is hopefully working at 60-70% accuracy with ~1 false positive per frame
- We might have a robot now, but don't get your hopes up.

## Week 6 : Feb 11 - 17
- Now we have a robot. It will likely not move much.
    - Have code ready to put on the Jetson just in case
- More neural net training
    - Start looking for onboard (or otherwise) video from other teams to use for testing

## Week 7 : Feb 18 - 24
- Wave as stop build day whizzes by.  
- Should be into full-on testing with the spare parts bot at this point.  
    - Fix what's broken in our code. 
- Start planning demos to wow the judges at competition

## Week 8 : Feb 25 - Mar 3
- 6 week build season you say?
- Week 1 event for us in Blacksburg, VA
- Should be mostly testing of "vital" vision functions at this point
- Still working on stretch goals, though, since we've got another few months to go if we're successful
- Work on competition checklists
    - Stuff to pack
    - Stuff to set up in pits
    - Stuff for demo
    - Plan to use on-field time efficiently
        - Capture videos of field
        - Tune settings on the robot

## Week 9 : Mar 4 - Mar 10
- First competition. 
    - See our enemies driven before us. Hear the lamentations of their women.
    - Capture lots of sample videos for later testing back at the lab
    - Tune stuff as much as possible
    - Use as extended dev time depending on how broken things are 

## Week 10 : Mar 11 - Mar 17
- Post mortem from Greensboro competition
    - Plan what to work on for next 1.5 weeks until next competition
    - Document what we forgot in the various checklists
- Use training data to test and retrain (if needed) detection code
- Try to prevent the mechanical team from re-building the robot from scratch

## Week 11 : Mar 18 - Mar 24
- Second competition
   - Same as the first, hopefully with a better robot
   - Collect as much training data as we can

## Week 12 : Mar 25 - Apr 1
- Post mortem from Raleigh event.
- Hopefully State championships
    - Lather, rinse, repeat

## Week 13 : Apr 2 - 8
- See if we're going to Worlds or not
- Here's where we rebuild the robot from scratch :)

## Week 14 : Apr 9 - 15
- Hand-wavey stuff about last minute coding for worlds

## Week 15 : Apr 16 - 22
- Worlds
    - Don't expect to get much done this week with packing and travel time
    - Then again all we need to program is a laptop, so maybe...

# Wild Cards and TODO
- Random code TODOs already on Trello will fit in wherever there are gaps in the schedule and available students
- We also have Arduinos plus LED strips to play with to fill in gaps
- New software releases (Jetson, ZED, who knows what else) - integrate as necessary
- New Jetson hardware (TX2) - need evaluation & porting time, plus figuring out power, mounting, IO, etc.
- ROS?  
    - This is likely going to be a parallel effort in addition to the other stuff
    - At a minimum we'd get the retro-reflective detection code going
    - And port the nnet code
    - and the object tracking code
    - and then get some sort of EKF-like pose estimation
        - From visual odometry
        - From CAN snooping?
    - Then see how well localization / mapping works.
