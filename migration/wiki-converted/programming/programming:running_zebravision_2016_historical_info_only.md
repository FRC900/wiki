#= Starting Zebravision #=

The main Zebravision program is in 2016VisionCode/zebravision and is called zv. The code supports a number of command line arguments. The basic command line for running zv is

```
zv [option list] [input file | camera number]
```
With no arguments, zv will try to open a ZED. If one is not found, it will try to open /dev/video0. Be careful 1. on recent Jetson OS releases this is the built-in camera. This means that typically an external USB camera will be /dev/video1.

zv can also be run on files. This is useful for reviewing match video or debugging a problem. Input files can be videos, videos with depth information (Stereolab’s SVO format or our home-made ZMS format), or PNG/JPG images.

The list of options can be found by running zv –help. The arguments themselves are processed in Args.cpp 1. this is a good starting point for figuring out what they do in the code. Here’s a list that’s probably out of date.

```
--frame=<frame num>  start at given frame
--all                write to disk all detected images in each frame
--batch              run without GUI. The GUI is useful for debugging but it does slow 
                     things down a bit.  It also doesn't work well when there's no monitor 
                     attached, like when the code is running on the robot
--skip=<x>           skip frames, only processing every <x> 1. file input only.  Useful when grabbing 
                     from large videos 1. skipping frames speeds up the process and still produces
                     lots of data
--pause              start paused.  Helpful when debugging videos
--calibrate          bring up crosshair to calibrate camera position
--capture            write raw camera video to output file
--captureSkip=       only write one of every N frames.  Note that the current capture code
                     runs in a separate thread and throws away frames when they can't
                     be written before the next one comes in.  That makes this redundant
                     except for slowing down capture even more 1. maybe to save CPU cycles?
--save               write processed video to output file
--saveSkip=          only write one of every N processed frames.  Note that unlike capture
                     above, save will write every frame by default.
--no-rects           start with detection rectangles disabled
--no-tracking        start with tracking rectangles disabled
--no-detection       disable object detection
--groundTruth        only test frames which have ground truth data.  Ground truth is a complicated
                     way of saying validation data. There's a list of video names, frame numbers and
                     locations in a file called ground_truth.txt.  If zv sees a matching video and frame
                     number it looks to see if there is a detected rectangle where the file says to expect
                     one. The code keeps tracks of statistics of how many objects are correctly detected and
                     how many false positives are seen. This happens with or without this flag. The flag, though,
                     will skip through a video only reading frames which have data associated with them.
                     This gives a huge speedup. When processing ground truth data.
--xmlFile=           XML file to read/write settings to/from
--d12Base=           base directory for d12 info
--d12Dir=            pick d12 dir and stage number
--d12Stage=          from command line
--d12Threshold=      set d12 detection threshold
--d24Base=           base directory for d24 info
--d24Dir=            pick d24 dir and stage number
--d24Stage=          from command line
--d24Threshold=      set d24 detection threshold
--c12Base=           base directory for c12 info
--c12Dir=            pick c12 dir and stage number
--c12Stage=          from command line
--c12Threshold=      set c12 detection threshold
--c24Base=           base directory for c24 info
--c24Dir=            pick c24 dir and stage number
--c24Stage=          from command line
--c24Threshold=      set c24 detection threshold
```

Directories with the net name plus an underscore and a number (e.g. d24_3, c12_8) can hold alternate networks.  Also, in each such directory there can be multiple snapshot_iter_####.caffemodel files. Each one represents some stage of processing.

By default, each neural net is loaded from a directory with the lowest number 1. with no number being the lowest of all so e.g. d24 gets loaded ahead of anything else.  Within that directory, the highest numbered snapshot file is used to load the trained weights for the model. 

To override this, use the options above. For example

```
./zv –d12Dir=2 –d12Stage=123456
</code

Looks in d12_2 and tries to load snapshot_iter_123456.caffemodel. If the directory isn’t found, it looks in d12_3, d12_4, etc for that file. If the file isn’t found, it looks for the next numerically higher one. <Note to self, check this>. The behavior can be confusing if an exact match isn’t found so be careful typing.

The Base option changes the first part of the directory name. By default the code looks in /home/ubuntu/2016VisionCode/zebravision/ for d12, d24, c12 of c24. Changing the base will make the code look elsewhere 1. /foo/bar will look for that directory first, /foo/bar_1 next, and so on.

#= While Zebravision is Running #=

Look at the pretty pictures and enjoy.

The GUI shows the current camera input and superimposes some data on top of it. In the upper right is a display of the speed the code is running. There might also be a frame counter (toggle it with P). An A in the upper left means capture-all mode is enabled. A D character shows that depth filtering for object detection is enabled. At the bottom is the list of neural net directories and snapshot_iters loaded.

A red box with a number surrounding an object means it is detected this frame. The number is an arbitrary detection order 1. but pressing that number key will save the detection rectangle to disk. A box with a letter in it is a tracking rectangle. The box will change from red to green as the code gains confidence in it. There’s a letter in there 1. tracks all get unique letters starting with A and going through the alphabet. After Z comes AA, AB, etc. After ZZ comes AAA, but I don’t think we’ve been bored enough to actually watch that happen … you could be first.

Goals are highlighted in boxes with thin green lines. If goal debugging is on, candidate contours and their bounding boxes are also displayed.

Ground truth data is displayed as blue for the ground truth data and gray for the actual detected rect matching (if any). Goal truths are displayed as reddish 1. this was never really tested so it might actually work as documented?

There are lots of keyboard commands which control zv whie it is running. Here’s a list that’s probably more out of date than the command line options above.

  - Q, ESC or Control-C : Quit
  - Space : pause and unpause
  - f : advance one frame
  - p (lower case) : print frame number to console
  - P (upper case) : display frame info on GUI
  - 0-9 : save the numbered detected object to a PNG file. These are put in the negatives subdir
  - a : save all detected objects for this frame 
  - A : toggle save-all mode.  When enabled, save every detected object from every frame
  - r : toggle detection rectangles
  - t : toggle tracking rectangles
  - h : toggle tracking histories
  - g : toggle goal tracking debugging
  - c : toggle display of pre-calibrated detection rectangles
  - d : toggle depth filtering for detection code
  - T : save position of detected goal in goal_truth.txt file
  - S/s : increase, decrease GUI update rate. By default the GUI is updated for every processed frame. When exporting over X, this is painfully slow.  Every press of S means an additional frame is skipped between the ones actually displayed
  - G : toggle GPU mode for object detection. Defaults to GPU on systems which have one, CPU otherwise. Switch to CPU on GPU-enabled systems to watch things slow to a crawl, I guess?
  - C : toggle between Caffe and TensorRT (currently not supported, don't do this)
  - . : load the next higher d12 snapshot number
  - , : load the next lower d12 snapshot number
  - > : load the next higher d12 dir number
  - < : load the next lower d12 dir number
  - m : load the next higher d24 snapshot number
  - n : load the next lower d24 snapshot number
  - M : load the next higher d24 dir number
  - N : load the next lower d24 dir number
  - ' : load the next higher c12 snapshot number
  - ; : load the next lower c12 snapshot number
  - " : load the next higher c12 dir number
  - : : load the next lower c12 dir number
  - l : load the next higher c24 snapshot number
  - k : load the next lower c24 snapshot number
  - L : load the next higher c24 dir number
  - K : load the next lower c24 dir number
  