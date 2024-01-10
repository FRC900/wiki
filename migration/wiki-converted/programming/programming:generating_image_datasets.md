#  Creating image data sets # 

Neural nets require a large number of images to train. This document describes several methods and tools used to generate that quantity of data.

##  Image Sources ## 

There are two main sources of images. The first set of images are those automatically extracted from videos shot against a “green” screen / chroma-key setup. Since this is a controlled, known setup, code can automatically find objects against the green screen and extract them. This is typically the first source used when generating training data - with it a large amount of data can be created relatively easily.

The second source of images is hand-marked objects in frames of videos. Typically after training a neural net with automatically extracted images there will still be some difficult cases the net can’t handle. At this point users need to go through individual videos and mark/extract the images from those difficult frames.

##  Image Sets ## 

There are two types of image sets. The first is those used to train detection neural nets. These images are tightly-cropped images of the object.

The second is used to train calibration nets. Calibration nets are trained on a set of 45 shifted and resized copies of the input images. From these resized/shifted versions of the original input the calibration net learns to correctly center and size each input image it is given.

Tools exist to automatically generate each type of image set from each type of image source.

###  Automatic Image Extraction ### 

The tool used to automatically generate images from a green screen / chroma key video is called grab_chroma, found in the grab_display subdirectory. The tool scans through a video and finds the clearest, most in focus frames which also have objects in them. Objects are detected as non-chroma-key-colored objects in the middle of a large block of chroma-key-colored pixels.

For each frame, grab_chroma outputs a number of slightly modified images of the detected object. The colors are changed slightly, rotated randomly, noise is added and the bounding box is randomly resized. Finally, the image is superimposed onto a randomly chosen background image.

By default, images are generated for both detection and calibration nets. For each image generated for training a detection net, 45 shifted and resized images are generated for each of the 45 shifts and resizes the calibration nets can correct for.

There are several options to set before running grab_chroma. The first selects the range of colors expected for a green screen background. The -r <html><median></html> <html><threshold></html> command line option is used to select them at run time. Each argument is a 6 digit hex value (RRGGBB) - median is the midpoint and threshold is the range above and below the midpoint recognized as a valid background color. A number of choices are hard coded in the source code as examples - if these are set correctly then the value shouldn’t need to be set on the command line.

Two more options control how many images are generated from each input video. -f <html><frames></html> controls how many frames are extracted and -i <html><images></html> controls how many output images are generated per frame. Each argument is an integer value. For each image specified in -i, several are actually output, each with shifted hue values.

–min and –max control the min and max reszie of the bounding box around the object. The final bounding box is selected from a random value between min and max. Setting both to 0 will mean the output images are tightly cropped around the object. Larger values introduce more of a border. A typical run will have –min set to 0 and –max set to 25 or 30.

–maxxrot, –maxyrot and –maxzrot control the maximum rotation applied to each image. The value is a floating point number in radians. For each output image, a random value between 0 and the maximum is chosen for each rotation axis. –bg <html><filename></html> specifies a file with a list of images to use for backgrounds. The code picks a random subset of a random image as the backgroun to superimpose each generated image onto. If no background file is selected, the background is randomized. Use the output of the find command to easily generate a list of files from a set of directories. These images can be anything.

A typical run might look like

```bash
./grab_chroma -f 50 -i 5 --max 35 --maxzrot 3.14 -bg negatives.txt ~/video_dir/*.avi
```
This will grab the 50 best frames from each input video. For each frame, 5 sets of images are generated (each set is 1 frame color-shifted a set number of times). Each generated image is dumped into the output directory for use in detection net training. In addition, 45 subdirs under output subdir the created. Each holds a copy of a given input image shifted and resized a certain amount for training calibration nets.

Generally neural nets are tolerant of some bad training data. Still, that should be minimized. A good practice might be to grab a number of frames but only do 1 image per frame. Inspect the output dir to make sure objects are captured correctly. If not, adjust HSV values for the bad frames and try again. Once things look good, rerun with multiple outputs per frame. The –no-shifts argument would be useful here while testing HSV values - that skips generating shifted versions used in calibration nets which will save a lot of time.

##  Manual Image Extraction ## 

Typically during testing you’ll find some video frames where objects just aren’t being detected. These need to be added to the training data the next time a net is trained. There are several tools used to help this process.

Images are extracted using a tool called imageclipper, found in 2016VisionCode/imageclipper/bin. Start it using ~/2016VisionCode/imageclipper/bin/imageclipper ./video.avi

The ./ is needed before the input video being clipped from if it is in the current directory. The tool uses the path to the video to create an imageclipper output subdir. Omitting the ./ will cause itto try and create the output in the root directory which is generally A Bad Idea.

The tool will start up and show the first frame of the video. It will also show a smaller window which will eventually show the currently selected subset of the image (it should be blank on startup since nothing is selected). Use the left mouse button to select a rectangle. The right button can be used to move or resize the selected area. Drag the middle mouse button to set up a watershed area - the code will try to select similarly colored areas near the cursor.

There are also keyboard bindings for fine tuning the selected location - see the help message that pops up when the program is started for details.

The selection rectangle will change colors depending on how close it is to a desired aspect ratio. Currently all of our neural nets are trained on square images so the selection box will change from red to yellow to green as the aspect ratio gets closer to 1. Since these images will be post-processed to exactly the correct aspect ratio getting the rectangle to yellow is usually good enough.

Once a subimage is selected, hit s to save it. Move forward and back through the video using f and b. Running the program without an input file will show valid command line args. The most useful is -f <html><frame number></html> to start at a certain frame rather than the start of the video.

###  What’s up with the filenames, anyway? ### 

Output images have a long filename. This filename encodes exactly where each image came from so we can find it later. The format is normally

```
<input_video_name>_<frame_number>_<rotation_degrees>_<x_pos>_<y_pos>_<width>_<height>.png
```
Since we were dealing with round objects in 2016 rotation was ignored. This means that sometimes it won’t be in output files.

##  Post Processing Manual Files ## 

There are two tools which post-process manually captures files. One generates files for detection nets, the other generates sets of shifted and rotated images for calibration nets. Each works similarly.

First, the file name is decoded and used to load the video frame the image was sourced from. The file name also provides a bounding box to locate the image. Then the image is post-processed and written to an output dir. This post-processing has several steps. First, the aspect ratio is adjusted to exactly square. Then the image is rotated around its center. Finally, for detection net data the bounding box for the image is randomly resized (this only applies to detection data code).

The utilities are in grab_display. rotate_from_imageclipper is used to generate detection net data while shift_from_imageclipper generates calibration net images. Neither take command line arguments - everything is hard coded to run from the directory where input images are stored.

The normal process for using these utilities is to extract a set of images using imageclipper. Go into the imageclipper subdir where those images are saved and run either of the from_imagecli
pper utilities. Move the output images generated by these tools into the correct subdirectories for positive training images as described below.

#  Image Directories # 

nVidia’s caffe tool is used to train neural nets. The tool trains neural nets using databases created from sets of images in a specific directory structure. The specific directory structure used for our nets is described below.

##  Creating a Detection Net Directory Structure ## 

Detection nets are used to find one or more classes of objects. Training images for each class of objects is put into their own top level subdirectories. For example :

```
my_detection_net/ -----> boulder
                  +----> bin
                  +----> negative
```
Here, my_detection_net is the directory where images are stored. boulder, bin and negative are subdirectories under my_detection_net. This would create three classes of objects to detect - boulders, bins, and everything else which isn’t either a boulder or a bin.

After generating sets of images using any of the methods above, move the generated images into the correct subdirectories. The negative subdirectory can be filled with images of anything which isn’t a member of the other classes. Typically it is easiest to re-use the generic negatives from previous data sets.

If you need to generate new ones, look at 2016VisionCode/framegrabber/GenerateInitNegFrom*.cpp. These tools grab random crops of either random images in a directory or from random frames in a set of videos. The input images or videos can typically be anything that isn’t specific to the current year’s game - random videos are very unlikely to have FRC game pieces in them.

Note that there can be subdirectories created under, say, boulder. Any image in any of those subdirectories of boulder would be considered a boulder image. This makes it possible to separate out images based on their source. For example, images extracted automatically from a certain set of videos could go in one subdir while those hand-extracted from another set of videos could go in another. This might be useful if we’re not sure that added training data will help - put it into a separate subdir so it can be easily removed later if needed.

##  Creating a Calibration Net Directory Structure ## 

Here, the input directory names are 1, 2, 3, and so on up to 45. Each number corresponds to a specific combination of shift and resize values. The output from the tools mentioned above will put output images into these 45 subdirs. You just have to move them into some top-level directory.

The first set of images is easy - simply move 1, 2, 3, … 45 into the desired top level directory. The next set of data is more complicated. You have to move the data from each of the source subdirs into the correct target subdir without deleting the old files. Use the rsync Unix command for this (see the man page for details).

#  PreProcessing Image Dirs # 

The neural net data is run through a preprocessing step called ZCA whitening before training. This process applies a set of transforms to the input images to enhance contrast and make it easier for the nets to train on them. Details are beyond the scope of this paper.

Preprocessing copies from a source dir created above into an output dir. A typical run would be 2016VisionCode/grab_display/zcarun <html><weights_file></html> <html><filelist></html> <html><outdir></html> weights_file is an xml file containing a matrix of weights used to transform the input to the output. Use the zcaWeights.xml file from either d24 (detection nets) or c24 (calibration nets). This way the transform being applied to the training data will be sure to match the transform being applied to live data when Zebravision is run.

filelist is a text file with one input file per line. Right now the code expects to be run from each class directory. Outdir is the output path. This output directory will end up looking very much like the input directory structure. The only difference should be that the files in the output have been preprocessed.

We don’t want to preprocess data twice. Typically this means you’ll have two directory trees. One is where un-preprocessed images go. As new images are collected, they’d be added to this directory. Then, before creating a database to actually train the nets, the images from this directory can be run through the pre-processing step and put into another pre-processed directory. That lateter pre-processed directory would be the one that the Caffe database is made from.

##  An aside about size ## 

The neural nets we use take a fixed-sized input. For running the detection and calibration nets, we use two sizes of input. The smaller 12x12 images are used for a first pass to weed out images which are obviously not the object. The larger 24x24 images are used as a final check to distinguish between more difficult to classify objects.

For training, we use slightly bigger images. The 12x12 nets are trained with 14x14 images and the 24x24 nets are trained using 28x28 images. During training, the code automatically takes a randomly-selected 12x12/24x24 crop of the larger image and passes that through the training. This both increases the amount of training data - each larger image generates several smaller sized cropped images - but it also makes the net able to identify objects which aren’t perfectly centered in an image.

The issue this creates is that there are 4 separate sizes of the data to deal with. To make this simpler, when creating training data we only generate processed 24x24 images. The DIGITS tool automatically resizes these to the desired input size. This is OK because the order of resize and preprocessing doesn’t matter. That is, preprocessing followed by a resize gives basically the same result as a resize followed by preprocessing.

The downside is we lose a bit of information. 24x24 resized to 28x28 won’t be quite as clear as an original 28x28 image. But right now we do not have the ZCA weights generated for 28x28 images so we’re stuck with using those generated for the 24x24 detection & calibration net runs. There’s a TODO to fix this.

#  Hard negative mining # 

As mentioned above, negative images can be any random image which doesn’t contain any of the objects you want to detect. But once we’ve trained a first attempt at a net, we can use this to extract so-called “hard negatives”. These are images which the detector incorrectly identifies as objects.

Using hard negative helps the net learn more efficiently. Instead of learning from random images, we can target specific images which the net has trouble classifying.

Zv has a built-in feature to capture all of the images it detects. If this is used while running over videos which don’t contain examples of positive images, everything detected will by definition be a false positive / hard negative. The command line looks like ./zv –batch –all <html><video name></html> Once the run is finished, zv will have put copies of all of the detected images in the negatives subdirectory. This might be a lot of data. If it is too much, the –skip=<html><count></html> command line argument can be used to skip over frames in the input and only process 1 every <html><count></html> frames. Finding a balance is useful here - you want enough negatives to statistically impact the training without overwhelming the other data. In other words, adding 1 negative to a list of several hundred thousand won’t help much. Likewise, adding a million new negatives to a training set of a few tens of thousands will overwhelm everything else.

##  rank_imagelist ## 

This is a simple utility which will run one of the classifiers over multiple input images. The command line argument should be a test file with a list of input image names. Use find or a similar utility to generate them. The output of this utility shows the confidence that each image is the object the classifier was trained for.

This program has several uses. When run over the positive images, it will identify images from that list which the classifier has problems detecting - look for low scores. This might identify particular types of images where more training data is needed.

Running the utility against a list of negative images will show which ones are classified incorrectly (look for high scores here). These could be added to the negative training data.