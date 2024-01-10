We’ll be using the grab_chroma tool to extract sample images of objects filmed against a green-screen. Try a sample video from [2015 Recycling Bins](https://drive.google.com/open?id=0B8hPVHrmVeDgcVlaWk1mVE1YNE0). This directory contains several videos of green recycling bins shot against a purple chroma-key background.

##  Create a list of background images. ## 

The eventual output will superimpose the objects onto randomly selected background images. These are also known as negative images since they don’t contain the object we’re looking for. Similarly, images which do have the object are known as positive images.

The code randomly selects an image, and from that a random subset of the image. It uses a plain-text file which is just a list of background images to select from. A tar file with some random images can be downloaded from Google drive here : [negative_full_images.tar](https://drive.google.com/open?id=0B8hPVHrmVeDgdEFNb2lfY3pDd0k).

Extract the file using

```bash
mkdir ~/negative_full_images
cd ~/negative_full_images
tar -xvf negative_full_images.tar
```
Then use the Unix [find](http://man7.org/linux/man-pages/man1/find.1.html) command to build a list of valid images in that directory

```bash
find ~/negative_full_images -name \*.png > ~/negative_list.txt
```
This will put the names of all of the PNG image files found in ~/negative_full_images into the file ~/negative_list.txt.

##  Build and run grab_chroma ## 

```bash
cd ~/2016VisionCode/grab_chroma
cmake .  && make
mkdir images
./grab_chroma -o images --maxxrot 0.2 --maxyrot 0.2 --maxzrot 0.5 --max 35 -f 50 -i 5 --bg ~/negative_list.txt bin11-19-15-18.avi
```
This command parses the video bin11-19-15-18.avi looking for frames which have an object surrounded by a purple chroma-key background. Replace this video name with whichever you picked. The tool also supports multiple videos or wildcards (e.g. bin11-19-15-*.avi) but for now work with just one video to speed things up.

From those valid frames, it picks the clearest 50 frames (-f 50) For each of those frames, it generates 5 sets of output images (-i 5) Each image is rotated up to 0.2 radians in x, 0.2 in y, 0.5 in z Each image is also resized a random amount between 100 and 135% of the original size.

The chroma-key values used are defined at the top of grab_chroma.cpp. There should be settings for a purple screen in there. The values might not be perfect since they change depending on lighting conditions and other factors. A tool is being written to tweak and test these values - FIXME when done.

Each output image is superimposed onto an image randomly selected from the list of images in the –bg argument text file.

The output goes into the directory images. The files in that base directory will be used as input to train detection nets. The files under images/shift will are shifted and resized versions used to train calibration nets

Note that you will see occasional “Rectangle out of bounds!” messages. This is normal  - it means that when resizing an image the new bounding box went off the edge of the image. That is, the rectangle the code wanted to use is too big to fit on the image. In that case the code will just ignore the image and automatically move on to the next one.

You can look at the images in the images subdirectory. They should all be various copies of recycling bins from the video. You can see how the code randomly resized and rotated them. You might also notice subtle changes in color and some random noise added to the images. This is a basic form of data augmentation - making small changes to input data to multiply how many training images we get from a single video.

##  Processing the images ## 

The next step is to process the images. The theory behind this is described in a separate [Image-Preprocessing](page), but the basic idea is to apply some basic transformations to make it easier for the neural nets to identify objects. The same transforms applied to the training data are also applied by the detection code in real time to input read from the camera - that makes the actual input match up with training data.

First off, create a directory structure to hold images the images once they’ve been extracted.

###  Preprocessing positive images ### 

Copy the detection net images to a subdir under bin:

```bash
mkdir -p ~/CNN_DEMO/bin/data1
cp images/*.png ~/CNN_DEMO/bin/data1
```
Note that you could preprocess images directly out of the images output directory, but I like to keep results of separate runs in their own directory. This lets me come back later and re-do some of them if we find problems.

Move to the new directory and create a list of images to run through preprocessing :

```bash
cd ~/CNN_DEMO/bin
find . -name \*.png > filelist.txt
```
Create output directories for the post-processed images

```bash
mkdir -p ~/CNN_DEMO_POSTPROC/bin
mkdir -p ~/CNN_DEMO_POSTPROC/negative
```
Process images :

```bash
/home/ubuntu/2016VisionCode/grab_display/zcarun /home/ubuntu/2016VisionCode/zebravision/d24/zcaWeights.zca filelist.txt ~/CNN_DEMO_POSTPROC/bin/
```
zcarun is the program name Use zcaWeights from the current ZV d24 definition to process the images. These are the weights used to generate 24x24 images for the detection net. Typically we only preprocess 24x24 images and then let the DIGITS tool resize them for 12x12 nets. Process each image in filelist.txt Put the results in ~/CNN_DEMO_POSTPROC/bin

Two numbers are displayed at the end of the run. These are the multipler and adder values used to scale the data values to fit in a 0-255 range. They’ll be seen again in the mean and scale options when inputting a Caffe net definition.

###  Preprocessing negative images ### 

Now generate some negative images. You’ll need to grab some directories with a bunch of random images. Luckily we did this in the first step above. The GenInitNegFromFile tool will grab random subimages from those to use as negatives.

Alternatively, the framegrabber tool can be used to grab a number of frames from any video input.

```bash
cd /home/ubuntu/2016VisionCode/framegrabber
```
  - edit GenerateInitNegFromFile.cpp. Look at the calls to GetFilePaths and set them to the path(s) containing negative images you wish to sample from. These should be the various subdirectories of ~/negative_full_images.


  - Set oFolder = “~/CNN_DEMO/negative/generic”
  - Make sure nNegs is a reasonable number to generate. A count 5-10x larger than the positive image count is a good place to start

Make a directory for the code to write generated negative images to :

```bash
mkdir ~/CNN_DEMO/negative
```
Compile the code using “make” to pick up these changes, then run the ./GenInitNegFromFile. The negative images will be put in ~/CNN_DEMO/negative/generic.

These new negatives will have to be preprocessed using a similar set of steps as for the positive images :

```bash
cd ~/CNN_DEMO/negative
find . -name \*.png > filelist.txt
/home/ubuntu/2016VisionCode/grab_display/zcarun /home/ubuntu/2016VisionCode/zebravision/d24/zcaWeights.xml filelist.txt ~/CNN_DEMO_POSTPROC/negative/
```
Note that the ouput dir on the last command is changed to point to the negative images directory.

At this point, ~/CNN_DEMO_POSTPROC can be used to create an image database in Caffe. If additional images need to be added put them in the CNN_DEMO dir in a new subdir under either bin or negatives. Use “find” to create a list of the new images and then run zcarun as above to generate the processed versions of the images.