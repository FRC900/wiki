#  Training Neural Nets using nVidia’s DIGITS # 

nVidia’s DIGITS program is a browser-based front end to Berkely’s Caffe neural net library.

There are two basic steps in training neural nets with nVidia’s DIGITS program. The first part of the process is creating a database of training images. The second part uses that database as an input to train the nets themselves. The output of this training process is a set of weights which, when combined with the structure of the net defined during training, can be imported into the team object detection software.

The process is normally iterative. It is rare to get a perfectly working net on the first pass through the process. Instead, an initial database is created. Using this database, a set of networks are trained, each one modified based on the results of training the last one. After a reasonable net architecture is found it is normal to still have some problems with certain types of images. Images representative of those problem image types will be collected and incorporated into a new database. The process then repeats until a reasonably working net is generated.

The rest of this document will describe the details of creating databases and using them to train neural networks. DIGITS is actively under development. Although we’ll try to keep this document up to date, there’s no doubt that some of the finer details of the process of using it will differ from it.

#  Creating Databases # 

Instead of working on large sets of individual images, DIGITS works on databases built from those images. The justification here is that it is much easier to keep track of a few large files rather than hundreds of thousands of individual ones.

A database is created from a collection of images in a specific directory structure. The top level directory is input to DIGITS when the database is created. Subdirectories under that top level define different classes of images. Each subdirectory name is the name of one of those categories. For example, a database created for object recognition might include subdirs named dog, cat, car, house, and so on. Each of those names would be a class of object the net was trained to recognize.

Connect to digits using a web browser. The address will be `%%localhost:5000%%` or `%%localhost:34448%%`. If DIGITS is running on a different machine, use that instead of localhost.

To create a dataset

  - Click on the Datasets tab
  - Select New Dataset -> Images -> Classification

  - You might need to enter a user name. TBD – which one to use

  - Set Image Type to color
  - Image size is discussed below
  - Resize doesn’t matter – the pre-processed inputs are already the correct aspect ratio
  - Training images – set this to the root dir of the training data
  - Pick a name. I like to include image size and date. Eg. `%%D28_20160919%%`
  - Click on the blue create button

The browser will switch to a window showing status. On the left will be a summary of the input parameters selected above. On the right will be status. This includes an estimated time to completion for generating the various parts of the training database. As described elsewhere, the list of input images are split into two groups. The first is training data - images actually used to update the net. The other is a distinct set of validation images. These are only used to test the net to see how good it is. Separating these out prevents the net from seeing the validation images during training and gives a better indication of how good the net will be with images it has never been trained on.

The graphs at the bottom of the page show the size of the various input classes. Typically there will be more negatives than positives. This is simply because there is a lot more variety in the set “not object” than there is in the set of images showing all the variations of a given object. But be careful here - you want the input set sizes to be reasonably balanced. If, for example, there are 1000x more negatives than positives the network could just learn to always guess “not object” and be correct 99.9% of the time. Keeping the negative count to within an order of magnitude or two of the positive count will prevent this issue.

Finally, note that there are options to delete the model and clone it. Cloning is nice for quickly creating both sizes of input images. After you’ve created, say, the 28x28 version, creating the 14x14 database is simply a click on clone, update the size and name, and then create. It is also nice for creating a new database after adding new training data. Click on clone from a previous run, update the name, and go.

Delete is straightforward. The only potential issue is that DIGITS won’t let users delete databases if any models trained from them exist. So typically you’ll have to clean out trained models before deleting the model they were trained from.

##  Which size to use ## 

There are two sizes of images processed in the detection code: 12x12 (called d12) and 24x24 (called d24). The simplest thing would be to pick 12x12 or 24x24 for the dataset size. But it turns out this doesn’t give the best detection accuracy. Instead it seems to work better if the input image is slightly larger and then cropped randomly during training. This seems to help the net be able to detect objects which aren’t perfectly centered. So for d24 nets the input size is 28x28 and for d12 nets the input size is 14x14. A “crop” parameter is added to the net definition to bring this input down to 12x12 or 24x24.

For calibration nets, on the other hand, use 12x12 or 24x24. Since these nets are trained to undo specific shifts they learn to recognize in input images. Randomly adding more shifting to the input would undo this process. Hit create. This will start the process of creating a database from the input directory specified. Now would be a good time to add some notes in the “Notes” field with some more detail on what the input data set was.

#  Training models # 

Neural nets in Caffe / DIGITS are described by a text file, typically named `%%train_val.prototxt%%`. In it, the user describes the inputs, details of the individual layers in the net, and the outputs generated. DIGITS can automate some of this process, and typically the architecture of nets are reused or copied heavily from previous designs.

To train a model:

  - Select New Model -> Images -> Classification
  - Select the correct dataset in the upper left window
  - On the left, select “none” for “Subtract Mean”. This is already done by the image preprocessing, so no need to do it again.
  - Training epochs is the number of times the complete data set is looped through to train the model.



  - For d12, 50 is probably good. For d/c24 125 or so should be enough.


  - Note that using too many epochs isn’t a huge problem. We can grab intermediate epochs and use them if the models, e.g. starts to overfit. The only downside is more epochs = more training time

  - Select a model. Best bet is to reuse one of the working models from zv
  - Select custom network
  - Grab original.prototxt from zebravision/d12, zebravision/d24, zebravision/c12 or zebravision/c24
  - Paste that into the text window - See the notes on visualizing a net below
  - Select a Base Learning Rate



  - This influences how quickly the net adapts to errors. Higher means the net will make bigger jumps to correct errors. This could be good for quickly converging on a solution but it might also completely jump over promising sets of weights.
  - Rates for d12/d24 nets are typically in the .00001 - .0001 range.
  - But that’s just a guideline, so this is typically a prime area for experimentation

  - Below the Base Learning Rate there’s a checkbox for “Show advanced learning rate options”. Select this.

  - What this does is reduce the learning rate as training progresses. The basic concept is that early on the net will have to make bigger jumps to find a sorta-optimal solution. Then as it gets closer, smaller and smaller jumps will be used to fine tune the net.
  - Step size is a % of training to wait between each reduction of learning rate
  - Gamma is how much to multiply the learning rate by at each reduction
  - Click on “Visualize LR” to see how the LR changes during training.
  - I’ve found the defaults to be a bit too aggressive and typically increase gamma to 0.5 or so. But like the learning rate this is something to experiment with.

  - At the bottom, pick a name for the trained model

  - I typically use a prefix indicating size & type of the model (d14, d28, c12, c24).
  - Then the date (20161118)
  - Then some ideas about number of conv and fc layers (6c12c24c_16fc)
  - Then the learning rate (_00025LR)
  - This gives a quick hint of what the model is simply by browsing in DIGITS.



  - Click create

The window will switch to showing training status.

The top left has info on where the trained model is stored. This will be useful later for copying it into the zebravision code. The middle box points back to the input data set. The status on the right tells how much time is left for training.

The graph in the middle shows training accuracy and loss. Accuracy is measured against the validation data and just shows the percentage of images correctly identified. Higher is better, obviously. If the accuracy doesn’t improve into the high 90s%, the net typically needs to be larger. That means increasing either the number of conv outputs or adding more outputs to the fully connected net. Remember that the d12 nets are just a first pass, though, so some inaccuracy is expected there.

Loss is a unit-less measure of how far off the model is from being perfect. This is measured for both the training data and the validation data. In some ways this is similar to the accuracy number, but it can also show some unique information. The biggest issue is if validation and training loss diverge. Validation and training loss should be similar if the net is actually generalizing the input data. But if it is overfitting - that is -learning to exactly identify the training data rather than a general representation of that data - validation loss will be higher. This can be fixed by reducing the size of the net.

Also note that a net can overfit if it is trained for too long. This will show up as validation loss starting to increase after a certain number of training epochs. This is a hint the net is too large, but it can also be worked around by simply using the results of an earlier training epoch.

The validation loss should show a smooth decreasing curve. Typically the training loss will follow the same overall curve but with more noise. This is because of dropout. During training random parts of the fully connected net are disabled. This helps training since it forces all parts of the fully connected net to contribute to the classification. The downside is that each particular training iteration will be run with part of the net disabled - and as such it will be randomly slightly better or worse than normal.

At the bottom is an option to classify individual images. Due to the way we preprocess data outside of DIGITS, this won’t work. The problem is that a normal image will have 8bitvalues from 0-255 (unsigned chars). The processing step in the zv code converts those to floating point values with a different range. Since the DIGITS “classify image” code doesn’t duplicate this conversion the data fed into the net isn’t as expected so the classification won’t work.

##  Visualizing a network ## 

DIGITS has a cool tool for visualizing networks. Any time there’s an input window for entering and editing network descriptions, there should be a visualize button. Click on it to see a visual display of the net.

The top of the net will be input data. Note that the input is the images plus a corresponding label. The image data will pass through the various layers of the network. They will then join back up with each other towards the bottom of the net to see how far off the prediction of the net is from actual reality.

The various stages of the net have boxes showing the parameters of that layer. The connections between the layers show the sizes of the output of each layer. Layers which operate inline will loop their output data back into the source.

This window is good for verifying that parameters match what you expected. They’re also good for making sure that the network layers actually connect to each other. If you have a net where there are disconnected pieces it is much more obvious graphically than looking at the text description.

#  Importing Into Zebravision # 

Once a model has been trained, it has to be made available for Zebravision to use. This is done by copying the model information into the zv directory structure. Notice that under `%%2016VisionCode/zebravision%%` there are 4 subdirs, one for each model type (d12, d24, c12, c24). Each subdir holds the io for that particular model.

To try out a new model you could just delete everything in that directory and copy the new data in. But zv also can switch between multiple model data dirs. Creating a model subdir with an _ and a number adds an alternative. For example, d12_1 will be searched for model data and if found, can be used as an alternative to the default d12 data.

To use this at startup, do

```
./zv --d12Dir=1
```
There are corresponding –d24Dir=, –c12Dir= and –c24Dir= command line options. There are also commands to switch between models in real time: < > for d12, N M for d24, : " for c12 and K L for c24. The choice seems arbitrary but note the keys are all near each other, d12/24 on the bottom row and c12/c24 on the top

There are 4 files needed in a given model subdir for zv to use a trained model : - The preprocessing weights (zcaWeights.zca). These don’t change, so just reuse the correct-sized one from a previous run. - A labels.txt file. This maps the classes of images to a human-readable name. This file will be found in the dataset directory used for a given training run. 1. There’s a link to the dataset info top center on the model status page. Click on it. 1. The status box at the top left will show a dataset directory. This will be the directory holding the file labels.txt. Copy that into the zv subdir. - A file called deploy.prototxt. This is a text file defining the net architecture. It will look very similar to the net entered into the custom net window while training. DIGITS does a bit of postprocessing to remove layers only used during training, though, so it isn’t an exact match. - This is in the “Job Directory” listed at the top left of the model status window.

- A file called snapshot_iter_<html><some numbers></html>.caffemodel. This holds the trained weights for every trainable element in the network. The numbers increase the longer training goes on. Typically you’ll want the highest numbered one. - This is in the same “Job Directory” as above. - If you look at the zv d12 directory as an example, there are some other files which are good to keep around for informational purposes. These are all from the “Job Directory” :

  - original.prototxt : this is an exact copy of what was entered into the custom net definition. This is useful to have to recreate a run if neccssary
  - solver.prototxt : this holds the parameters used to kick off a training run (learning rate, # of epochs, etc)
  - train_val.prototxt : a processed version of original.prototxt with some info added to associate a particular training run with a certain input dataset. Most importantly this points back to the database used for the training run which makes it useful to keep around.

With all of these files copied into a zebravision subdirectory, you’re ready to test out a model.

#Specific tricks we use

##  Which epoch to use? ## 

While training the results will typically converge fairly quickly. Then, the accuracy and loss will flatline so it is hard to know which iteration to use. Luckily zv makes it (relatively) simple to switch between data from different epochs. If you copy multiple snapshot_iter_*.caffemodel files into a subdir, zv can switch between them on the fly. By default it will load the highest numbered iteration.

The keys to do this are the “unshifted” version of the keys used to switch between model directories. For example, to switch between d12 iterations, use , and . keys. The iteration number used will be shown in the status at the bottom of the zv screen. For example, d12_1:19104000 means the data from snapshot_iter_1910400.caffemodel from d12_1 is being used.

That being said, it is typically difficult to see which model is best just by watching a video. The zebravision directory has several Perl scripts which will iterate through each epoch and test against a set of videos. This needs to be documented better than it is in this paragraph, so FIXME.

##  Scale steps to handle pre-processed input ## 

Many of the preprocessing steps work on floating point values - that is, numbers with decimal points and potentially large ranges. The final results have to be mapped to normal image values between 0 and 255 so these images can be used in the normal Caffe/DIGITS training flow. The preprocessing code was written to look at the range of final result values and find a linear mapping which converts those values back into the range of 0-255.

The reason it maps the majority rather than all of the values is because there are always a few outlier pixels with very large values. If the range was extended to include those in the range from 0-255, the vast majority of “normal” pixel values would end up mapped to just a few values and the images would lose a lot of detail. Instead, pixels with really large negative or positive values are clamped to 0/255 respectively.

The net is to make sure the scale and mean values are set correctly in the transform_param {} block. Make sure they are set for both the train-data and val-data blocks. Typically these values won’t change so copy-paste from previous runs are fine.  

The biggest drawback to this approach is that DIGITS "classify one image" option won't work. This is because it doesn't apply the correct scaling to input images for this process as it does for training data. We have other tools which can test individual images so it isn't too much of a loss.