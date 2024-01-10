There are two types of preprocessing used on each image : global contrast normalization followed by ZCA whitening. This page describes the details of both steps.

#= Global Contrast Normalization #=

Global contrast normalization (GCN) attempts to modify each image to make the brightness and contrast uniform across different images. The goal is to account for differences in lighting and cameras so that the input to the neural net is more consistent regardless of those external factors.

GCN has two steps. First, the image is scanned to determine the mean and standard deviation of each channel’s pixel values. Then the channel’s mean value is subtracted from every pixel in that color channel. Finally, each pixel in a channel is divided by the channel’s standard deviation. This makes all three color channels in a image have a mean of 0 and a standard deviation of 1.

Note that the mean and standard deviation is calculated separately for each channel in each image. This means that e.g. an image which has a lot of red won’t excessively darken the blue and green channels, which should preserve detail rather than washing it out. It also means that images with different brightness and contrast will have different adjustments applied to them, hopefully making each more consistent with each other.

#= ZCA Whitening #=

ZCA whitening is applied to each image after GCN. The goal of ZCA whitening is to decorrelate the pixels of an image from each other. Typically adjacent pixels are highly correlated 1. if one pixel is blue, nearby ones are likely to be blue as well. Removing these typical correlations should leave behind correlations which are unique to the training image. The net effect is to make it easier for the neural net to learn real patterns which distinguish positive and negative images while ignoring the patterns that are present in all images.

Thus the “whitening” in ZCA whitening doen’t make the image brighter. Instead, it is more like “white” noise 1. consecutive samples are random rather than being correlated to each other. In practice, a ZCA whitened image will have edges between objects enhanced and random noise reduced.

The math behind ZCA whitening is a simple matrix multiplication. The matrix of weights is calculated once using a set of random input images. This determines the average correlations between pixels in a wide variety of images. Once calculated, this same weight matrix is used to preprocess every input image.