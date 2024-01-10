This page goes through a sample neural network definition. It doesn’t attempt to explain how a convolutional network works from the ground up - see [Neural Net Resources](Neural Net Resources) for a reading list.

##  Input Layers ## 

These layers define the input to the network. In this case, the training data is a 28x28 image.

```
layer {
  name: "train-data"
  type: "Data"
  top: "data"
  top: "label"
  include {
    phase: TRAIN
  }
  transform_param {
    scale: 0.015164888
    mirror: true
    crop_size: 24
    mean_value: 127.5
  }
  data_param {
    batch_size: 128
  }
}
layer {
  name: "val-data"
  type: "Data"
  top: "data"
  top: "label"
  include {
    phase: TEST
  }
  transform_param {
    scale: 0.015164888
    mirror: true
    crop_size: 24
    mean_value: 127.5
  }
  data_param {
    batch_size: 128
  }
}
```
The input definition used by Caffe is pretty standard from net to net. Typically all that needs to be verified is that the crop_size matches the size of the network. Here we’re defining a d24 network so the size is 24 pixels square.

There are two basically identical definitions here, differing only name (train vs. val) and on which phase of the training they’re included for (again, TRAIN vs. TEST). When DIGITS generates a training database, it actually splits the data into two sets. The training data is used to actually train the net. It is run through in batches, the error is calculated and then used to update the weights of the net.

The other set is validation data. This is a separate set of data used to independently test the accuracy of the net. This data is never used for training. That’s a better indication of how well the network is doing. A network can overfit (learn the training data perfectly but be unable to identify anything else). Keeping separate validation data helps identify when that happens so it can be corrected.

The scale & mean values are used to convert from unsigned char data (0-255 values) into floating point numbers. The mean is subtracted and the result is multiplied by the scale. These correspond to the values identified during processing. Mean should always be 127.5. The GCN part of preprocessing makes the input data 0-mean. Subtracting a mean of 127.5 makes the data centered on 0 (range is -127.5 to 127.5). The scale value is selected to preserve ~97% of the input range by making it fit exactly to the range of 0-255 held by a byte in the image.

Note that each of these layers outputs data (the image data itself) and labels (text identifying what each image is). The data is used immediately in the next step while the labels are only used at the bottom to check how well the predicted label matches the actual one.

Next up is our first convolutional layer. This is a set of NxN learned filters which operate directly on the input data.

```
layer {
  name: "conv1_d24"
  type: "Convolution"
  bottom: "data"
  top: "conv1_d24"
```
This identifies the name of the layer and type. The “bottom:” field is the input. For whatever reason, nets are traditionally visualized bottom to top. The input shows up at the bottom and data moves up through the net. This “top:” in this case is the output

```
  param {
    lr_mult: 1.0
    decay_mult: 1.0
  }
  param {
    lr_mult: 2.0
    decay_mult: 0.0
  }
```
Learning rate modifiers for this particular convolutional block. The two sets of parameters are for the multiplicative weights and the added bias. The lr_mult is a multiplier applied to the globally-defined learning rate. The learning rate defines how quickly the learned values of the net are influenced by the error between the expected and acutal values of the net during training.

The decay_mult is a multiplier applied to the global decay rate. The higher this parameter is, the more the network penalizes larger weight and bias values. Increasing this value can help reduce overfitting - but if that’s an issue it is probably best to start by increasing it in the global Caffe/DIGITS settings first.

```
  convolution_param {
    num_output: 6
    kernel_size: 3
    stride: 1
    weight_filler {
      type: "xavier"
    }
    bias_filler {
      type: "constant"
    }
  }
}
```
The important parameters for the convolutional block. There are 6 outputs - basically 6 different filters. Since each filter can learn weights separately they will come to identify different features in the input image.

The kernel size is 3x3 pixels. While that might seem like a small receptive field, research has shown that instead of using bigger filters there is more benefit to just adding another layer of 3x3 filters rather than increasing the size of them.

The stride is 1. That means the filters are run across every offset in the input image. A stride larger than 1 would skip over every N pixels - e.g. a stride of 2 would move the filter across the input in steps of 2 pixels.

The fillers identify how the weights are initialized at the start of a run. Since our networks are relatively easy to train these aren’t too important.

```
layer {
  name: "relu1_d24"
  type: "ReLU"
  bottom: "conv1_d24"
  top: "conv1_d24"
}
```
This is an activation function. It is applied to the output of the previous step to introduce non-linearity into the process. In this case, we’re using “Rectified Linear Units”. The function f(x) is 0 if x < 0, and x otherwise. This particular function has been shown to significantly speed up learning rates.

Note that the top and bottom of this are the same. That means that it applies the function in-place to the output of conv1_d24.

Next up, another convolutional layer.
```
layer {
  name: "conv1a_d24"
  type: "Convolution"
  bottom: "conv1_d24"
  top: "conv1a_d24"
  param {
    lr_mult: 1.0
    decay_mult: 1.0
  }
  param {
    lr_mult: 2.0
    decay_mult: 0.0
  }
  convolution_param {
    num_output: 18
    kernel_size: 3
    stride: 1
    weight_filler {
      type: "xavier"
    }
    bias_filler {
      type: "constant"
    }
  }
}
```
The only real difference from the previous conv layer is that it has 18 filters instead of the previous 6.

Each filter here is also 3x3. The interesting part is that instead of having 3 channels (BGR) like the first conv layer, the input here is 6 channels - one for each of the filters of the previous conv layer. In other words, this is doing filtering on the output of the previous filtering step. That lets this deeper layer start to learn more complex representations by combining the results of the 1st layer of simple filters.

```
layer {
  name: "pool1a_d24"
  type: "Pooling"
  bottom: "conv1a_d24"
  top: "pool1a_d24"
  pooling_param {
    pool: MAX
    kernel_size: 3
    stride: 2
  }
}
```
A pooling layer is used to reduce the size of the input. This is a max pool - it returns the largest value in within the kernel size set of pixels (3x3 in this example).

The stride is 2, so it skips over every other pixel in its input. This reduces the amount of data subsequent layers have to work with by a factor of 2 in each dimension.

Dince it grabs the max over a range of pixels, it also adds some “spatial invariance”. That is, shifting a high value over one pixel will give basically the same output from the pooling since it is looking at a range of pixels at once. This means that the net is somewhat insensitive to slight shifts in the position of the input image.

```
layer {
  name: "relu1a_d24"
  type: "ReLU"
  bottom: "pool1a_d24"
  top: "pool1a_d24"
}
```
Another ReLU layer. Same as the previous one. This could be swapped with the pooling layer above for basically no difference. It is here since there are fewer pixels coming out of a pool layer than going in, so there’s a bit less work doing f(max(x,y,…z)) than max(f(x), f(y)…f(z))

A final conv layer is next

```
layer {
  name: "conv1b_d24"
  type: "Convolution"
  bottom: "pool1a_d24"
  top: "conv1b_d24"
  param {
    lr_mult: 1.0
    decay_mult: 1.0
  }
  param {
    lr_mult: 2.0
    decay_mult: 0.0
  }
  convolution_param {
    num_output: 12
    kernel_size: 3
    stride: 1
    weight_filler {
      type: "xavier"
    }
    bias_filler {
      type: "constant"
    }
  }
}
```
Here there are 12 convolutional filters, each 3x3. Remember that each layer works off the results of the previous conv layer, so the filters are 3x3x18 deep.

```
layer {
  name: "pool1b_d24"
  type: "Pooling"
  bottom: "conv1b_d24"
  top: "pool1b_d24"
  pooling_param {
    pool: MAX
    kernel_size: 3
    stride: 2
  }
}
layer {
  name: "relu1b_d24"
  type: "ReLU"
  bottom: "pool1b_d24"
  top: "pool1b_d24"
}
```
A final pool/ReLu combo.

A fully-connected layer. Unlike a convolutional layer where the size is set as a parameter, a fully connected layer takes as input the entire set of data from the previous layer. This is where the output of the full set of filters is being classified into object / not object.

```
layer {
  name: "fc1_d24"
  type: "InnerProduct"
  bottom: "pool1b_d24"
  top: "fc1_d24"
  param {
    lr_mult: 1.0
    decay_mult: 1.0
  }
  param {
    lr_mult: 2.0
    decay_mult: 0.0
  }
  inner_product_param {
    num_output: 12
    weight_filler {
      type: "xavier"
    }
    bias_filler {
      type: "constant"
    }
  }
}
```
Much of this is the same as a conv layer. The big difference is a lack of size or stride parameters. Since the fully connected layer looks at the entire input from the previous layer, these don’t make sense.

```
layer {
  name: "relu2_d24"
  type: "ReLU"
  bottom: "fc1_d24"
  top: "fc1_d24"
}
```
A ReLU layer is added between fully-connected layers to add non-linearity into the function it is computing.

A dropout layer randomly ignores different parts of the outputs of the previous layer for each step of the training. This tends to improve training speed. Instead of a few outputs being used to determine the entire classification this forces all the nodes to play a part.

```
layer {
  name: "drop1_d24"
  type: "Dropout"
  bottom: "fc1_d24"
  top: "fc1_d24"
  dropout_param {
    dropout_ratio: 0.5
  }
}
```
Final fully connected layer. This takes the values from the previous FC layer and using the weights it learns, outputs a single value for each class of object being classified. In this case, since we are a binary (object / not-object) classifier it will have two outputs.

```
layer {
  name: "fc2_d24"
  type: "InnerProduct"
  bottom: "fc1_d24"
  top: "fc2_d24"
  param {
    lr_mult: 1.0
    decay_mult: 1.0
  }
  param {
    lr_mult: 2.0
    decay_mult: 0.0
  }
  inner_product_param {
    num_output: 2
    weight_filler {
      type: "xavier"
    }
    bias_filler {
      type: "constant"
    }
  }
}
```
Used by the test (validation) phase to determine the accuracy of the net when tested against validation data. Note it takes both the output of the net plus labels as input.

```
layer {
  name: "accuracy"
  type: "Accuracy"
  bottom: "fc2_d24"
  bottom: "label"
  top: "accuracy"
  include {
    phase: TEST
  }
}
```
Softmax converts the output of the last fully connected net to a confidence percentage for each class. The sum of these confidences adds up to 100%. The loss part generates an error for the predicted vs actual classification which is fed back into the net to update weights during training. This is boilerplate code which appears in every net and is required to make training work.

```
layer {
  name: "loss"
  type: "SoftmaxWithLoss"
  bottom: "fc2_d24"
  bottom: "label"
  top: "loss"
}
```
