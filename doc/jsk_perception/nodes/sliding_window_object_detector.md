# SlidingWindowObjectDetector

## What is this?
This nodelet performs supervised object detection
through binary support vector machine trained object classifier.

The Nodelet uses a sliding window detection method as a raster scaning of the image.

Currently, the detector is trained on Histogram of Oriented Gradients and HS Color histogram.

To train this detector please use the custom implemented
[jsk_perception/sliding_window_object_detector_trainer_node](sliding_window_object_detector_trainer.md)
node bundled in jsk_perception pkg.

This manifest file contains specific configurations of the trainer,
the feature dimensionalities, the detector window size and the output directories with filenames.

## Usage

The nodelet can be configured to run as either:
* `1) a detector (DETECTOR)`

  The nodelet simply loads the manifest and trainer and performs object detection.

* `2) a bootstraper (BOOTSTRAPER)`

  This mode, is used to refine the trained classifier by accumulating
  the false positive detection in the environment to re-train the detector.

  This method helps reduce the false positives.

  Note when doing bootstrapping make sure that the object of interest is `NOT` in the environment.

  The nodelet before doing bootstrapping will load the negative training bag file,
  read, and will write using the same name and will append the images from bootstrapping to the bag.

  This is done to set all training set to similar time stamps.

![](images/sliding_window_object_detector.png)

## Subscribing Topic
* `~input` (`sensor_msgs/Image`)

  Input image.


## Publishing Topic
* `~output/image` (`sensor_msgs/Image`)

  Raw image marked with bounding boxes of detected objects.

* `~output/rects` (`jsk_recognition_msgs/RectArray`)

  Array of detected bounding boxes


## Parameters
* `~run_type` (string, required)

  Run the nodelet as a detector or bootstraper. See above.

* `~trainer_manifest` (string, default: `sliding_window_trainer_manifest.xml`)

  Manifest file containing training parameters, which is one of outputs of
  `sliding_window_object_detector_trainer_node`.

* `override_manifest` (bool, default: `false`)

  Override parameters shown below after loading `~trainer_manifest`.

  Overridable parameters are ...

  * `~trainer_path` (string)

    Trained SVM model.

  * `~swindow_x` (int)
  * `~swindow_y` (int)

    Kernel size of sliding window.

  * `~dataset_path` (string)

    Path to directory which contains dataset rosbag files.

    It should end with `/`.

  * `~object_dataset_filename` (string)
  * `~nonobject_dataset_filename` (string)

    Rosbag file name of the object/background training set.

* `~image_downsize` (Int, default: `2`)

  Reduces the image by this factor. (Smaller image dimensions makes processing faster)

  This parameter can be changed by `dynamic_reconfigure`.

* `~scaling_factor` (float, default: `-0.06`)

  Scale factor for pyramidical scaling of the window.
  `+` value indices increase while `-` reduce.

  This parameter can be changed by `dynamic_reconfigure`.

* `~stack_size` (int, default: `2`)

  Spefices the number of times a window is to be changed for raster scanning.

  The changed factor for each traversal is determined by `~scaling_factor`

  This parameter can be changed by `dynamic_reconfigure`.

* `~sliding_window_increment` (int, default: `16`)

  Spefices the number of pixels to shift the window for next detection
  (a.k.a. stride).

  This parameter can be changed by `dynamic_reconfigure`.

![](images/sliding_window_object_detector_rqt.png)

Note that these parameters are critical in determining the detection rate and the speed of execution.
Please fine tune the parameters accordingly to get the best performance.


## Sample

```bash
roslaunch jsk_perception sample_sliding_window_object_detector.launch
```
