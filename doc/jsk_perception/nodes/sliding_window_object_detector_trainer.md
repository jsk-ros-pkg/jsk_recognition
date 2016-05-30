# SlidingWindowObjectDetectorTrainer
## What is this?
Nodelet to train `jsk_perception/SlidingWindowObjectDetector` using binary support vector machine. The object is assigned a label of +1 and -1 otherwise. The SVM used is from the OpenCV Library with default set to RBF Kernel and 10-Fold Cross Validations.

Note that this nodelet produces two output files of ".xml" format to the working directory.
* `1) Trained Classifier` - this file the trained object SVM. Dont edit this file.
* `2) sliding_window_trainer_manifest.xml` - this file contains parameters of the trainer that are not in (1). Information such as trainer window size, save directory, etc. 

![](images/trainer_manifest.png)

## Parameters
* `~dataset_path` (string, default: `training_dataset`)

  Folder name where the training sets resides.

* `~object_dataset_filename` (string)

   Rosbag file name of the object (positive) training set 

* `~nonobject_dataset_filename` (string)

   Rosbag file name of the non-object (negative) training set 

* `~classifier_name` (string)

   Name of the trained svm output file

* `~swindow_x` (int, default: `32`)

   Window width

* `~swindow_y` (int, default: `64`)

   Window height  
