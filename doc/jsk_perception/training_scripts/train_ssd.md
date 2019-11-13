# Train SSD

This page shows how to train SSD with your own dataset.

SSD is a neural network model used for object detection.

## Available Dataset Class

`ObjectDetectionDataset` (imported from `jsk_recognition_utils.datasets`)

This class assumes the following directory structure for each split.

```
path_to_awesome_dataset/
|-- JPEGImages
|   |-- foo.jpg
|   |-- bar.jpg
|   `-- etc.
|-- SegmentationClass
|   |-- foo.npy
|   |-- bar.npy
|   `-- etc.
|-- SegmentationObject
|   |-- foo.npy
|   |-- bar.npy
|   `-- etc.
|-- class_names.txt
`-- etc.
```

## Arguments

- `--train_dataset_dir` (`string`, default: `$(rospack find jsk_perception)/learning_datasets/kitchen_dataset/train`)
- `--val_dataset_dir` (`string`, default: `$(rospack find jsk_perception)/learning_datasets/kitchen_dataset/test`)

  Directory name which contains dataset for training and validation respectively.

- `--model_name` (`string`, default: `ssd512`)

  Model name. Currently, `ssd300` and `ssd512` are supported.

- `--gpu` (`int`, default: `0`)

  GPU id. `-1` means CPU mode, but we recommend to use GPU for much faster computing.

- `--batch-size` (`int`, default: `8`)

  Number of images used simultaneously in each iteration.

  You should decrease this number when you face memory allocation error.

- `--max-epoch` (`int`, default: `100`)

  Stop trigger for training.

- `--out_dir` (`string`, default: `${ROS_HOME}/learning_logs/<timestamp>`)

  Output directory name.

## Output

All these files will be automatically generated under `<out_dir>`.

- `log.json`
- `model_snapshot.npz`

## Usage

```
rosrun jsk_perception train_ssd.py [ARGS]
```

## Sample Output

There are some pre-trained mask rcnn model on jsk_perception.\
Getting trained data by build `jsk_perception` or run script [install_trained_data](https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_perception/scripts/install_trained_data.py)

`73b2 kitchen model` is some of the typical example of pre-trained mask rcnn model on jsk_perception.\
The results of mask rcnn using `73b2 kitchen model` are as follows.

![](./result_images/ssd_73b2_kitchen_sample_result.png)
