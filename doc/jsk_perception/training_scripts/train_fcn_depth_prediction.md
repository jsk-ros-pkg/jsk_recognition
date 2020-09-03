# Train FCN Depth Prediction

This page shows how to train FCN Depth Prediction with your own dataset.

FCN Depth Prediction is a neural network model used for depth prediction with semantic segmentation.

Any size of image can be applied to this network as long as your GPU has enough memory.


## Available Dataset Class

`DepthPredictionDataset` (imported from `jsk_recognition_utils.datasets`)

This class assumes the following directory structure.

```
path_to_awesome_dataset
|-- foo1
|   |-- train
|   |   |-- bar1
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   |-- bar2
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   `-- etc.
|   |-- test
|   |   |-- bar3
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   |-- bar4
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   `-- etc.
|   `-- etc.
|-- foo2
|   |-- train
|   |   |-- bar5
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   |-- bar6
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   `-- etc.
|   |-- test
|   |   |-- bar7
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   |-- bar8
|   |   |   |-- depth.npz
|   |   |   |-- depth_gt.npz
|   |   |   |-- image.png
|   |   |   |-- label.png
|   |   |   `-- etc.
|   |   `-- etc.
|   `-- etc.
`-- etc.
```

## Arguments

- `--dataset_dir` (`string`, default: `$(rospack find jsk_perception)/learning_datasets/human_size_mirror_dataset`)

  Directory name which contains dataset for training and validation.

- `--model` (`string`, default: `FCN8sDepthPredictionConcatFirst`)

  Model name. Currently, `FCN8sDepthPredictionConcatFirst` is supported.

- `--gpu` (`int`, default: `0`)

  GPU id. `-1` means CPU mode, but we recommend to use GPU for much faster computing.

- `--batch_size` (`int`, default: `1`)

  Number of images used simultaneously in each iteration.

  You should decrease this number when you face memory allocation error.

- `--epoch` (`int`, default: `100`)

  Stop trigger for training.

- `--out` (`string`, default: `${ROS_HOME}/learning_logs/<timestamp>`)

  Output directory name.


Output
------

All these files will be automatically generated under `<out_dir>`.

- `batch_size.txt`
- `dataset.txt`
- `log.json`
- `loss_plot.png`
- `model.txt`
- `model_snapshot.npz`
- `network_architecture.dot`


Usage
-----

```
rosrun jsk_perception train_fcn_depth_prediction.py [ARGS]
```
