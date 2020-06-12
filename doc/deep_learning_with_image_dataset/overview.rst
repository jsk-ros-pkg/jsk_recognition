Deep Learning with Your Own Image Dataset
=========================================

This page shows the overview of how to start deep learning with your own image dataset.


1. Collect raw images
---------------------

Collect available images from Web or take pictures by yourself.


2. Annotate images
------------------

Label each image or each region in the images.

.. toctree::
   :maxdepth: 1

   annotate_images_with_labelme


3. Train neural network
-----------------------

Choose one network model according to your task.

There are some training scripts in jsk_perception package.

   .. toctree::
      :maxdepth: 1
      :caption: Training for semantic segmentation

      ../jsk_perception/training_scripts/train_fcn

   .. toctree::
      :maxdepth: 1
      :caption: Training for instance segmentation

      ../jsk_perception/training_scripts/train_mask_rcnn


   .. toctree::
      :maxdepth: 1
      :caption: Training for object detection

      ../jsk_perception/training_scripts/train_ssd

4. Infer with trained network
-----------------------------

Now you can use your trained network to infer things as a ROS node.

   .. toctree::
      :maxdepth: 1
      :caption: Nodes for semantic segmentation

      ../jsk_perception/nodes/fcn_object_segmentation

   .. toctree::
      :maxdepth: 1
      :caption: Nodes for instance segmentation

      ../jsk_perception/nodes/mask_rcnn_instance_segmentation

   .. toctree::
      :maxdepth: 1
      :caption: Nodes for object detection

      ../jsk_perception/nodes/ssd_object_detector
