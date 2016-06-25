vgg16_object_recognition.py
===========================


What is this?
-------------

.. image:: ./images/vgg16_object_recognition.png

Recognize object with VGG16 net by resizing input image to 224 x 224.
This node requires pretrained Chainer_ model.
For training VGG16 net, please refer to `wkentaro/vgg16 <https://github.com/wkentaro/vgg16>`_

.. _Chainer:: https://github.com/pfnet/chainer


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input image.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/ClassificationResult``)

  Classification result of input image.

* ``~debug/net_input`` (``sensor_msgs/Image``)

  Resized image to 224 x 224.


Parameters
----------

* ``~gpu`` (Int, Default: ``-1``)

  GPU id. ``-1`` represents CPU mode.

* ``~target_names`` (List of String, Required)

  Target names for classification.

* ``~model_name`` (String, Required)

  Currently ``vgg16`` or ``vgg16_batch_normalization`` is only supported.
  See models in ``$(rospack find jsk_recognition_utils)/python/jsk_recognition_utils/chainermodels``.

* ``~model_h5`` (String, Required)

  Saved h5 file for trained model.

* ``use_mask`` (Bool, Default: ``False``)

  If true, topic ``~input/mask`` is enabled.

* ``~approximate_sync`` (Bool, Default: ``False``)

  Use approximate synchronization policy.

* ``~queue_size`` (Int, Default: ``10``)

  Queue size for synchronization.

* ``~slop`` (Float, Default: ``0.1``)

  Slop for approximate sync.


Example
-------

.. code-block:: bash

   roslaunch jsk_perception sample_vgg16_object_recognition.launch
