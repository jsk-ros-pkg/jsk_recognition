fcn_object_segmentation.py
==========================


What is this?
-------------

.. image:: ./images/fcn_object_segmentation.png

Segment object in pixel wise with Fully Convolutional Networks.


Subscribing Topic
-----------------

**Default**

* ``~input`` (``sensor_msgs/Image``)

  Raw image.

**Optional**

* ``~input/mask`` (``sensor_msgs/Image``)

  Mask whose black region must be the background label: ``0``.
  This topic is subscribed only when param ``~use_mask`` is ``true``.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Label image each object in param ``~target_names`` is segmented.


Parameters
----------

**Default**

* ``~gpu`` (Int, Default: ``-1``)

  GPU id. ``-1`` represents CPU mode.

* ``~target_names`` (List of String, Required)

  Target names for classification.

* ``~model_name`` (String, Required)

  Currently ``fcn8s``, ``fcn16s`` or ``fcn32s`` is only supported.
  See models in https://github.com/wkentaro/fcn/tree/master/fcn/models.

* ``~model_h5`` (String, Required)

  Saved h5 file for trained model.


**Optional**

* ``~queue_size`` (Int, default: ``10``)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.
  This is used only when param ``~use_mask`` is ``true``.

* ``~approximate_sync`` (Bool, default: ``False``)

  Whether to use approximate for input topics.
  This is used only when param ``~use_mask`` is ``true``.

* ``~slop`` (Float, default: ``0.1``)

  How many seconds you allow about the difference of timestamp.
  This is used only when param ``~use_mask`` and ``~approximate_sync`` are ``true``.


Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_fcn_object_segmentation.launch

.. image:: images/sample_fcn_object_segmentation.png
