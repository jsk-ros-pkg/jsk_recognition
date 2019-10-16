fast_rcnn.py
============

What is this?
-------------

.. image:: ./images/fast_rcnn.gif

Publish an image with object bounding boxes, scores and labels.

::

  CLASSES = ('__background__',
             'aeroplane', 'bicycle', 'bird', 'boat',
             'bottle', 'bus', 'car', 'cat', 'chair',
             'cow', 'diningtable', 'dog', 'horse',
             'motorbike', 'person', 'pottedplant',
             'sheep', 'sofa', 'train', 'tvmonitor')


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.

* ``~input/rect_array`` (``jsk_recognition_msgs/RectArray``)

  Object location proposals.



Publishing Topic
----------------

* ``~output/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Detected object class labels and probabilities.

* ``~output/rect_array`` (``jsk_recognition_msgs/RectArray``)

  Rects of detected objects.


Parameters
----------

* ``~model`` (String, **required**)

  Network model name. (``vgg_cnn_m_1024`` or ``vgg16``)
  ``vgg_cnn_m_1024`` is small network and requires ~2GB GPU memory.
  ``vgg16`` is large network and requires ~5GB GPU memory.

* ``~gpu`` (Int, default: ``-1``)

  GPU ID.
  Negative value means CPU mode.

* ``~classifier_name`` (String, default: ``rospy.get_name()``)

  Classifier name written to ``classifier`` field of ``~output/class``.

* ``~approximate_sync`` (Bool, default: ``False``)

  Whether to use approximate for input topics.

* ``~queue_size`` (Int, default: ``10``)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.

* ``~slop`` (Float, default: ``0.1``)

  How many seconds you allow about the difference of timestamp
  when you specify ``~approximate_sync``.


Example
-------

.. image:: images/fast_rcnn_example.jpg
   :width: 50%

.. code-block:: bash

  roslaunch jsk_perception sample_fast_rcnn.launch
