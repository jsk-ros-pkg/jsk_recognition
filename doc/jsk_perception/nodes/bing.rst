Bing
====

What is this?
-------------

.. image:: ./images/bing.png

Compute objectness in a image frame, and publishes object proposals.

.. note:: This node is supported only when OpenCV3 is installed.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/RectArray``)

  Object proposals.

* ``~output/objectness`` (``sensor_msgs/Image``, encoding: ``32FC1``)

  Float image which describes objectness.

Parameters
----------

* ``~score_threshold`` (Double, default: ``0.0``)

  Score threshold for objecness values. Max value is ``1.0``.

* ``~max_num`` (Int, default: ``-1.0``)

  Maximum number of returning bounding boxes. ``-1.0`` means returning all bounding boxes.


Sample
------

.. code-block:: bash

  roslaunch jsk_perception bing.test


Reference
---------

::

  @inproceedings{BingObj2014,
    title={{BING}: Binarized Normed Gradients for Objectness Estimation at 300fps},
    author={Ming-Ming Cheng and Ziming Zhang and Wen-Yan Lin and Philip H. S. Torr},
    booktitle={IEEE CVPR},
    year={2014},
  }
