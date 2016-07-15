BoundingObjectMaskImage
=======================

What is this?
-------------

.. image:: ./images/bounding_object_mask_image.png

Publish mask that is bounding object mask of the input mask.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input mask image.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Bounding object mask image.


Parameters
----------

None.


Sample
------
::

    roslaunch jsk_perception sample_bounding_object_mask_image.launch
