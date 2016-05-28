BoundingRectMaskImage
=====================

What is this?
-------------

.. image:: ./images/bounding_rect_mask_image.png

Publish mask that is bounding rect mask of the input mask.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input mask image.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Bounding rect mask image.


Parameters
----------

None.


Sample
------
::

    roslaunch jsk_perception sample_bounding_rect_mask_image.launch
