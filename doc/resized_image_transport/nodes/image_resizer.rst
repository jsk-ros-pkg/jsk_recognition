image_resizer
=============

Publish resized image and camera_info.


Subscribing Topic
-----------------

- ``~input/image`` (``sensor_msgs/Image``)

   Input image.

-  ``~input/camera_info`` (``sensor_msgs/CameraInfo``)

   Input camera info.


.. note::
   The subscribing topic name is changed when you remap the one of input image,
   because it uses `image_transport <http://wiki.ros.org/image_transport>`_.


Publishing Topic
----------------

-  ``~output/image`` (``sensor_msgs/Image``)

   Resized image.

-  ``~output/camera_info`` (``sensor_msgs/CameraInfo``)

   Resized camera info.


Sample
------

::

    $ roslaunch resized_image_transport image_resizer.test
    $ rosrun image_view image_view image:=/image_resizer/output/image
