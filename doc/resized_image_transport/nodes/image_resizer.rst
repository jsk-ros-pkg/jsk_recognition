image_resizer
=============

Publish resized image and camera_info.


Subscribing Topic
-----------------

- ``~input/image`` (``sensor_msgs/Image``)

  Input image.

- ``~input/camera_info`` (``sensor_msgs/CameraInfo``)

  Input camera info.


.. note::
   The subscribing topic name is changed when you remap the one of input image,
   because it uses `image_transport <http://wiki.ros.org/image_transport>`_.


Publishing Topic
----------------

- ``~output/image`` (``sensor_msgs/Image``)

  Resized image.

- ``~output/camera_info`` (``sensor_msgs/CameraInfo``)

  Resized camera info.


Parameters
----------

- ``~resize_scale_x``, ``~resize_scale_y`` (``Double``, default: ``0.25``)

  Resizing scale.

- ``~use_messages`` (``Bool``, default: ``true``)

  If ``true``, topic publishing rate will be limited, and it causes some problems
  on handling rostime: for example ``rosbag play --loop`` won't work with this option,
  and the topic publication is stopped.

- ``~msg_par_second`` (``Double``, default: ``15.0``)

  Topic publishing rate if ``~use_messages`` is ``true``.

- ``~interpolation`` (``String``, default: ``LINEAR``)

  Candidates: NEAREST, LINEAR, AREA, CUBIC, LANCZOS4

  See `this page <https://docs.opencv.org/2.4/modules/imgproc/doc/geometric_transformations.html#resize>`_ for more details

Sample
------

::

    $ roslaunch resized_image_transport sample_image_resizer.launch
