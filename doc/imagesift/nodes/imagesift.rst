ImageSift
=========

.. image:: images/imagesift.png

Extract sift features from input image.


Subscribing Topic
-----------------

- ``/image`` (``sensor_msgs/Image``)

   Input image. This triggers output ``Feature0D``.

-  ``/camera_info`` (``sensor_msgs/CameraInfo``)

   Input camera_info.


Publishing Topic
----------------

-  ``/Feature0D`` (``posedetection_msgs/Feature0D``)

   This appears with input ``image``.

-  ``/ImageFeature0D`` (``posedetection_msgs/ImageFeature0D``)

   This appears with both inputs ``image`` and ``camera_info``.

Parameters
----------

- ``~image_transport`` (``String``, default: ``raw``)

  Set `compressed` or `theora` to subscribe compressed images

Run
---
You can run executable like below::

    $ rosrun imagesift imagesift

To subscribe compressed image, run executable like below::

    $ rosrun imagesift imagesift _image_transport:=compressed


Sample
------

::

    $ roslaunch imagesift imagesift_sample.launch
