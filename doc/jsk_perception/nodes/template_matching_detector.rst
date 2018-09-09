TemplateMatchDetector
=====================

What is this?
-------------

.. image:: ./images/template_matching_detector.jpg

Detect object by template matching


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.

* ``~input/info`` (``sensor_msgs/CameraInfo``)

  Camera Info.


Publishing Topic
----------------

* ``~output/rect`` (``jsk_recognition_msgs/RectArray``)

  Rectangles of detected objects

* ``~output/viz`` (``sensor_msgs/Image``)

  Visualized image of detection result.


Parameters
----------

* ``~queue_size`` (Int, default: ``100``)

  Size of queue

* ``~template_filename`` (String, default: ``template.png``)

  Template image's file name

* ``~min_scale`` (Float, default: ``0.4``)

  Min scale of template

* ``~max_scale`` (Float, default: ``1.6``)

  Max scale of template

* ``~resize_template_num`` (Int, default: ``20``)

  the number of resizing template

* ``~matching_threshold``, (Float, default: ``0.7``)

  Threshold of matching result

* ``~nms_threshold``, (Int, default: ``0.5``)

  Threshold of non maximum suppression for overlapped rects

* ``~target_num``, (Int, default: ``6``)

  The number of target

* ``~sort_direction``, (Int, default: ``0``)

  Option of sorting target. 0: NoOperation, 1: sort by Vertical 2: sort by horizontal

* ``~check_flipped_image``, (Bool, default: ``True``)

  If ``True``, template matching using original template image and use flipped template image

* ``~update_matching_threshold``, (Bool, default: ``False``)

  If ``True``, update matching threshold

* ``~update_step_of_threshold``, (Float, default: ``0.05``)

  Update step of matching threshold


Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_template_matching.launch
