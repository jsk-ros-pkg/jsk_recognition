image_to_label.py
=================

What is this?
-------------

Publish an label image converted from raw image.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``, ``encoding: 32SC1``)

  Label image.
