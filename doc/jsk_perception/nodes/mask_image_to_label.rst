mask_image_to_label.py
======================

What is this?
-------------

.. image:: ./images/mask_image_to_label.png

Publish an label image converted by mask image.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Mask image.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``, ``encoding: 32SC1``)

  Label image.


Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_mask_image_to_label.launch
