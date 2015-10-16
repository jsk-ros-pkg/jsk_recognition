label_image_decomposer.py
=========================

What is this?
-------------

.. image:: ./images/label_image_decomposer.png

Publish an image topic to which tile mask applied and tile image with decomposed with label.


Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.

* ``~input/label`` (``sensor_msgs/Image``)

  Label image to decompose the input raw image with.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Label image mask applied image.

* ``~output/tile`` (``sensor_msgs/Image``)

  Tile image listing decomposed images.
  This is published when ``~publish_tile`` is ``True``. See **Parameters**


Parameters
----------

* ``~approximate_sync`` (Bool, default: ``False``)

  Whether to use approximate for input topics.

* ``~publish_tile`` (Bool, default: ``False``)

  Whether to publish tile image with decomposed images.

* ``~slop`` (Float, default: ``0.1``)

  How many seconds you allowed about the difference of timestamp
  when you specify ``~approximate_sync``.
