add_bounding_box_array.py
=========================

What is this?
-------------

Add multiple topics of ``jsk_recognition_msgs/BoundingBoxArray``
and republish them as a topic.


Subscribing Topic
-----------------

See `~topics` in **Parameters**.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/BoundingBoxArray``)

  Added bounding box array.


Parameters
----------

**Required**

* ``~topics`` (Array of String, required)

  Topics to be added.

**Optional**

* ``~approximate_sync`` (Bool, default: ``False``)

  Whether to use approximate for input topics.

* ``~queue_size`` (Int, default: ``10``)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.

* ``~slop`` (Float, default: ``0.1``)

  How many seconds you allow about the difference of timestamp
  when you specify ``~approximate_sync``.


Usage
-----

.. code-block:: bash

  roslaunch jsk_recognition_utils sample_add_bounding_box_array.launch
