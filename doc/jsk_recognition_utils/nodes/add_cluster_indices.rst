add_cluster_indices.py
======================

What is this?
-------------

Add multiple topics of ``jsk_recognition_msgs/ClusterPointIndices``
and republish them as a topic.


Subscribing Topic
-----------------

See `~topics` in **Parameters**.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/ClusterPointIndices``)

  Added cluster of point indices.


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

  rosrun jsk_recognition_utils add_cluster_indices.py _topics:='[/node_a/cluster_indices, /node_b/cluster_indices]'
