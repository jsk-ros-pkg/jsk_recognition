bof_histogram_extractor.py
==========================


What is this?
-------------

.. image:: ../images/bof_object_recognition.png

Extract Bag-of-Features histogram.

For other information about object recognition using Bag-of-Features, see `here <../bof_object_recognition.html>`_.


Subscribing Topic
-----------------

* ``~input`` (``posedetection_msgs/Feature0D``)

  Feature data.

* ``~input/label`` (``sensor_msgs/Image``)

  Label image with witch features will be extracted.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/VectorArray``)

  Extracted feature data.


Parameters
----------

* ``~bof_data`` (String, required)

  Path to Bag-of-Features data.

  The file contains ``jsk_recognition_utils.feature.BagOfFeatures`` object, and it should be pickled and gzipped.

* ``~queue_size`` (Int, default: ``10``)

  Maximum number of messages which subscriber keeps in the queue.

* ``~approximate_sync`` (Bool, default: ``False``)

  Whether to use ApproximateSync for input topics.

* ``~slop`` (Float, default: ``0.1``)

  Number of seconds you allow about the difference of timestamp.

  This parameter is enabled only when ``~approximate_sync`` is ``True``.


Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_bof_object_recognition.launch
