unwrap_histogram_with_range_array.py
====================================

.. image:: ./images/unwrap_histogram_with_range_array.png

What is this?
-------------

Unwrap a HistogramWithRangeArray message and publish its first element.


Subscribing Topic
-----------------

* ``~input`` (``jsk_recognition_msgs/HistogramWithRangeArray``)

  Histogram array.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/HistogramWithRange``)

  Histogram


Sample
------

.. code-block:: bash

  roslaunch jsk_perception sample_unwrap_histogram_with_range_array.launch
