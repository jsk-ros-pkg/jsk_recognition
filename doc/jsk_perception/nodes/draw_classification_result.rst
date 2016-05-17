draw_classification_result.py
=============================


What is this?
-------------

.. image:: ./images/draw_classification_result.jpg
   :width: 50%

Publish an image topic on which ``jsk_recognition_msgs/ClassificationResult`` is visualized.


Subscribing Topic
-----------------

* ``~input`` (``jsk_recognition_msgs/ClassificationResult``)

  Classification result.

* ``~input/image`` (``sensor_msgs/Image``)

  Image on which the result is visualized.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Image on which classification result is drew.


Example
-------

.. code-block:: bash

   roslaunch jsk_perception sample_draw_classification_result.launch
