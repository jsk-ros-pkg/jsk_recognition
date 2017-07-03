label_image_classifier.py
=========================


What is this?
-------------

.. image:: ./images/label_image_classifier.png

Classify from label image.

Return largest segmented label.

Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Input label image.


Publishing Topic
----------------

* ``~output`` (``jsk_recognition_msgs/ClassificationResult``)

  Classification result of input image.

Parameters
----------

* ``~ignore_labels`` (List of Int, Default: ``[]``)

  Ignoring labels.


Example
-------

.. code-block:: bash

   roslaunch jsk_perception sample_label_image_classifier.launch
