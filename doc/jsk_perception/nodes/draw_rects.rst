draw_rects
==================

What is this?
-------------

.. image:: https://user-images.githubusercontent.com/1901008/31119884-7063d85e-a86d-11e7-9d33-8b8290de4168.gif
.. image:: https://user-images.githubusercontent.com/1901008/31119889-758b560e-a86d-11e7-9f84-3b9dd8db17bd.png

Draw rectangles (and their classes if available) on image

Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/Image``)

  Raw image.

* ``~input/rects`` (``jsk_recognition_msgs/RectArray``)

  Rectangles on an input image.

* ``~input/class`` (``jsk_recognition_msgs/ClassificationResult``)

  Class labels for each rectangles.


Publishing Topic
----------------

* ``~output`` (``sensor_msgs/Image``)

  Image on which rectangles and classes are drawn.

Parameters
----------

* ``~approximate_sync`` (Bool, default: ``False``)

  Whether to use approximate for input topics.

* ``~queue_size`` (Int, default: ``100``)

  How many messages you allow about the subscriber to keep in the queue.
  This should be big when there is much difference about delay between two topics.

* ``~use_classification_result`` (Bool, default: ``False``)

  Use ``~input/class`` as class labels if enabled.

* ``~show_proba`` (Bool, default: ``True``)

  Show probability for each class labels if enabled.
  This option is valid when ``~use_classification_result`` is ``True``.

* ``~rect_boldness`` (Int, default: ``2``)

  Boldness for each rectangles.

* ``~label_size`` (Double, default: ``1.0``)

  Text size for each class labels.

* ``~label_boldness`` (Double, default: ``2.0``)

  Boldness for each characters of class label texts.

* ``~label_font`` (Enum[Int], default: ``FONT_HERSHEY_SIMPLEX``)

  Font for class labels.

* ``~label_margin_factor`` (Dobule, default: ``1.1``)

  Margin factor for class label background rectangle

* ``~resolution_factor`` (Double, default: ``2.0``)

  Factor for resolution of output image.
  When this option is set as ``1.0``, an output image has the same resolution as an input image.

* ``~interpolation_method`` (Enum[Int], default: ``INTER_LANCZOS4``)

  Method for interpolation on input image resizing.

Example
-------

.. image:: https://user-images.githubusercontent.com/1901008/31119893-7cbcb9c2-a86d-11e7-9f04-ab01462acd65.jpg
   :width: 50%

.. code-block:: bash

   roslaunch jsk_perception sample_draw_rects.launch
