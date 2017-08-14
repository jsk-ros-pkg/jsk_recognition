weight_candidates_refiner.py
============================


What is this?
-------------

Refine object candidates with weight scale inputs.

Return label array of refined object candidates.

Subscribing Topic
-----------------

* ``~input/candidates`` (``jsk_recognitiom_msgs/LabelArray``)

  Input candidates

Publishing Topic
----------------

* ``output/candidates/picked`` (``jsk_recognitiom_msgs/LabelArray``)

  Output candidates of picked object

* ``output/candidates/placed`` (``jsk_recognitiom_msgs/LabelArray``)

  Output candidates of placed object

* ``debug/weight_sum`` (``jsk_recognitiom_msgs/WeightStamped``)

  Output weight sum calculated from scale input

* ``debug/weight_sum_at_reset`` (``jsk_recognitiom_msgs/WeightStamped``)

  Output weight sum at reset calculated from scale input

Advertising Service
-------------------

* ``~reset`` (``std_srvs/Trigger``)

  Reset weight sum.


Parameters
----------

* ``~input_topics`` (List of String)

  List of scale inputs

* ``~object_weights`` (List of {String: Float})

  List of object weights

* ``~error`` (Float, ``1.0``)

  Permissible weight error

* ``~approximate_sync`` (Bool, Default: ``False``)

  use approximate_sync policy

* ``~queue_size`` (Int, Default: ``10``)

  Queue size of message_filters

* ``~slop`` (Float, Default: ``0.1``)

  Slop size of message_filters


Sample
------
::

    roslaunch jsk_perception sample_weight_candidates_refiner.launch
