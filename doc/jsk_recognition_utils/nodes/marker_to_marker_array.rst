marker_to_marker_array.py
=========================

What is this?
-------------

Create (1,) shaped ``visualization_msgs/MarkerArray`` from ``visualization_msgs/Marker``.


Subscribing Topic
-----------------

* ``~input`` (``visualization_msgs/Marker``)

  Input marker.


Publishing Topic
----------------

* ``~output`` (``visualization_msgs/MarkerArray``)

  Marker array message with the input marker as the first element.
