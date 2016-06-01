ParallelEdgeFinder
====================


.. image:: images/parallel_edge_finder.png


What is this?
-------------

Find parallel edges.


Subscribing Topics
------------------

- ``~input_indices`` (``jsk_recognition_msgs/ClusterPointIndices``)

- ``~input_coefficients`` (``jsk_recognition_msgs/ModelCoefficientsArray``)

Publishing Topics
-----------------

- ``~output_edges_groups`` (``jsk_recognition_msgs/ParallelEdgeArray``)

- ``~output_clusters`` (``jsk_recognition_msgs/ClusterPointIndices``)

Parameters
----------

- ``~angular_threshold`` (``Double``, default: ``0.1``)

  Allowable angle of inclination of pair of edges.
