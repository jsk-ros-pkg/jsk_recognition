GridSampler
===========


.. image:: images/grid_sampler.png


What is this?
-------------

Sample clusters of point cloud with grid size.


Subscribing Topics
------------------

- ``~input`` (``sensor_msgs/PointCloud2``)

  Input point cloud.


Publishing Topics
-----------------

- ``~output`` (``jsk_recognition_msgs/ClusterPointindices``)

  Clusters of point indices which reprent each grid sample.


Parameters
----------

- ``~grid_size`` (``Double``, default: ``0.2``)

- ``~min_indices`` (``Int``, default: ``0``)
