PCDCapture
==========================

What is this?
-------------

Capture point cloud topic and save in pcd file.

Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/PoinCloud2``)

  PointCloud.

Parameters
----------

* ``filename`` (String, default: ``tmp.pcd``)
  
  PCD filename.

* ``duration`` (Int, default: ``1``)

  Saving duration. You can chage saving frequency with this parameter.
