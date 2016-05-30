PointCloudToPCD
==========================

What is this?
-------------

Subscribe PointCloud2 topic and save in pcd file.

Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/PoinCloud2``)


Parameters
----------

* ``prefix`` (String, default: Empty)
  
  prefix of PCD filenames.
  pcd files are saved as ``{prefix}{stamp}.pcd``.

* ``binary`` (Boolean, default ``False``)

  save binary pcd files.

* ``compressed`` (Boolean, default ``False``)

  save as compressed pcd files.
  this parameter is only effective when ``binary`` is ``True``.

* ``fixed_frame`` (String, default: Empty)

  transform point cloud to fixed frame.
  when it is empty, point cloud is not transformed.

* ``duration`` (Double, default: ``1.0``)

  Saving duration. You can change saving frequency with this parameter.
