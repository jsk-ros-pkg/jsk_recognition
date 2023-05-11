PointCloudToPCD
==========================

What is this?
-------------

Subscribe PointCloud2 topic and save in pcd file.

Subscribing Topic
-----------------

* ``~input`` (``sensor_msgs/PoinCloud2``)


Advertising Services
-----------------
* ``~save_pcd`` (``std_srvs/Empty``)

  Convert PointCloud2 to pcd, saved under `{prefix}{stamp}.pcd`.

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
  If the duration is greater than 0.0, the pcd data is stored continuously under the set duration.
  When you want to use this as ROS service, set the duration to 0.0.
