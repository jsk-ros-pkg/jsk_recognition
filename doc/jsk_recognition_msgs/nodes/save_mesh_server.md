# save_mesh_server.py

## What is this?

Subscribe `jsk_recognition_msgs/BoundingBox` and send request to mesh saving server.
This node is a service server called from users, and at the same time a service client.

See also: [jsk_pcl/Kinfu](../../jsk_pcl_ros/nodes/kinfu.md)


## Subscribing Topic

* `~input/bbox` (`jsk_recognition_msgs/BoundingBox`)

  Bounding box used for request.


## Publishing Topic

None.


## Advertising Service

* `~request` (`std_srvs/Empty`)

  Trigger to send request by users.


## Calling Service

* `~save_mesh` (`jsk_recognition_msgs/SaveMesh`)

  Service call from this node as a client.


## Parameters

* `~ground_frame_id` (String, default: `""`)

  Frame ID used for calling `~save_mesh`.
