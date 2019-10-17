bounding\_box\_array\_publisher.py
==================================

What is this?
-------------

Add multiple topics of `jsk_recognition_msgs/ClusterPointIndices` and
republish them as a topic.

Publishing Topic
----------------

- `~output` (`jsk_recognition_msgs/BoundingBoxArray`)

  Bounding boxes in the specified frame\_id.

Parameters
----------

**Required**

- `~frame_id` (String, required)

  Frame id of bounding boxes.

- `~boxes` (Yaml, required)

  Pose and dimension of bounding boxes. It is something like below:

```xml
<rosparam>
  boxes:
    - position: [-0.22, 0.280, 0.361] # required
      rotation: [0, 0, 1.57] # optional
      dimension: [0.37, 0.248, 0.218] # required
      label: 0 # optional
    - position: [-0.22, 0, 0.361]
      dimension: [0.37, 0.306, 0.218]
</rosparam>
```

**Optional**

* `~rate` (Int, default: `1`)

  How many messages are published in a second.

Sample
------

```bash
roslaunch jsk_recognition_utils sample_bounding_box_array_publisher.launch
```
