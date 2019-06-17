checkerboard_detector
====================
![](images/checkerboard_detector.png)

`checkerboard_detector` is an executabel to detect checker board.
You can find marker patter in [jsk-ros-pkg/calibboard_sheet](https://github.com/jsk-ros-pkg/calibboard_sheet).

Subscribing Topics
------------------
* `~image` (`sensor_msgs/Image`)
* `~camera_info` (`sensor_msgs/CameraInfo`)

  Input image and camera info. Intrinsic camera parameter is
  acquired from `~camera_info`.

Publishing Topics
-----------------
* `ObjectDetection` (`posedetection_msgs/ObjectDetection`)
* `objectdetection_pose` (`geometry_msgs/PoseStamped`)

  Pose of checkerboard in `posedetection_msgs/ObjectDetection` and `geometry_msgs/PoseStamped`.
* `corner_point` (`geometry_msgs/PointStamped`)

  Corner points.
* `polygons` (`jsk_recognition_msgs/PolygonArray`)

  Publish checker board as `jsk_recognition_msgs/PolygonArray`. It is useful to visualize in rviz.

Parameters
----------
* `display` (default: `0`)

  Set `1` to enable debug view.
* `board_type` (default: `chess`)

  Type of marker. `chess`, `circle` and `acircle` are supported.

* `rect%d_size_x`
* `rect%d_size_y`

  Size of checkerboard in m unit.
* `grid%d_size_x`
* `grid%d_size_y`

  The number of grids in x and y axis.
* `use_P` (default: `false`)

  By default, use camera matrix (K) and unrectified image (image_raw).
  If you use rectified image (image_rect), use_P should be true.
* `invert_color` (default: `false`)

  Invert white and black before searching cross points or circles.
* `message_throttle`   (default: `1`)

  Finding checker boards every `message_throttle` images
* `queue_size`         (default: `1`)
* `publish_queue_size` (default: `1`)

  Size of queue of subscriber is `queue_size`, publisher is `publish_queue_size`.
* `axis_size`   (default: `0.05`)
* `circle_size` (default: `6`)

  For setting displayed marker size.
  Set circle_size as [pixel].
  Set axis_size as [m].


Trouble Shooting
----------------
* Q. Estimated checker board pose is not correct

  A. First check debug image and all the detected corner points correctly superimposed on camera view.
  * If the detected corner points is **not correct**, you need to modify checker board grid size (`grid_size` parameters).
  * If the detected corner points is **correct**, confirm checker board size (`rect_size` parmaeters) and intrinsic camera paramter is calibrated well.

* Q. How many number of grids better?

  A. I strongly recomment to choose `odd`x`even` or `even`x`odd`. Because if you choose `odd`x`odd` or `even`x`even`,
  detector will have two potential poses.
