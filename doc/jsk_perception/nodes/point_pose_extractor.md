# PointPoseExtractor

![](images/point_pose_extractor.png)

  Calcute object pose compared to template. 

## Subscribing Topic

- `/ImageFeature0D` (`posedetection_msgs::ImageFeature0D`)

  Image, camera and tempalte feature information. 

## Publishing Topics

- `/ObjectDetection` (`posedetection_msgs::ObjectDetection`)

  Detected object pose with time stamp. 

- `/ObjectDetection_agg` (`posedetection_msgs::ObjectDetection`)

  Detected object pose with time stamp. 

- `/object_pose` (`geometry_msgs::PoseStamped`)

  Detected Object Pose. 

- `/debug_image` (`cv_bridge::CvImage`) 

  Output image for debug.

## Parameters

- `~template_filename` (str default: "/sample/opencv-logo2.png")
- `~reprojection_threshold` (float default: 3.0)
- `~distanceratio_threshold` (float default: 0.49)
- `~error_threshold` (float default: 50.0)
- `~theta_step` (float default: 5.0)
- `~phi_step` (float default: 5.0)
- `~viewer_window` (bool default: true)
- `~window_name` (str default: "sample1")
- `~autosize` (bool default: false)
- `~publish_null_object_detection` (bool default: false)

## Service 

- `/SetTemplate` 

  Used to add another template.

## Sample

`sample_point_pose_extractor.launch`
```
<launch>
  <node name="nodelet_manager"
        pkg="nodelet" type="nodelet"
        args="manager" />
  <node name="imagesift"
        pkg="nodelet" type="nodelet"
        args="load imagesift/ImageSift nodelet_manager">
    <remap from="image" to="<rgb camera image topic name>"/>
    <remap from="camera_info" to="<rgb camera info topic name>" />
  </node>
  <node name="point_pose_extractor" pkg="jsk_perception" type="point_pose_extractor"
        respawn="true" output="screen"/>
</launch>
``` 

```
roslaunch sample_point_pose_extractor.launch  
rostopic echo /ObjectDetection
```

### Example of how to run set_template service 

```
#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2

import cv_bridge
import rospy
from jsk_perception.srv import SetTemplate, SetTemplateRequest

rospy.init_node('point_pose_extractor_sample')

client = rospy.ServiceProxy('SetTemplate', SetTemplate)
rospy.sleep(1)

req= SetTemplateRequest()

im = cv2.imread('../../sample/ros_diamondback.jpg')
bridge = cv_bridge.CvBridge()
imgmsg = bridge.cv2_to_imgmsg(im, encoding='bgr8')
imgmsg.header.frame_id = 'dummy_camera'
imgmsg.header.stamp = rospy.Time.now()

req.type = 'img0001'
req.image = imgmsg
req.dimx = 0.1
req.dimy = 0.05
req.savefilename = 'img0001.png'
res = client.call(req)
print(res)
```