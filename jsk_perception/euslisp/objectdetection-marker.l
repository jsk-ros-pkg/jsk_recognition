#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_perception")

(defun objectdetection-cb (obj-msg)
  (let (frame-id type pose
        (msg-array (instance visualization_msgs::MarkerArray :init))
        (header (send obj-msg :header))
        (id 0) msgs)
    (setq frame-id (send obj-msg :header :frame_id))
    (dolist (obj-pose (send obj-msg :objects))
      (setq type (send obj-pose :type))
      (setq pose (ros::tf-pose->coords (send obj-pose :pose)))
      (format t "frame_id=~A, type=~A, pose=~A~%" frame-id type pose)
      ;;
      (push (coords->marker-msg pose header :id id :size 2 :width 20 :lifetime 1) msgs)
      (push (text->marker-msg type pose header :ns "text" :id id :scale 150 :lifetime 2) msgs)
      (incf id))
    (send msg-array :markers msgs)
    (ros::publish "objectdetection_marker_array" msg-array) 
    ))

(ros::roseus "objectdetection_marekr")
(ros::subscribe "ObjectDetection" posedetection_msgs::ObjectDetection #'objectdetection-cb)
(ros::advertise "objectdetection_marker_array" visualization_msgs::MarkerArray 5)
(ros::ros-info "start objectdetection_marker")
(ros::ros-info "Wait for /ObjectDetection")
(ros::ros-info "Publish for /objectdetection_marker_array")

(ros::rate 10)
(do-until-key
 (ros::spin-once)
 (ros::sleep))
