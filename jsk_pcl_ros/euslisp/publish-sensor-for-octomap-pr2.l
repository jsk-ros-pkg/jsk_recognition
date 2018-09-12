#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(ros::roseus "publish_sensor_for_octomap_pr2")
(ros::advertise "/camera_cloud" sensor_msgs::PointCloud2 1)
(ros::advertise "/hand_cloud" sensor_msgs::PointCloud2 1)
(ros::advertise "/hand_contact" jsk_recognition_msgs::ContactSensorArray 1)
(ros::rate 10)

(while (ros::ok)
  (let* ((header
          (instance std_msgs::Header :init :frame_id "/octomap_world" :stamp (ros::time-now)))
         camera-point-list
         hand-point-list
         contact-msg-list
         )
    ;; publish point cloud of camera (simulate depth camera) and hand (simulate proximity sensor)
    (dotimes (x 100)
      (dotimes (y 200)
        (push (float-vector (- x 50) (- y 100) (+ 400 (random 50.0))) camera-point-list)
        ))
    (dotimes (x 10)
      (dotimes (y 20)
        (push (float-vector (- x 5) (- y 10) (+ 40 (random 10.0))) hand-point-list)
        (push (float-vector (- x 5) (- y 10) (+ -100 (random 10.0))) hand-point-list)
        ))
    ;; publish contact information
    (dolist (link-name (list "l_gripper_palm_link" "l_gripper_r_finger_link" "l_gripper_l_finger_link"))
      (push
       (instance jsk_recognition_msgs::ContactSensor :init
                 :header header :contact nil :link_name link-name)
       contact-msg-list)
      )
    (ros::publish "/camera_cloud"
                  (make-ros-msg-from-eus-pointcloud
                   (instance pointcloud :init :points (apply #'matrix camera-point-list))
                   :frame "/wide_stereo_optical_frame"))
    (ros::publish "/hand_cloud"
                  (make-ros-msg-from-eus-pointcloud
                   (instance pointcloud :init :points (apply #'matrix hand-point-list))
                   :frame "/r_gripper_tool_frame"))
    (ros::publish "/hand_contact"
                  (instance jsk_recognition_msgs::ContactSensorArray :init
                            :header header :datas contact-msg-list))
    (ros::sleep)
    ))
(ros::roseus "shutdown")
(exit)
