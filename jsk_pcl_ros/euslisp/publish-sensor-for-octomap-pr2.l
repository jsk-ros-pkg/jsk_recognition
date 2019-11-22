#!/usr/bin/env roseus

(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")

(ros::roseus "publish_sensor_for_octomap_pr2")
(ros::advertise "/camera_cloud" sensor_msgs::PointCloud2 1)
(ros::advertise "/hand_cloud" sensor_msgs::PointCloud2 1)
(ros::advertise "/hand_contact" jsk_recognition_msgs::ContactSensorArray 1)
(ros::rate 20)

(let* (camera-point-list
       hand-point-list
       camera-pc-msg
       hand-pc-msg
       contact-msg-list
       contact-array-msg
       stamp
       )
  ;; setup point cloud of camera (simulate depth camera) and hand (simulate proximity sensor)
  (dotimes (x 100)
    (dotimes (y 200)
      (push (float-vector (- x 50) (- y 100) (+ 400 (random 50.0))) camera-point-list)
      ))
  (dotimes (x 10)
    (dotimes (y 20)
      (push (float-vector (- x 5) (- y 10) (+ 40 (random 10.0))) hand-point-list)
      (push (float-vector (- x 5) (- y 10) (+ -100 (random 10.0))) hand-point-list)
      ))
  (setq camera-pc-msg
        (make-ros-msg-from-eus-pointcloud
         (instance pointcloud :init :points (apply #'matrix camera-point-list))
         :frame "/wide_stereo_optical_frame"))
  (setq hand-pc-msg
        (make-ros-msg-from-eus-pointcloud
         (instance pointcloud :init :points (apply #'matrix hand-point-list))
         :frame "/r_gripper_tool_frame"))
  ;; setup contact information (set no contact detection, so updated only with self link occupancy)
  (dolist (link-name (list "l_gripper_palm_link" "l_gripper_r_finger_link" "l_gripper_l_finger_link"))
    (push
     (instance jsk_recognition_msgs::ContactSensor :init
               :contact nil :link_name link-name)
     contact-msg-list)
    )
  (setq contact-array-msg
        (instance jsk_recognition_msgs::ContactSensorArray :init
                  :header (instance std_msgs::Header :init :frame_id "/octomap_world")
                  :datas contact-msg-list))
  (while (ros::ok)
    (setq stamp (ros::time-now))
    (send camera-pc-msg :header :stamp stamp)
    (send hand-pc-msg :header :stamp stamp)
    (send contact-array-msg :header :stamp stamp)
    (ros::publish "/camera_cloud" camera-pc-msg)
    (ros::publish "/hand_cloud" hand-pc-msg)
    (ros::publish "/hand_contact" contact-array-msg)
    (ros::sleep)
    ))
(ros::roseus "shutdown")
(exit)
