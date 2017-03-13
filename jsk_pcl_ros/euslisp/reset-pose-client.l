#!/usr/bin/env roseus

(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")
(ros::roseus-add-srvs "jsk_recognition_msgs")

(ros::roseus "reset_pose_client")


;; setup ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun setup
  (&key
   (object-shape nil)
   )
  ;; setup variable
  (unless object-shape
    (setq object-shape (ros::get-param "~object_shape" "cube")))
  (setup-saved-pose object-shape)
  (setq *marker-pose-msg* nil)

  ;; setup ros
  (ros::rate 20)
  (ros::subscribe "/object_marker/pose" geometry_msgs::PoseStamped #'pose-cb)
  (ros::advertise-service "/reset_object_pose_with_marker" std_srvs::Empty #'reset-with-marker-cb)
  (ros::advertise-service "/reset_object_pose_with_saved" std_srvs::Empty #'reset-with-saved-cb)
  )

(defun setup-saved-pose
  (object-shape)
  (let* (saved-coords
         frame-id
         )
    (cond ((string= object-shape "pepperbox")
           ;; (setq saved-coords (make-coords :pos (float-vector -364.696 308.04 711.733) :rpy (list -1.21952 0.695578 -1.01755)))
           ;; (setq frame-id "camera_rgb_optical_frame")
           (setq saved-coords (make-coords :pos (float-vector 936.392 140.681 297.545) :rpy (list -3.04521 -1.554 -1.41545)))
           (setq frame-id "ground")
           )
          ((string= object-shape "cart")
           (setq saved-coords (make-coords :pos (float-vector 510.225 -220.998 -711.502) :rpy (list -0.201385 0.039023 -0.051783)))
           (setq frame-id "odom")
           )
          ((string= object-shape "door")
           (setq saved-coords (make-coords :pos (float-vector 89.7771 518.528 1300.4) :rpy (list 1.76417 0.552361 -2.67219)))
           (setq frame-id "camera_rgb_optical_frame")
           )
          ((string= object-shape "cupboard")
           (setq saved-coords (make-coords :pos (float-vector 1722.8 -2.96322 -154.72) :rpy (list -0.112008 0.128574 -0.047535)))
           (setq frame-id "odom")
           )
          ((string= object-shape "table")
           (setq saved-coords (make-coords :pos (float-vector 1306.0 -50.1156 -193.006) :rpy (list -0.009615 0.154302 0.014597)))
           (setq frame-id "odom")
           )
          ((string= object-shape "cardboardbox")
           (setq saved-coords (make-coords :pos (float-vector -234.808 149.317 2322.65) :rpy (list -1.38561 -1.19024 -0.024554)))
           (setq frame-id "camera_rgb_optical_frame")
           )
          (t
           (setq saved-coords (make-coords))
           (setq frame-id "camera_rgb_optical_frame")
           ))
    (setq *saved-pose-msg* (ros::coords->tf-pose-stamped saved-coords frame-id))
    ))


;; callback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pose-cb
  (msg)
  (setq *marker-pose-msg* msg)
  )

(defun reset-with-marker-cb
  (req)
  (let* ((m (send req :response))
         (req)
         )
    (when *marker-pose-msg*
      (setq req (instance jsk_recognition_msgs::ResetPoseRequest :init :pose *marker-pose-msg*))
      (ros::service-call "/element_matching_pose_estimation/reset_estimated_pose" req t)
      )
    m))

(defun reset-with-saved-cb
  (req)
  (let* ((m (send req :response))
         (req)
         )
    (when *saved-pose-msg*
      (setq req (instance jsk_recognition_msgs::ResetPoseRequest :init :pose *saved-pose-msg*))
      (ros::service-call "/element_matching_pose_estimation/reset_estimated_pose" req t)
      )
    m))


(setup)
(ros::spin)
