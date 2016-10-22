#!/usr/bin/env roseus

(ros::roseus "collision_detector_client")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "jsk_recognition_msgs")
(ros::roseus-add-msgs "jsk_rviz_plugins")

(defun setup-robot
  ()
  (load "package://hrpsys_ros_bridge/models/samplerobot.l")
  (setq *robot* (instance samplerobot-robot :init))
  ;; (objects (list *robot*))
  )

(defun set-random-angle-vector
  ()
  (send *robot* :reset-pose)
  (let* ((limbs (list :larm :rarm :torso))
         (joint-list
          (apply #'append (mapcar #'(lambda (limb) (send *robot* limb :joint-list)) limbs)))
         )
    (dolist (j joint-list)
      (let ((max-angle (min (send j :max-angle) 180))
            (min-angle (max (send j :min-angle) -180)))
        (send j :joint-angle (+ (random (- max-angle min-angle)) min-angle))
        ))
    ))

(defun set-random-stand-pose
  ()
  (let* ((x-min -1000) (x-max 1000)
         (y-min -1000) (y-max 1000)
         (yaw-min -pi) (yaw-max pi)
         (x (+ (random (- x-max x-min)) x-min))
         (y (+ (random (- y-max y-min)) y-min))
         (yaw (+ (random (- yaw-max yaw-min)) yaw-min))
         (stand-pose (make-coords :pos (float-vector x y 0) :rpy (list yaw 0 0)))
         )
    (send *robot* :fix-leg-to-coords stand-pose)
    )
  )

(defun request-collision-check
  ()
  (send *robot* :update-descendants)
  (let* ((tm (ros::time-now))
         (req (instance jsk_recognition_msgs::CheckCollisionRequest :init))
         res)
    (send req :joint
          (instance sensor_msgs::JointState :init
                    :header (instance std_msgs::Header :init :stamp tm)
                    :name (send-all (send *robot* :joint-list) :name)
                    :position (mapcar #'deg2rad (coerce (send *robot* :angle-vector) cons))))
    (send req :pose
          (instance geometry_msgs::PoseStamped :init
                    :header (instance std_msgs::header :init :frame_id "" :stamp tm)
                    :pose (ros::coords->tf-pose (send (car (send *robot* :links)) :worldcoords))))
    (setq res (ros::service-call "/collision_detector/collision_detector/check_collision" req))
    (publish-collision-status (send res :result))
    )
  )

(defun publish-collision-status
  (status)
  (let ((text-msg (instance jsk_rviz_plugins::OverlayText :init)))
    (send text-msg :bg_color :r 0.2)
    (send text-msg :bg_color :b 0.2)
    (send text-msg :bg_color :g 0.2)
    (send text-msg :bg_color :a 0.8)
    (cond (status
           (send text-msg :text "Collision !!!")
           (send text-msg :fg_color :r 1.0)
           (send text-msg :fg_color :g 0.0)
           (send text-msg :fg_color :b 0.0)
           (send text-msg :fg_color :a 1)
           )
          (t
           (send text-msg :text "No Collision")
           (send text-msg :fg_color :r 0.0)
           (send text-msg :fg_color :g 1.0)
           (send text-msg :fg_color :b 0.0)
           (send text-msg :fg_color :a 1)
           ))
    (ros::publish "/collision_status_text" text-msg)
    ))

(defun main
  ()
  (ros::rate 1)
  (do-until-key
   (set-random-angle-vector)
   (set-random-stand-pose)
   (request-collision-check)
   ;; (send *irtviewer* :draw-objects)
   (ros::spin-once)
   (ros::sleep)
   )
  )

(ros::advertise "/collision_status_text" jsk_rviz_plugins::OverlayText 1)
(ros::subscribe "/collision_detector/collision_detector/collision_status" std_msgs::Bool #'publish-collision-status)
(ros::wait-for-service "/collision_detector/collision_detector/check_collision")

(setup-robot)
(main)
