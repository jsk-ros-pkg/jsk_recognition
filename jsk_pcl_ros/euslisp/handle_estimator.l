#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "jsk_pcl_ros")

(defclass handle-estimator
  :slots
  (*gripper-size*
   *approach-offset*
   *angle-difference*
   *candidate-num*
   *candidate-buffer*
   *best-topic*
   *best-preapproach-topic*
   *selected-topic*
   *selected-preapproach-topic*
   *candidate-topic*
   *candidate-preapproach-topic*
   *tfl*
   *camera-frame*
   ))

(defmethod handle-estimator
  (:init
   ()
   (setq *nodename* "handle_estimator")
   (ros::roseus *nodename*)

   (setq *candidate-buffer* (make-list 100))

   ;;get params
   (setq *gripper-size* (ros::get-param "~gripper_size" 0.08))
   (setq *approach-offset* (ros::get-param "~approach_offset" 0.1))
   (setq *angle-difference* (deg2rad (ros::get-param "~angle_difference" 20)))
   (setq *candidate-num* (ros::get-param "~candidate_num" 7))

   ;;advertise
   (setq *best-topic* (format nil "~A/output_best" *nodename*))
   (setq *best-preapproach-topic* (format nil "~A/output_best_preapproach" *nodename*))
   (ros::advertise *best-topic* geometry_msgs::PoseStamped)
   (ros::advertise *best-preapproach-topic* geometry_msgs::PoseStamped)

   (setq *selected-topic* (format nil "~A/output_selected_pose" *nodename*))
   (setq *selected-preapproach-topic* (format nil "~A/output_selected_preapproach" *nodename*))
   (ros::advertise *selected-topic* geometry_msgs::PoseStamped)
   (ros::advertise *selected-preapproach-topic* geometry_msgs::PoseStamped)

   (setq *candidate-topic* (format nil "~A/output" *nodename*))
   (setq *candidate-preapproach-topic* (format nil "~A/output_preapproach" *nodename*))
   (ros::advertise *candidate-topic* geometry_msgs::PoseArray)
   (ros::advertise *candidate-preapproach-topic* geometry_msgs::PoseArray)
   
   ;;subscribe
   (ros::subscribe (format nil "~A/input_box" *nodename*) jsk_recognition_msgs::BoundingBox #'send self :estimate)
   (ros::subscribe (format nil "~A/selected_index" *nodename*) jsk_recognition_msgs::Int32Stamped #'send self :selected-index-callback)

   (setq *tfl* (instance ros::transform-listener :init))
   (setq *camera-frame* (ros::get-param "~camera_frame" "head_mount_link"))

   (ros::spin)
   )

  (:selected-index-callback
   (index)
   (dolist (candidate-list *candidate-buffer*)
     (let ((candidate-pa (car candidate-list))
	   (candidate-preapproach-pa (cadr candidate-list))
	   )
       (when (equal (send candidate-pa :header :stamp) (send index :header :stamp))
	 ;;selected pose
	 (setq selected-ps (instance geometry_msgs::PoseStamped :init))
	 (send selected-ps :header (send candidate-pa :header))
	 (send selected-ps :pose (elt (send candidate-pa :poses) (send index :data)))

	 (ros::publish *selected-topic* selected-ps)
	 ;;selected preapproach pose
	 (setq selected-preapproach-ps (instance geometry_msgs::PoseStamped :init))
	 (send selected-preapproach-ps :header (send candidate-preapproach-pa :header))
	 (send selected-preapproach-ps :pose (elt (send candidate-preapproach-pa :poses) (send index :data)))
	 (ros::publish *selected-preapproach-topic* selected-preapproach-ps)
	 (return-from :selected-index-callback t)
	 )
       )
     )
   )

  (:estimate
   (box)
   (let ((c (send *tfl* :lookup-transform (send box :header :frame_id) *camera-frame* (ros::time 0)))
	 (p (ros::tf-pose->coords (send box :pose))))
     (print (send box :header :frame_id))
     (when (<= (vector-angle (matrix-column (send c :worldrot) 0) (matrix-column (send p :worldrot) 2)) pi/2)
       (send p :rotate pi :x)
       (send box :pose (ros::coords->tf-pose p))))

   (let* ((dimensions (send box :dimensions))
	  (x (send dimensions :x))
	  (y (send dimensions :y))
	  (z (send dimensions :z))
	  (handle-type :no-handle))
     (cond
      ((> z (max x y))
       (cond
	((< (max x y) *gripper-size*)
	 (setq handle-type :stand-on))
	((< x *gripper-size*)
	 (setq handle-type :stand-on-grasp-y))
	((< y *gripper-size*)
	 (setq handle-type :stand-on-grasp-x))
	))
      ((and (> x (max y z)) (< y *gripper-size*))
       (setq handle-type :lie-grasp-z-around-x))
      ((and (> y (max x z)) (< x *gripper-size*))
       (setq handle-type :lie-grasp-z-around-y))
      )
     (ros::ros-info "handle type ~A" handle-type)
     (send self :estimate-handle box handle-type)
     ))
  
  (:estimate-handle
   (box handle-type)
   (case handle-type
     (:stand-on
      (send self :handle-stand-on-grasp-y box)
      )
     (:stand-on-grasp-x
      (send self :handle-stand-on-grasp-x box)
       )
     (:stand-on-grasp-y
      (send self :handle-stand-on-grasp-y box)
      )
     (:stand-on-grasp-y
      (send self :handle-stand-on-grasp-y box)
      )
     (:lie-grasp-z-around-y
      (send self :handle-lie-grasp-z-around-y box)
      )
     (:lie-grasp-z-around-x
      (send self :handle-lie-grasp-z-around-x box)
      )
     ))

  (:handle-stand-on-grasp-x
   (box)
   (setq best-coords (ros::tf-pose->coords (send box :pose)))
   (send self :publish-poses box best-coords)
   )

  (:handle-stand-on-grasp-y
   (box)
   (setq best-coords (ros::tf-pose->coords (send box :pose)))
   (send best-coords :rotate pi/2 :z)
   (send self :publish-poses box best-coords)
   )
  
  (:handle-lie-grasp-z-around-y
   (box)
   (setq best-coords (ros::tf-pose->coords (send box :pose)))
   (send best-coords :rotate pi/2 :z)
   (send best-coords :rotate pi/2 :y)
   (send self :publish-poses box best-coords)
   )

  (:handle-lie-grasp-z-around-x
   (box)
   (setq best-coords (ros::tf-pose->coords (send box :pose)))
   (send best-coords :rotate pi/2 :y)
   (send self :publish-poses box best-coords)
   )

  (:publish-poses
   (box best-coords)

   ;;best pose
   (setq best-ps (instance geometry_msgs::PoseStamped :init))
   (send best-ps :header (send box :header))
   (send best-ps :pose (ros::coords->tf-pose best-coords))
   (ros::publish *best-topic* best-ps)

   ;;preapproach pose
   (send best-ps :pose (ros::coords->tf-pose (send (send best-coords :copy-worldcoords) :locate (float-vector (- (* *approach-offset* 1000)) 0 0))))
   
   (ros::publish *best-preapproach-topic* best-ps)

   ;;candidate poses
   (let ((candidate-pa (instance geometry_msgs::PoseArray :init))
	 (candidate-preapproach-pa (instance geometry_msgs::PoseArray :init))
	 candidate-poses
	 candidate-preapproach-poses
	 candidate-indexes
	 )
     (send candidate-pa :header (send box :header))
     (send candidate-preapproach-pa :header (send box :header))

     (dotimes (i *candidate-num*)
       (push (- i (/ *candidate-num* 2)) candidate-indexes))

     (dolist (i candidate-indexes)
       (let (candidate-coords
	     candidate-pose)
	 ;;candidate pose
	 (setq candidate-coords (send (send best-coords :copy-worldcoords) :rotate (* *angle-difference* i) :z))
	 (setq candidate-pose (ros::coords->tf-pose candidate-coords))
	 (push candidate-pose candidate-poses)

	 ;;preapproach pose
	 (send candidate-coords :locate (float-vector (- (* *approach-offset* 1000)) 0 0))
	 (setq candidate-pose (ros::coords->tf-pose candidate-coords))
	 (push candidate-pose candidate-preapproach-poses)
	 )
       )
     (send candidate-pa :poses candidate-poses)
     (ros::publish *candidate-topic* candidate-pa)
     
     (send candidate-preapproach-pa :poses candidate-preapproach-poses)
     (ros::publish *candidate-preapproach-topic* candidate-pa)
     
     (setq *candidate-buffer* (append (list (list candidate-pa candidate-preapproach-pa)) (butlast *candidate-buffer*)))
     )
   )
  )

(instance handle-estimator :init)
