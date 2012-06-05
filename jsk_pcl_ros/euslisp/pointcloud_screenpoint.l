#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_pcl_ros")
(ros::load-ros-manifest "image_view2")

(defun ros::tf-translation->pos (trans)
  (float-vector (send trans :x) (send trans :y) (send trans :z)))

;;;;
;;;;
;;;;

(defun visualize-point (x y &key ((:lifetime lf) 5))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :id 1)
    (send mrk :type image_view2::ImageMarker2::*CIRCLE*)
    (send mrk :action image_view2::ImageMarker2::*ADD*)
    (send mrk :position (instance geometry_msgs::Point :init :x x :y y))
    (send mrk :outline_color (instance std_msgs::ColorRGBA :init :r 0.0 :g 1.0 :b 0.0 :a 1.0))
    (send mrk :scale 15)
    (send mrk :lifetime (ros::Time lf))
    (ros::publish "image_marker" mrk)))

(defun visualize-frame (frame &key ((:lifetime lf) 10))
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*FRAMES*)
    (send mrk :frames frame)
    (send mrk :lifetime (ros::Time lf))
    (ros::publish "image_marker" mrk)))

;;;
;;;
(defvar *screenpoint* nil)
(defun point-cb (msg)
  (let* ((x (send msg :point :x))
	 (y (send msg :point :y))
	 (req (instance jsk_pcl_ros::TransformScreenpointRequest :init
			:x x :y y))
         res)
     (ros::wait-for-service *ray_srv*)
     (setq res (ros::service-call *ray_srv* req))
     (visualize-point x y)
     (setq *screenpoint* res)
     (print (list (send res :vector :x)
		  (send res :vector :y)
		  (send res :vector :z)))
    ))

(defun show-marker (frame p1 v1)
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id frame))
	 (p0 (float-vector 0 0 0))
	 (sp (make-sphere 30 :pos p1))
	 (sp-msg (sphere->marker-msg sp header
				     :color (float-vector 1 0 1) :alpha 1.0))
	 (li-msg (line->marker-msg (list p0 p1) header :scale 15
				     :color (float-vector 0 1 1) :alpha 0.5))
	 (msg (instance visualization_msgs::MarkerArray :init))
	 (rmsg (instance geometry_msgs::PoseStamped :init))
	 (av v1) (xv #f(1 0 0)) (bv (v* av xv)) (cv (v* av bv))
	 (dm (matrix (normalize-vector av)
		     (normalize-vector bv)
		     (normalize-vector cv)))
	 (cam-cds (send *tfl* :lookup-transform
			"base_footprint" frame (ros::time 0))) base-cds)
    (cond ((null frame)
	   (ros::ros-warn "detected service fail")
	   (return-from show-marker nil))
	  ((null cam-cds)
	   (ros::ros-warn "transform fail")
	   (return-from show-marker nil)))
    (setq base-cds (send cam-cds :transform (make-coords :pos p1 :rot dm)))
    (send sp-msg :ns "test_sphere")
    (send li-msg :ns "test_line")
    (send sp-msg :lifetime (ros::time 5))
    (send li-msg :lifetime (ros::time 5))
    (send msg :markers (list sp-msg li-msg))
    (ros::publish "ray_marker_array" msg)
    (send *tfb* :send-transform (make-coords :pos p1) frame "/ray_target")
    (visualize-frame (list "ray_target"))

    ;;send coords based on "base_footprint"
    (cond ((> (distance (send *cds* :pos) (send base-cds :pos)) 10)
	   (ros::ros-warn "estimated cds is  ~A" base-cds)
	   (send rmsg :header header)
	   (send rmsg :pose (ros::coords->tf-pose base-cds))
	   (ros::publish "ray_coords" rmsg)))
    (setq *cds* base-cds)
    ))

(defun init-settings ()
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (setq *tfl* (instance ros::transform-listener :init))
  (setq *sensor_topic* (ros::get-param "~sensor_topic"))
  (setq *ray_srv* (ros::get-param "~ray_srv"))
  (setq *cds* (make-coords))
  (ros::advertise "ray_marker_array" visualization_msgs::MarkerArray 10)
  (ros::advertise "image_marker" image_view2::ImageMarker2 10)
  (ros::advertise "ray_coords" geometry_msgs::PoseStamped 1)
  (ros::subscribe (format nil "~A~A" *sensor_topic* "/screenpoint")
		  geometry_msgs::PointStamped #'point-cb)
  (ros::ros-warn "pointcloud_screenpoint:sensor -> ~A" *sensor_topic*)
  (ros::ros-warn "pointcloud_screenpoint:ray_srv -> ~A" *ray_srv*)
  )

(ros::roseus "pointcloud_screenpoint")
(init-settings)
(ros::rate 10)
(while (ros::ok)
  (ros::spin-once)
  (ros::sleep)
  (when (and *screenpoint*
	     (> (send (send *screenpoint* :header :stamp) :to-sec) 0.0)
	     (< (send (ros::time- (ros::time-now) (send *screenpoint* :header :stamp)) :to-sec) 20))
    (show-marker (send *screenpoint* :header :frame_id)
		 (ros::tf-point->pos (send *screenpoint* :point))
		 (ros::tf-translation->pos (send *screenpoint* :vector))))
  )
(ros::exit)




