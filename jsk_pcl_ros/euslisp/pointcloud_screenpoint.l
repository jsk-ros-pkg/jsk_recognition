#!/usr/bin/env roseus
;;
;; convert screenpoint from image_view2 to 3D point
;;
;; publish:
;;  ray_marker_array (visualization_msgs::MarkerArray)
;;    marker of touched point for rviz.
;;
;;  image_marker (image_view2::ImageMarker2)
;;    marker of touched point for image_view2.
;;
;;  ray_coords (geometry_msgs::PoseStamped)
;;    pose of 3D touched point, frame_id is copied from image.
;;
;;
;; subscribe:
;;  $(param sensor_topic)/screenpoint (geometry_msgs::PointStamped)
;;    screen point from image_view2.
;;
;;
;; TF:
;;  /ray_target
;;    tf of 3D touched point, frame_id is copied from image.
;;
(ros::load-ros-manifest "jsk_recognition_msgs")
(ros::load-ros-manifest "image_view2")

(defun ros::tf-translation->pos (trans)
  "convert ros::tf-translation to float-vector
@param trans ros::tf-translation
@return translation as float-vector
"
  (float-vector (send trans :x) (send trans :y) (send trans :z)))

;;;;
;;;;
;;;;

(defun visualize-point (x y &key ((:lifetime lf) 5))
  "publish marker of touchd point to image_view2
@param x x[px]
@param y y[px]
@param :lifetime life time [s]"
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
  "publish marker of frame coordinate to image_view2
@param frame frame id
@param :lifetime life time [s]"
  (let ((mrk (instance image_view2::ImageMarker2 :init)))
    (send mrk :header :stamp (ros::time-now))
    (send mrk :type image_view2::ImageMarker2::*FRAMES*)
    (send mrk :frames frame)
    (send mrk :lifetime (ros::Time lf))
    (ros::publish "image_marker" mrk)))

;;;
;;;
;; screen point is stored in *screenpoint*
(defvar *screenpoint* nil)
(defun point-cb (msg)
  "subscriber callback
@param msg subscribed msg"
  (let* ((x (send msg :point :x))
	 (y (send msg :point :y))
	 (req (instance jsk_recognition_msgs::TransformScreenpointRequest :init
			:x x :y y))
         res)
    ;; call PointcloudScreenPointNodelet::screen_to_point
     (ros::wait-for-service *ray_srv*)
     (setq res (ros::service-call *ray_srv* req))
     ;; send marker to image_view2
     (visualize-point x y)
     ;; store result to *screenpoint*
     (setq *screenpoint* res)
     ;; check
     (print (list (send res :vector :x)
		  (send res :vector :y)
		  (send res :vector :z)))
    ))

(defun show-marker (frame p1 v1)
  "publish markers
@param frame frame id
@param p1 postion
@param v1 vector"
  (let* ((header (instance std_msgs::header :init
			   :stamp (ros::time-now) :frame_id frame))
	 (p0 (float-vector 0 0 0))
	 (sp (make-sphere 30 :pos p1))
   ;; Marker::SPHERE
	 (sp-msg (sphere->marker-msg sp header
				     :color (float-vector 1 0 1) :alpha 1.0))
   ;; Marker::LINE
	 (li-msg (line->marker-msg (list p0 p1) header :scale 15
				     :color (float-vector 0 1 1) :alpha 0.5))
   ;; MarkerArray msg
	 (msg (instance visualization_msgs::MarkerArray :init))
   ;; PoseStamped msg
	 (rmsg (instance geometry_msgs::PoseStamped :init))
	 (av v1) (xv #f(1 0 0)) (bv (v* av xv)) (cv (v* av bv))
	 (dm (matrix (normalize-vector av)
		     (normalize-vector bv)
		     (normalize-vector cv)))
   ;; camera coordinates from base_footprint
	 (cam-cds (send *tfl* :lookup-transform
                    *base-frame* frame (ros::time 0)))
   base-cds)  ;; (let*)
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
    ;; send marker to image_view2
    (visualize-frame (list "ray_target"))

    ;; send coords based on "base_footprint"
    (cond ((> (distance (send *cds* :pos) (send base-cds :pos)) 10)
	   (ros::ros-warn "estimated cds is  ~A" base-cds)
	   (send rmsg :header header)
	   (send rmsg :pose (ros::coords->tf-pose base-cds))
	   (ros::publish "ray_coords" rmsg)))
    (setq *cds* base-cds)
    ))

(defun init-settings ()
  "initializer"
  ;; TF broadcaster and listener
  (setq *tfb* (instance ros::transform-broadcaster :init))
  (setq *tfl* (instance ros::transform-listener :init))

  ;; topics are published under *sensor_topic*
  (setq *sensor_topic* (ros::get-param "~sensor_topic"))
  ;; service of PointcloudScreenpointNodelet
  (setq *ray_srv* (ros::get-param "~ray_srv"))
  ;; set base frame
  (setq *base-frame* (ros::get-param "~base_frame" "base_footprint"))
  ;; to store previous coords
  (setq *cds* (make-coords))

  (ros::advertise "ray_marker_array" visualization_msgs::MarkerArray 10)
  (ros::advertise "image_marker" image_view2::ImageMarker2 10)
  (ros::advertise "ray_coords" geometry_msgs::PoseStamped 1)
  (ros::subscribe (format nil "~A~A" *sensor_topic* "/screenpoint")
		  geometry_msgs::PointStamped #'point-cb)

  ;; check topic and service
  (ros::ros-warn "pointcloud_screenpoint:sensor -> ~A" *sensor_topic*)
  (ros::ros-warn "pointcloud_screenpoint:ray_srv -> ~A" *ray_srv*)
  )


;; main routine
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




