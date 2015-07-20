#!/usr/bin/env roseus
(load "~/73b2-cupboard-without-door-object.l")
(ros::roseus "marker_model_tracking_demo")
(ros::load-ros-manifest "visualization_msgs")

(setq *cupboard* (73b2-cupboard-without-door))
(ros::ros-info "Marker Model Tracking Demo will reset the tracker target !!")
(ros::advertise "/particle_filter_tracker/renew_model_with_marker" visualization_msgs::Marker)
(unix::sleep 3)

(dotimes (i 5)
  (ros::publish "/particle_filter_tracker/renew_model_with_marker"
                (object->marker-msg (send *cupboard* :translate #f(500 0 0)) (instance std_msgs::header :init :stamp (ros::time-now) :frame_id "/base_link")))
  (ros::ros-info "== publish to renew model with marker in marker_model_tracking_demo (~A / 5) ==" i)
  (unix::sleep 7)
  )
(ros::ros-info "end publishing marker model demo")
(exit)
