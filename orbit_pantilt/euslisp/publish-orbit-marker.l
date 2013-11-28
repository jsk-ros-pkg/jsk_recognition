#!/usr/bin/env roseus

(load "package://roseus/euslisp/roseus-utils.l")

(ros::roseus "publish_orbit_marker")

(defvar *base-frame-id* "/eng8/6f/610")

(if (not (boundp '*tfb*)) (setq *tfb* (instance ros::transform-broadcaster :init)))

(defun tf-orbit-broadcaster ()
  (ros::rate 10)
  (do-until-key
     (send *tfb* :send-transform (make-cascoords) *base-frame-id* "/orbit_base_link")
     (ros::spin-once)
     (ros::sleep)
     ))


(tf-orbit-broadcaster)