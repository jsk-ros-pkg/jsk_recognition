#!/usr/bin/env roseus

(load "package://roseus/euslisp/roseus-utils.l")

(ros::roseus "publish_orbit_marker")

(defvar *base-frame-id* "/eng8/6f/610")

(if (not (boundp '*tfb*)) (setq *tfb* (instance ros::transform-broadcaster :init)))

(defun tf-orbit-broadcaster ()
  (ros::rate 10)
  (do-until-key
     (send *tfb* :send-transform
           (make-coords :pos #f(5050.0 -1000.0 700.0) :rpy (float-vector pi 0 0)) ;;table
           ;;(make-coords :pos #f(5500.0 2000.0 800.0) :rpy (float-vector -pi/2 0 0)) ;;shelf
           ;;(make-coords :pos #f(7000.0 2000.0 1700.0) :rpy (float-vector -pi/2 0 0)) ;;on refrigerator
           *base-frame-id* "/orbit_base_link")
     (ros::spin-once)
     (ros::sleep)
     ))


(tf-orbit-broadcaster)