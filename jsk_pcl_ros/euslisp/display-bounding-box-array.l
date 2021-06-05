#!/usr/bin/env roseus

(defvar *topic-name* "/cluster_decomposer/boxes")
(defvar *bounding-box-list* nil)

(load "package://jsk_pcl_ros/euslisp/bounding-box-to-cube.l")

(ros::roseus "boundingboxarray_subscriber")

(defun bounding-box-array-cb (msg)
  (setq *bounding-box-list* (send msg :boxes))
  (when *bounding-box-list*
    (send *irtviewer* :draw-objects :flush nil)
    (dolist (cube (mapcar #'bounding-box->cube *bounding-box-list*))
      (send cube :draw-on :flush nil :color #f(1 0 0)))
    (send *irtviewer* :viewer :viewsurface :flush)
    ))

(ros::subscribe *topic-name* jsk_recognition_msgs::BoundingBoxArray #'bounding-box-array-cb 1)

(unless (boundp '*irtviewer*) (make-irtviewer))

(do-until-key
 (x::window-main-one)
 (ros::spin-once)
 (ros::sleep)
 )
;;(exit)
