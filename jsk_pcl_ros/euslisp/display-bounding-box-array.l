#!/usr/bin/env runeus

(ros::load-ros-manifest "jsk_pcl_ros")

(defvar *topic-name* "/cluster_decomposer/boxes")
(defvar *bounding-box-list* nil)

(ros::roseus "boundingboxarray_subscriber")

(defun bounding-box-array-cb (msg)
  (setq *bounding-box-list* (send msg :boxes))
  (when *bounding-box-list*
    (send *irtviewer* :draw-objects :flush nil)
    (mapcar #'(lambda (b)
                (let* ((dims (ros::tf-point->pos (send b :dimensions)))
                       (bx (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
                  (send bx :newcoords (ros::tf-pose->coords (send b :pose)))
                  (send bx :worldcoords)
                  (send bx :draw-on :flush nil :color #f(1 0 0))
                  bx))
            *bounding-box-list*)
    (send *irtviewer* :viewer :viewsurface :flush)
    ))

(ros::subscribe *topic-name* jsk_pcl_ros::BoundingBoxArray #'bounding-box-array-cb 1)

(unless (boundp '*irtviewer*) (make-irtviewer))

(do-until-key
 (x::window-main-one)
 (ros::spin-once)
 (ros::sleep)
 )
;;(exit)
