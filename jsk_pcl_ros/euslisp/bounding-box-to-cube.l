#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(defun bounding-box->cube (b)
  (let* ((dims (ros::tf-point->pos (send b :dimensions)))
         (cube (make-cube (elt dims 0) (elt dims 1) (elt dims 2))))
    (send cube :newcoords (ros::tf-pose->coords (send b :pose)))
    cube))
