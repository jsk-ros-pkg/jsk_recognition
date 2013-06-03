#!/usr/bin/env roseus
(ros::load-ros-manifest "jsk_perception")

;; define detection parameters befor load detection_interface.l
;;(defvar *detection-topic* "/ObjectDetection")
(defvar *detection-topic* "/openni/rgb/ObjectDetection")
;;(defvar *detection-topic* "/narrow_stereo/left/ObjectDetection")

;; load functions  
(load "package://jsk_perception/euslisp/detection_interface.l")

;; stret program
(ros::roseus "objectdetection_publisher")

;;(load "package://euslisp/jskeus/eus/models/room610-chair-object.l")
;;(setq *target-object* (room610-chair))
;;(load "package://euslisp/jskeus/eus/models/room610-laundry-machine-object.l")
;;(setq *target-object* (room610-laundry-machine))
(setq *target-object* (make-cube 300 300 500))

(ros::roseus "object_detection_marker")
(ros::rate 10)
(do-until-key
 (send *target-object* :move-to (make-coords) :world)
 (check-detection :type nil ;; work for any object
		  :speak-name "test object"
		  :target-object *target-object*
		  :speak nil)
 (ros::spin-once)
 (ros::sleep))
