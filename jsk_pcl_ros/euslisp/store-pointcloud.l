#!/usr/bin/env roseus

;; (setq ros::*compile-message* t) ;; switch for using compiled messages
(ros::load-ros-manifest "jsk_pcl_ros")

(defvar *topic-name* "/openni/depth_registered/points_throttle")
;;(defvar *topic-name* "/openni/depth_registered/points")
;;(defvar *store-points* nil)
;;(defvar *transform-points* nil) ;; set frame_id
(defvar *store-points* t)
(defvar *transform-points* "/base_footprint")

(unless (boundp '*irtviewer*) (make-irtviewer))
(setq *3dpos* (instance pointcloud :init))
(objects (list *3dpos*))

(if *store-points* (setq *lst* nil))

(defun pcloud-cb (msg)
  (pprint 'capture)
  (make-eus-pointcloud-from-ros-msg msg :pcloud *3dpos*)
  (when *store-points*
    (let ((pt (copy-object *3dpos*)))
      (setf (get pt :header) (send msg :header))
      (push pt *lst*)))
  (send *irtviewer* :draw-objects))

(ros::roseus "pointcloud_subscriber")

(when *transform-points*
  (setq *tfl* (instance ros::transform-listener :init 15.0)))

(ros::subscribe *topic-name* sensor_msgs::PointCloud2 #'pcloud-cb 100)

(defun ros-spin ()
  (let (lastpts pts)
    (do-until-key
     (ros::spin-once)
     (when *transform-points*
       (unless (eq lastpts (setq pts (car *lst*)))
         (when (send *tfl* :wait-for-transform
                     *transform-points* (send (get pts :header) :frame_id)
                     (send (get pts :header) :stamp) 0.5)
           (let ((cds (send *tfl* :lookup-transform
                            *transform-points* (send (get pts :header) :frame_id)
                            (send (get pts :header) :stamp))))
             (setf (get pts :transcoords) cds)
             (send pts :transform-points cds)
             ))
         (setq lastpts pts)))
     (x::window-main-one)
     )
    ))

(defun ros-shutdown ()
  (ros::roseus "shutdown")
  (exit))

(ros-spin)

;;
;; (setq *lst* (remove-if #'(lambda (x) (null (get x :transcoords))) *lst*))
;; (setq slst (send-all *lst* :step 2 :create t))
;; (send-all slst :filter :key #'(lambda (p) (not (c-isnan (elt p 0)))))
;;
