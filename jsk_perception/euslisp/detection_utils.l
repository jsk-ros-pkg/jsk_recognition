#!/usr/bin/env roseus

(defvar *detection-topic* "/openni/rgb/ObjectDetection")
(defvar *base-frame-id* "/base_footprint")
(defvar *solve-tf* t)
(ros::load-ros-manifest "roseus")
(ros::roseus "marker-publish")

(ros::roseus-add-msgs "posedetection_msgs")

(defmethod coordinates
  (:speak-name
   (&optional s)
   (if s (setf (get self :speak-name) s)
     (let ((nm (get self :speak-name)))
       (if nm nm (send self :name)))))
  (:stamp (&optional s) (if s (setf (get self :stamp) s)) (get self :stamp))
  (:frame_id (&optional f) (if f (setf (get self :frame_id) f)) (get self :frame_id)))

(defun detection-interface-objectdetection (force target-obj msg)
  (let ((obj-lst (send msg :objects))
        ;;(av (send *pr2* :angle-vector))
        (frame (send msg :header :frame_id))
        (tp (get target-obj :type))
        stamp trans)
    (ros::ros-info "object detection callback target ~A(~A), frame ~A, object ~A"
                   tp (stringp tp) frame target-obj)
    (dolist (obj obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (setq stamp (if force (ros::time 0) (send msg :header :stamp)))
        (cond
         (*solve-tf*
          (unless (boundp '*tfl*)
            (setq *tfl* (instance ros::transform-listener :init)))
          (unless (send *tfl* :wait-for-transform *base-frame-id* frame stamp 1)
            (ros::ros-error "could not solve ~A to ~A" *base-frame-id* frame))
          (setq trans (send *tfl* :lookup-transform *base-frame-id* frame stamp))
          (ros::ros-info "trans ~A" trans))
         (t (setq trans (make-coords))))

        (when trans
          (ros::ros-info "object detected ~A ~A ~20,9f" (send obj :type)
                         (ros::tf-pose->coords (send obj :pose))
                         (send (send msg :header :stamp) :to-sec))
          (send target-obj :name (send obj :type))
          (send target-obj :stamp (send msg :header :stamp))
          (send target-obj :frame_id (send msg :header :frame_id))
          ;; (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
          (send target-obj :reset-coords)
          (send target-obj :move-to trans :local)
          (send target-obj :move-to (ros::tf-pose->coords (send obj :pose)) :local)
          (when (find-method target-obj :update-descendants)
            (send target-obj :update-descendants) ;; clean me
            )
          (unless (boundp '*tfb*)
            (setq *tfb* (instance ros::transform-broadcaster :init)))
          (send *tfb* :send-transform (ros::tf-pose->coords (send obj :pose)) frame (send target-obj :name))
          ;; target-obj should be on *base-frame-id* coordinates
          ;; (send *pr2* :angle-vector av)
          ))
      )
    ))

(defun start-detection-interface-subscribe (tf-force obj &optional (topic *detection-topic*))
  (ros::ros-info "start subscribe ~A" topic)
  (ros::subscribe topic
                  posedetection_msgs::ObjectDetection
                  #'detection-interface-objectdetection tf-force obj))
(defun stop-detection-interface-subscribe (&optional (topic *detection-topic*))
  (ros::ros-info "stop subscribe ~A" topic)
  (ros::unsubscribe topic))

(defun publish-marker (target-obj marker-life &key (publish-name "/object_detection_marker_array"))
  (ros::publish publish-name
                (instance visualization_msgs::MarkerArray :init
                          :markers
                          (list (text->marker-msg
                                 (send target-obj :name)
                                 (send (send (send target-obj :copy-worldcoords) :copy-worldcoords)
                                       :translate #f(-100 0 100))
                                 (instance std_msgs::header :init
                                           :stamp (send target-obj :stamp)
                                           :frame_id *base-frame-id*)
                                 :ns (format nil "object_text_~A" (send target-obj :name)) :color #f(1 0 0)
                                 :scale 100 :lifetime marker-life :id 0)
                                (coords->marker-msg
                                 (send target-obj :copy-worldcoords)
                                 (instance std_msgs::header :init
                                           :stamp (send target-obj :stamp)
                                           :frame_id *base-frame-id*)
                                 :ns (format nil "object_coords_~A" (send target-obj :name))
                                 ;; :size 200 :width 20
                                 :lifetime marker-life :id 1)
                                (object->marker-msg
                                 target-obj
                                 (instance std_msgs::header :init
                                           :stamp (send target-obj :stamp)
                                           :frame_id *base-frame-id*)
                                 :ns (format nil "object_body_~A" (send target-obj :name))
                                 :lifetime marker-life :id 2)
                                ))))

