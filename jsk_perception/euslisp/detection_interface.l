;;
;;
(defvar *detection-topic* "/narrow_stereo/left/ObjectDetection")
(defvar *base-frame-id* "/base_footprint")
(defvar *solve-tf* t)

(ros::roseus-add-msgs "posedetection_msgs")
;;
(unless (ros::ok)
  (ros::roseus "detection_interface"))

(unless (boundp '*tfl*)
  (setq *tfl* (instance ros::transform-listener :init)))
(unless (boundp '*tfb*)
  (setq *tfb* (instance ros::transform-broadcaster :init)))
(ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)
(ros::ros-info "Publish for /objectdetection_marker_array")

(defmethod coordinates
  (:speak-name
   (&optional s)
   (if s (setf (get self :speak-name) s)
     (let ((nm (get self :speak-name)))
       (if nm nm (send self :name)))))
  (:stamp (&optional s) (if s (setf (get self :stamp) s)) (get self :stamp))
  (:frame_id (&optional f) (if f (setf (get self :frame_id) f)) (get self :frame_id)))

(defun objectdetection (force target-obj msg)
  (let ((obj (car (send msg :objects)))
        ;;(av (send *pr2* :angle-vector))
        (frame (send msg :header :frame_id))
        (tp (get target-obj :type))
        stamp trans)
    (ros::ros-info "object detection callback target ~A, frmae ~A, object ~A" tp frame (send obj :type))
    (when (or (not (stringp tp))
              (substringp tp (send obj :type))) ;; found target
      (setq stamp (if force (ros::time 0) (send msg :header :stamp)))
      (cond
       (*solve-tf*
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
	(send *tfb* :send-transform (ros::tf-pose->coords (send obj :pose)) frame (send target-obj :name))
        ;; target-obj should be on *base-frame-id* coordinates
        ;; (send *pr2* :angle-vector av)
        ))
    ))

(defun start-subscribe (tf-force obj)
  (ros::ros-info "start subscribe ~A" *detection-topic*)
  (ros::subscribe *detection-topic*
                  posedetection_msgs::ObjectDetection #'objectdetection tf-force obj))
(defun stop-subscribe ()
  (ros::ros-info "stop subscribe ~A" *detection-topic*)
  (ros::unsubscribe *detection-topic*))

(defun check-detection (&key (timeout 10) (type nil) (tf-force nil)
                             (diff-position 10.0) (single nil) (speak t)
                             (marker-life 30) (speak-name)
                             ((:target-object target-obj) (make-cube 60 60 60)))
  (let ((tgt-cds (send target-obj :copy-worldcoords)))
    (send target-obj :reset-coords)
    (setf (get target-obj :type) type)
    (when speak-name
      (send target-obj :speak-name speak-name))
    (start-subscribe tf-force target-obj) (unix::sleep 2)
    (send target-obj :stamp (ros::time-now))

    (let* (current-object-coords
           previous-object-coords
           (start-time (ros::time-now)) (detect-object-list))
      (ros::ros-info "check (timeout=~A, type=~A, tf-force=~A)" timeout type tf-force)
      (while (ros::ok)
        (ros::spin-once) ;; target-obj was updated in objectdetection
        ;; check target-obj is previously detected
        (when (and (> (send (send target-obj :stamp) :to-sec)
                      (send start-time :to-sec))
                   ;; discard object detection result before check function called
                   (not (member (send target-obj :stamp) detect-object-list))
                   ;; check if obj is already stored in detect-object-list
                   )
          #|(when speak
            (speak-jp (format nil "~a を さがして います"
                              (send target-obj :speak-name))))|#
          (push (send target-obj :stamp) detect-object-list)
          (setq current-object-coords (send target-obj :copy-worldcoords)) ;; updated position
          (ros::ros-info "current -> ~A (~A)" current-object-coords (send target-obj :name))
          (ros::publish "/object_detection_marker_array"
                        (instance visualization_msgs::MarkerArray :init
                                  :markers
                                  (list (text->marker-msg
                                         (send target-obj :name)
                                         (send (send current-object-coords :copy-worldcoords)
                                               :translate #f(-100 0 100))
                                         (instance std_msgs::header :init
                                                   :stamp (send target-obj :stamp)
                                                   :frame_id *base-frame-id*)
                                         :ns "object_text" :color #f(1 0 0)
                                         :scale 100 :lifetime marker-life :id 0)
                                        (coords->marker-msg
                                         current-object-coords
                                         (instance std_msgs::header :init
                                                   :stamp (send target-obj :stamp)
                                                   :frame_id *base-frame-id*)
                                         :ns "object_coords"
                                         ;; :size 200 :width 20
                                         :lifetime marker-life :id 1)
                                        (object->marker-msg
                                         target-obj
                                         (instance std_msgs::header :init
                                                   :stamp (send target-obj :stamp)
                                                   :frame_id *base-frame-id*)
                                         :ns "object_body"
                                         :lifetime marker-life :id 2)
                                        )))

          (ros::ros-info "object detected ~A diff-pos:~A, diff-rot:~A"
                         (send target-obj :name)
                         (if previous-object-coords
                             (norm (send previous-object-coords
                                         :difference-position current-object-coords)))
                         (if previous-object-coords
                             (elt (send previous-object-coords
                                        :difference-rotation current-object-coords
                                        :rotation-axis :xy) 2)))
          (when (or single
                    (and previous-object-coords
                         (< (norm (send previous-object-coords
                                        :difference-position current-object-coords))
                            diff-position))) ;; moved a little -> found obj
            (cond
             ((or (not (stringp type))
                  (substringp type (send target-obj :name))) ;; found target
              (ros::ros-info (format nil "ok - obj-type(~a)"  target-obj))
              (if (and (boundp '*irtviewer*) x::*display* (/= x::*display* 0))
                  (send *irtviewer* :draw-objects))
              (stop-subscribe)
              (when speak
                (speak-jp (format nil "~a が みつかりました"
                                  (send target-obj :speak-name))))
              (return-from check-detection target-obj))
              (t
               (ros::ros-info (format nil "no - obj-type(~a)" (send target-obj :name)))
              ))
            ) ;; when (and previous...
          (when (and (boundp '*irtviewer*) x::*display* (/= x::*display* 0))
            (send *irtviewer* :draw-objects)
            (x::window-main-one))
          (setq previous-object-coords current-object-coords)
          ;; set current coords ad previous coords
          ) ;; memq target-obj detect-object-list
        (when (< timeout
                 (send (ros::time- (ros::time-now) start-time) :to-sec))
          (ros::ros-info "timeout")
          (stop-subscribe)
          (when speak
            (speak-jp (format nil "~a は みつかりませんでした"
                              (send target-obj :speak-name))))
          (send target-obj :reset-coords)
          (send target-obj :transform tgt-cds)
          (return-from check-detection nil))
        (unix::usleep (* 50 1000))
        ) ;; while (ros::ok)
      (stop-subscribe)
      target-obj
      )))
