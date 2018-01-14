;;
;;
;;
;;(unless (ros::ok) (ros::roseus "detection_interface"))

(ros::ros-info "Publish for /objectdetection_marker_array")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "posedetection_msgs")
(ros::roseus-add-msgs "posedetection_msgs")

(defvar *detection-topic* "/openni/rgb/ObjectDetection")
(defvar *base-frame-id* "/base_footprint")
(defvar *solve-tf* t)

(ros::roseus "detection-interface")

(defmethod coordinates
  (:stamp (&optional s) (if s (setf (get self :stamp) s)) (get self :stamp))
  (:frame_id (&optional f) (if f (setf (get self :frame_id) f)) (get self :frame_id))
  (:speak-name
   (&optional s)
   (if s (setf (get self :speak-name) s)
     (let ((nm (get self :speak-name)))
       (if nm nm (send self :name))))))

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


(defun check-detection (&key (timeout 10) ((:type atype) nil) (tf-force nil)
                             (diff-position 10.0)
                             (diff-rotation 1) ;; radian
                             (single nil) (speak t)
                             (publish-objectdetection-marker t)
                             (marker-life 30) (speak-name)
                             (detection-topic *detection-topic*)
                             ((:target-object target-obj) (make-cube 60 60 60)))
  (let ((tgt-cds (send target-obj :copy-worldcoords))
        (tgt-name (send target-obj :name)))
    (send target-obj :reset-coords)
    (setf (get target-obj :type) atype)
    (when speak-name
      (send target-obj :speak-name speak-name))
    (when speak
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp (format nil "I am looking for ~a." (send target-obj :speak-name))))
           (t (speak-jp (format nil "~aをさがしています" (send target-obj :speak-name)))))
        (speak-jp (format nil "~a を さがして います" (send target-obj :speak-name)))))
    (start-detection-interface-subscribe tf-force target-obj detection-topic)
    (send *ri* :ros-wait 0.0 :spin-self t
          :func-before-throw #'(lambda (sl) (stop-detection-interface-subscribe detection-topic))) ;; attention-check ...
    (send target-obj :stamp (ros::time-now))
    (let* (current-object-coords
           previous-object-coords
           (start-time (ros::time-now)) (detect-object-list))
      (ros::ros-info "check (timeout=~A, type=~A, tf-force=~A)" timeout atype tf-force)
      (while (ros::ok)
        (send *ri* :ros-wait 0.0 :spin t
              :func-before-throw #'(lambda (sl) (stop-detection-interface-subscribe detection-topic))) ;; attention-check ...  ;; target-obj was updated in objectdetection
        ;; check target-obj is previously detected
        (when (and (> (send (send target-obj :stamp) :to-sec)
                      (send start-time :to-sec))
                   ;; discard object detection result before check function called
                   (not (member (send target-obj :stamp) detect-object-list))
                   ;; check if obj is already stored in detect-object-list
                   )
          (push (send target-obj :stamp) detect-object-list)
          (setq current-object-coords (send target-obj :copy-worldcoords)) ;; updated position
          (ros::ros-info "current -> ~A (~A)" current-object-coords (send target-obj :name))
          (when publish-objectdetection-marker
            (ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)
            (publish-marker target-obj marker-life)
            )
          (ros::ros-info "check detected ~A diff-pos:~A, diff-rot:~A"
                         (send target-obj :name)
                         (if previous-object-coords
                             (norm (send previous-object-coords
                                         :difference-position current-object-coords)))
                         (if previous-object-coords
                             (norm (send previous-object-coords
                                         :difference-rotation current-object-coords))))
          (when (or single
                    (and previous-object-coords
                         (and diff-position
                              (< (norm (send previous-object-coords
                                             :difference-position current-object-coords))
                                 diff-position)) ;; moved a little -> found obj
                         (and diff-rotation
                              (< (norm (send previous-object-coords
                                             :difference-rotation current-object-coords))
                                 diff-rotation)) ;; moved a little -> found obj
                         ))
            (send *ri* :ros-wait 0.0 :func-before-throw
                  #'(lambda (sl) (stop-detection-interface-subscribe detection-topic))) ;; attention-check ...
            (cond
             ((or (not (stringp atype))
                  (substringp atype (send target-obj :name))) ;; found target
              (ros::ros-info (format nil "ok - obj-type(~a)"  target-obj))
              (if (and (boundp '*irtviewer*) x::*display* (/= x::*display* 0))
                  (send *irtviewer* :draw-objects))
              (stop-detection-interface-subscribe detection-topic)
              (when speak
                (if (and (boundp '*use-voicetext*) *use-voicetext*)
                    (cond
                     ((and (boundp '*use-english*) *use-english*)
                      (speak-jp (format nil "I have found ~a." (send target-obj :speak-name))))
                     (t (speak-jp (format nil "~aが見つかりました。" (send target-obj :speak-name)))))
                  (speak-jp (format nil "~a が みつかりました" (send target-obj :speak-name)))))
              (send target-obj :name tgt-name)
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
          (stop-detection-interface-subscribe detection-topic)
          (when speak
            (if (and (boundp '*use-voicetext*) *use-voicetext*)
                (cond
                 ((and (boundp '*use-english*) *use-english*)
                  (speak-jp (format nil "I could not find ~a." (send target-obj :speak-name))))
                 (t (speak-jp (format nil "~aは見つかりませんでした。" (send target-obj :speak-name)))))
              (speak-jp (format nil "~a は みつかりませんでした" (send target-obj :speak-name)))))
          (send target-obj :reset-coords)
          (send target-obj :transform tgt-cds)
          (send *ri* :ros-wait 0.0) ;; attention-check ...
          (send target-obj :name tgt-name)
          (return-from check-detection nil))
        (send *ri* :ros-wait 0.05 :spin-self t
              :func-before-throw #'(lambda (sl) (stop-detection-interface-subscribe detection-topic))) ;; attention-check ...
        ) ;; while (ros::ok)
      (stop-detection-interface-subscribe detection-topic)
      (send *ri* :ros-wait 0.05 :spin-self t) ;; attention-check ...
      target-obj
      )))
;;
;; check program, publish objectdetection_msgs as marker_array
;;
(defun detection-checker (msg)
  (pprint 'subscribe)
  (let ((objs (send msg :objects))
        (hdr (send msg :header))
        (cntr 0))
    (dolist (obj objs)
      (let ((objtype (send obj :type))
            (cds (ros::tf-pose->coords (send obj :pose))))
        (ros::publish "/object_detection_marker_array"
                      (instance visualization_msgs::MarkerArray :init
                                :markers
                                (list
                                 (text->marker-msg
                                  objtype
                                  (send (send cds :copy-worldcoords) :translate #f(100 -100 0))
                                  hdr
                                  :ns (format nil "object_text_~A" objtype)
                                  :color #f(1 0 0)
                                  :scale 100 :lifetime 0 :id 0)
                                 (coords->marker-msg
                                  cds hdr
                                  :ns (format nil "object_coords_~A" objtype)
                                  ;; :size 200 :width 20
                                  :lifetime 0 :id 1)
                                 )))
        (incf cntr)
        ))
    ))
(defun detection-marker-test ()
  (ros::roseus "detection_marker_test")
  (ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 100)
  (ros::subscribe *detection-topic*
                  posedetection_msgs::ObjectDetection
                  #'detection-checker)
  (ros::rate 100)
  (do-until-key
   (ros::spin-once)
   (ros::sleep))
  )

;;(load "package://jsk_perception/euslisp/detection_interface_servicecall.l")