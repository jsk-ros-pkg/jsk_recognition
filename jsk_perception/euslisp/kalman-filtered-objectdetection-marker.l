#!/usr/bin/env roseus

(load "irteus/kalmanlib.l")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "posedetection_msgs")
(ros::roseus "marker-publish")
(ros::roseus-add-msgs "posedetection_msgs")

(setq *map-id* "/map")
(setq *solve-tf* t)
(setq *object-width* (ros::get-param "~object_width" 0))
(setq *object-height* (ros::get-param "~object_height" 0))
(setq *relative-pose-str* (ros::get-param "~relative_pose" "0 0 0 0 0 0 1"))
(setq *detection-topic* (ros::get-param "~detection_topic" "/kinect_head/rgb/ObjectDetection"))
(setq *marker-life* (ros::get-param "~marker_life" 300))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "/base_footprint"))
(setq *calc-kalman* t)
(setq *detect-flag* t)
(setq *tf-force* nil)
(setq *target-obj* (make-cube 60 60 60))
(send *target-obj* :reset-coords)
(setf (get *target-obj* :type) (ros::get-param "~target_type" nil))
(setq *map-frame-objectdetection* (instance posedetection_msgs::ObjectDetection :init))
(send *map-frame-objectdetection* :header :frame_id *map-id*)
(setq *filtered-objectdetection* (instance posedetection_msgs::ObjectDetection :init))
(send *filtered-objectdetection* :header :frame_id *map-id*)
(setq *kfs-lst* nil)
(setq *list-of-target-x-pos* nil)
(setq *threshold-of-target-x-pos-diff* 30) ; mm
(setq *average-of-target-x-pos* 0)

(defclass kalman-filters
  :super propertied-object
  :slots (kfp kfr)
  )

(defmethod kalman-filters
  (:init
   (&key (pos (float-vector 0 0 0)) (yaw (float-vector 0)))
   (setq kfp (instance kalman-filter :init :state-dim 3 :measure-dim 3 :q-variance 1))
   (send kfp :x (float-vector (elt pos 0) (elt pos 1) (elt pos 2)))
   (setq kfr (instance kalman-filter :init :state-dim 2 :measure-dim 1 :q-variance 0.1))
   (send kfr :x (float-vector yaw 0))
   ;; (send kfp :x pos)
   ;; (send kfp :A (make-array (list 3 3)))
   ;; (send kfp :A #2f((1 0 0 1 0 0) (0 1 0 0 1 0) (0 0 1 0 0 1) (0 0 0 1 0 0) (0 0 0 0 1 0) (0 0 0 0 0 1)))
   ;; (send kfr :A (make-array (list 2 2)))
   ;; (send kfr :A #2f((1 1) (0 1)))
   ;; (setq kfr (instance kalman-filter :init :state-dim 1 :measure-dim 1))
   ;; (send kfr :x (float-vector yaw))
   )
  (:pos
   (p)
   (send kfp :x p)
   )
  (:yaw
   (y)
   (send kfr :x y)
   )
  (:proc
   (z1 z2)
   (let (h1 h2)
     (setq h1 (send kfp :proc z1 :adaptive t :m 3))
     (setq h2 (send kfr :proc (float-vector z2) :adaptive t :m 30))
     (list h1 h2)
     ))
  (:P
   ()
   (send kfp :P)
   ))

(defmethod coordinates
  (:stamp (&optional s) (if s (setf (get self :stamp) s)) (get self :stamp))
  (:frame_id (&optional f) (if f (setf (get self :frame_id) f)) (get self :frame_id)))

(defun remap-to-map-frame-and-store-ObjectDetection (target-obj msg)
  (let ((obj-lst (send msg :objects))
        (frame (send msg :header :frame_id))
        (tp (get target-obj :type))
        tmp-obj-lst stamp trans)
    (ros::ros-debug "object detection callback target ~A(~A), frame ~A, object ~A"
                    tp (stringp tp) frame target-obj)
    (dolist (obj obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (setq stamp (if *tf-force* (ros::time 0) (send msg :header :stamp)))
        (cond
         (*solve-tf*
          (unless (boundp '*tfl*)
            (setq *tfl* (instance ros::transform-listener :init)))
          (unless (send *tfl* :wait-for-transform *map-id* frame stamp 1)
            (ros::ros-error "could not solve ~A to ~A" *map-id* frame))
          (setq trans (send *tfl* :lookup-transform *map-id* frame stamp))
          (ros::ros-debug "trans ~A" trans))
         (t (setq trans (make-coords))))
        (when trans
          (let ((tmp-obj (instance posedetection_msgs::Object6DPose :init)))
            (ros::ros-info "object detected ~A ~A ~20,9f" (send obj :type)
                           (ros::tf-pose->coords (send obj :pose))
                           (send (send msg :header :stamp) :to-sec))
            (send target-obj :name (send obj :type))
            (send target-obj :stamp (send msg :header :stamp))
            (send target-obj :frame_id (send msg :header :frame_id))
            (send target-obj :reset-coords)
            (send target-obj :move-to trans :local)
            (send target-obj :move-to (ros::tf-pose->coords (send obj :pose)) :local)
            (send tmp-obj :pose (ros::coords->tf-pose target-obj))
            (send tmp-obj :type (send obj :type))
            (setq tmp-obj-lst (append tmp-obj-lst (list tmp-obj)))
            (when (find-method target-obj :update-descendants)
              (send target-obj :update-descendants) ;; clean me
              )
            (unless (boundp '*tfb*)
              (setq *tfb* (instance ros::transform-broadcaster :init)))
            (send *tfb* :send-transform (ros::tf-pose->coords (send obj :pose)) frame (send target-obj :name))
            ))))
    (send *map-frame-objectdetection* :objects tmp-obj-lst)
    (send *map-frame-objectdetection* :header :stamp (send msg :header :stamp))
    ))

(defun publish-marker (target-obj marker-life &key (publish-name "object_detection_marker_array")
                                  (object-name *map-frame-objectdetection*)
                                  (text-name nil) (text-color nil) (obj-color nil))
  (let ((mf-obj-lst (send object-name :objects))
        (tmp-tgtobj (make-cube 60 60 60))
        (tp (get target-obj :type)))
    (when obj-color
      (send tmp-tgtobj :set-color obj-color))
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (send tmp-tgtobj :move-to (ros::tf-pose->coords (send obj :pose)) :local)
        (ros::publish publish-name
                      (instance visualization_msgs::MarkerArray :init
                                :markers
                                (list (text->marker-msg
                                       (cond
                                        (text-name
                                         text-name)
                                        (t
                                         (send obj :type)
                                         ))
                                       (send (send (ros::tf-pose->coords (send obj :pose)) :copy-worldcoords) :translate #f(-100 0 100))
                                       (send object-name :header)
                                       :ns (format nil "object_text_~A" (send obj :type))
                                       :color (cond
                                               (text-color
                                                text-color)
                                               (t
                                                #f(1 0 0)))
                                       :scale 100 :lifetime marker-life :id 0)
                                      (coords->marker-msg
                                       (send (ros::tf-pose->coords (send obj :pose)) :copy-worldcoords)
                                       (send object-name :header)
                                       :ns (format nil "object_coords_~A" (send obj :type))
                                       ;; :size 200 :width 20
                                       :lifetime marker-life :id 1)
                                      (object->marker-msg
                                       tmp-tgtobj
                                       (send object-name :header)
                                       :ns (format nil "object_body_~A" (send obj :type))
                                       :lifetime marker-life :id 2)
                                      )))))))

(defun catch-outlier (target-obj)
  (let ((mf-obj-lst (send *map-frame-objectdetection* :objects))
        (tmp-tgtobj (make-cube 60 60 60))
        (marker-life 700)
        (outlierflag nil)
        (tp (get target-obj :type)))
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (send tmp-tgtobj :move-to (ros::tf-pose->coords (send obj :pose)) :local)
        (cond
         ((< (length *list-of-target-x-pos*) 20) ;;stock only
          (setq *list-of-target-x-pos* (append *list-of-target-x-pos* (list (aref (send tmp-tgtobj :pos) 0))))
          )
         (t
          (setq *average-of-target-x-pos* 0)
          (dolist (x *list-of-target-x-pos* nil)
            (setq *average-of-target-x-pos* (+ *average-of-target-x-pos* x))
            )
          (setq *average-of-target-x-pos* (/ *average-of-target-x-pos* 20))
          (ros::ros-info "ave:~A x: ~A diff:~A outlierflag:~A" *average-of-target-x-pos*
                         (aref (send tmp-tgtobj :pos) 0)
                         (abs (- *average-of-target-x-pos* (aref (send tmp-tgtobj :pos) 0)))
                         (< *threshold-of-target-x-pos-diff* (abs (- *average-of-target-x-pos* (aref (send tmp-tgtobj :pos) 0))))
                         )
          (cond
           ((< *threshold-of-target-x-pos-diff* (abs (- *average-of-target-x-pos* (aref (send tmp-tgtobj :pos) 0)))) ;; outsider
            (ros::ros-info "publish outlier")
            (setq outlierflag t)
            (setq *list-of-target-x-pos* (append (cdr *list-of-target-x-pos*) (list (aref (send tmp-tgtobj :pos) 0))))
            )
           (t ;; inside
            ;; like queue
            (setq *list-of-target-x-pos* (append (cdr *list-of-target-x-pos*) (list (aref (send tmp-tgtobj :pos) 0))))
            ))))))
    (when outlierflag
      (publish-marker target-obj *marker-life* :publish-name "outlier"
                      :object-name *map-frame-objectdetection*
                      :text-name "outlier" :text-color (float-vector 0 1 0)
                      :obj-color (float-vector 0.4 1 0.4))
      )
    (return-from catch-outlier outlierflag)
    ))

(defparameter *poselist* nil)
(defun add-to-xlist (x)
  (setq *poselist* (append *poselist* (list x)))
  (cond
   ((> (length *poselist*) 10)
    (setq *poselist* (cdr *poselist*))
    )
   (t
    nil)))

(defun calc-covariance ()
  (let ((x-cov 0)
        (y-cov 0)
        (z-cov 0)
        (x-ave 0)
        (y-ave 0)
        (z-ave 0))
    ;; (ros::ros-info "poselist:~A" *poselist*)
    (dolist (pose *poselist*)
      (setq x-ave (+ x-ave (send pose :position :x)))
      (setq y-ave (+ y-ave (send pose :position :y)))
      (setq z-ave (+ z-ave (send pose :position :z)))
      )
    (setq x-ave (/ (* 1.0 x-ave) (length *poselist*)))
    (setq y-ave (/ (* 1.0 y-ave) (length *poselist*)))
    (setq z-ave (/ (* 1.0 z-ave) (length *poselist*)))
    ;; (ros::ros-info "x-ave:~A" x-ave)
    (dolist (pose *poselist*)
      (setq x-cov (+ x-cov
                     (* (- (send pose :position :x) x-ave) (- (send pose :position :x) x-ave))))
      (setq y-cov (+ y-cov
                     (* (- (send pose :position :y) y-ave) (- (send pose :position :y) y-ave))))
      (setq z-cov (+ z-cov
                     (* (- (send pose :position :z) z-ave) (- (send pose :position :z) z-ave))))
      )
    (setq x-cov (/ (* 1.0 x-cov) (length *poselist*)))
    (setq y-cov (/ (* 1.0 y-cov) (length *poselist*)))
    (setq z-cov (/ (* 1.0 z-cov) (length *poselist*)))
    ;; too small
    (setq x-cov (* x-cov 100))
    (setq y-cov (* y-cov 100))
    (setq z-cov (* z-cov 100))
    ;; (ros::ros-info "x-cov:~A" x-cov)
    ;;    (return-from calc-covariance (list x-cov y-cov z-cov))
    (list x-cov y-cov z-cov)
    ))


(defun simply-calc-covariance-and-pub-PoseStamped-and-PoseWithCovarianceStamped
  (target-obj &key (object-name *map-frame-objectdetection*))
  (let* ((mf-obj-lst (send object-name :objects))
        (tmp-posestamped (instance geometry_msgs::PoseStamped :init))
        (tp (get target-obj :type))
        (relative-pose-stream (make-string-input-stream *relative-pose-str*))
        (tmp-pose (instance geometry_msgs::Pose :init))
        covlst
        (tmp-posewithcovariancestamped (instance geometry_msgs::PoseWithCovarianceStamped :init))
        (tmp-cov (send tmp-posewithcovariancestamped :pose :covariance))
        )
    ;; currently considered only 1 kind of data comes
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (add-to-xlist (send obj :pose))
        ))
    (setf covlst (calc-covariance))
    (send tmp-pose :position :x (read relative-pose-stream nil))
    (send tmp-pose :position :y (read relative-pose-stream nil))
    (send tmp-pose :position :z (read relative-pose-stream nil))
    (send tmp-pose :orientation :x (read relative-pose-stream nil))
    (send tmp-pose :orientation :y (read relative-pose-stream nil))
    (send tmp-pose :orientation :z (read relative-pose-stream nil))
    (send tmp-pose :orientation :w (read relative-pose-stream nil))
    (setq inverse-trans (send (send (ros::tf-pose->coords tmp-pose) :inverse-transformation)
                              :translate (float-vector (* (/ *object-height* 2.0) 1000) (* (/ *object-width* 2.0) 1000) 0)))
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (let ((tmp-coords (send (ros::tf-pose->coords (send obj :pose)) :copy-worldcoords)))
          (send tmp-coords :transform inverse-trans)
          (send tmp-posestamped :pose (ros::coords->tf-pose tmp-coords))
          (send tmp-posestamped :header (send object-name :header))
          )))

    (setf (elt tmp-cov 0) (car covlst))
    (setf (elt tmp-cov 7) (cadr covlst))
    (setf (elt tmp-cov 14) (caddr covlst))
    (unless (eq (car covlst) 0.0)
      (ros::ros-info "simplecov x:~A y:~A z:~A" (car covlst) (cadr covlst) (caddr covlst))
      (setf (elt tmp-cov 21) 0.1)
      (setf (elt tmp-cov 28) 0.1)
      (setf (elt tmp-cov 35) 0.1)
      (send tmp-posewithcovariancestamped :header (send object-name :header))
      (send tmp-posewithcovariancestamped :pose :pose (send tmp-posestamped :pose))
      (send tmp-posewithcovariancestamped :pose :covariance tmp-cov)
      (ros::publish "simplecov" tmp-posewithcovariancestamped)
      )
    ))

(defun apply-kalman-filter-to-mapframeobjectdetection ()
  (let ((mf-obj-lst (send *map-frame-objectdetection* :objects))
        (flag t)
        )
    (dolist (obj mf-obj-lst)
      (dolist (kf *kfs-lst*)
        (cond
         ((equal (get kf :type) (send obj :type))
          (let ((ps (send (ros::tf-pose->coords (send obj :pose)) :pos))
                (yaw (elt (elt (rpy-angle (send (ros::tf-pose->coords (send obj :pose)) :rot)) 0) 0))
                z^
                )
            (setq flag nil)
            (setq z^ (send kf :proc ps yaw))
            (dolist (fo (send *filtered-objectdetection* :objects))
              (cond
               ((equal (get kf :type) (send fo :type))
                (let ((tmp-coords (make-coords)))
                  (send tmp-coords :locate (elt z^ 0))
                  (send tmp-coords :rotate (elt (elt z^ 1) 0) :z)
                  (send fo :pose (ros::coords->tf-pose tmp-coords))))
               (t
                nil)))
            ))
         (t
          nil)))
      (if flag
          (progn
            (let ((tmp-kfs (instance kalman-filters :init
                                     :pos (send (ros::tf-pose->coords (send obj :pose)) :pos)
                                     :yaw (elt (elt (rpy-angle (send (ros::tf-pose->coords (send obj :pose)) :rot)) 0) 0)))
                  tmp-fo
                  )
              (setf (get tmp-kfs :type) (send obj :type))
              (setq *kfs-lst* (append *kfs-lst* (list tmp-kfs)))
              (send *filtered-objectdetection* :objects (append (send *filtered-objectdetection* :objects) (list obj)))
              ))))
    (send *filtered-objectdetection* :header (send *map-frame-objectdetection* :header))
    ))

(defun pub-PoseStamped-and-PoseWithCovarianceStamped (target-obj &key (object-name *map-frame-objectdetection*))
  (let ((mf-obj-lst (send object-name :objects))
        (tmp-posestamped (instance geometry_msgs::PoseStamped :init))
        (tmp-posewithcovariancestamped (instance geometry_msgs::PoseWithCovarianceStamped :init))
        (relative-pose-stream (make-string-input-stream *relative-pose-str*))
        (tmp-pose (instance geometry_msgs::Pose :init))
        (tp (get target-obj :type)))

    (ros::unsubscribe "/openni_c2/rgb/image_rect_color")

    (send tmp-pose :position :x (read relative-pose-stream nil))
    (send tmp-pose :position :y (read relative-pose-stream nil))
    (send tmp-pose :position :z (read relative-pose-stream nil))
    (send tmp-pose :orientation :x (read relative-pose-stream nil))
    (send tmp-pose :orientation :y (read relative-pose-stream nil))
    (send tmp-pose :orientation :z (read relative-pose-stream nil))
    (send tmp-pose :orientation :w (read relative-pose-stream nil))
    (print-ros-msg tmp-pose)
    (setq inverse-trans (send (send (ros::tf-pose->coords tmp-pose) :inverse-transformation)
                              :translate (float-vector (* (/ *object-height* 2.0) 1000) (* (/ *object-width* 2.0) 1000) 0)))
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (let ((tmp-coords (send (ros::tf-pose->coords (send obj :pose)) :copy-worldcoords)))
          (send tmp-coords :transform inverse-trans)
          (send tmp-posestamped :pose (ros::coords->tf-pose tmp-coords))
          (send tmp-posestamped :header (send object-name :header))
          )))

    (unless (null *kfs-lst*)
      (let ((cov (send (car *kfs-lst*) :P))
            (tmp-cov (send tmp-posewithcovariancestamped :pose :covariance)))
        (setq *pwc* tmp-cov)
        (setq *cov* cov)
        (setf (elt tmp-cov 0) (/ (elt (matrix-row cov 0) 0) 100.0))
        (setf (elt tmp-cov 1) (/ (elt (matrix-row cov 0) 1) 100.0))
        (setf (elt tmp-cov 2) (/ (elt (matrix-row cov 0) 2) 100.0))
        (setf (elt tmp-cov 6) (/ (elt (matrix-row cov 1) 0) 100.0))
        (setf (elt tmp-cov 7) (/ (elt (matrix-row cov 1) 1) 100.0))
        (setf (elt tmp-cov 8) (/ (elt (matrix-row cov 1) 2) 100.0))
        (setf (elt tmp-cov 12) (/ (elt (matrix-row cov 2) 0) 100.0))
        (setf (elt tmp-cov 13) (/ (elt (matrix-row cov 2) 1) 100.0))
        (setf (elt tmp-cov 14) (/ (elt (matrix-row cov 2) 2) 100.0))
        (setf (elt tmp-cov 21) 0.1)
        (setf (elt tmp-cov 28) 0.1)
        (setf (elt tmp-cov 35) 0.1)

        (ros::ros-info "estimated covariance of adaptive kalmanfilter:~A" cov)
        (ros::ros-info "poswithcovariance: ~A" tmp-cov)

        (send tmp-posewithcovariancestamped :header (send object-name :header))
        (send tmp-posewithcovariancestamped :pose :pose (send tmp-posestamped :pose))
        (send tmp-posewithcovariancestamped :pose :covariance tmp-cov)
        (ros::publish "posewithcovstamped" tmp-posewithcovariancestamped)
        ))
    (ros::publish "attention_clipper/input/pose" tmp-posestamped)
    ))

(defun attention-init (msg)
  (let ((tmp-pose (instance geometry_msgs::Pose :init))
        (tmp-pose-stamped (instance geometry_msgs::PoseStamped :init))
        (header (instance std_msgs::header :init))
        )
    (send tmp-pose :position :x 0)
    (send tmp-pose :position :y 0)
    (send tmp-pose :position :z 0.2)
    (send tmp-pose :orientation :x 0)
    (send tmp-pose :orientation :y 0)
    (send tmp-pose :orientation :z 0)
    (send tmp-pose :orientation :w 1)
    (send header :stamp (ros::time 0))
    (send header :frame_id "/openni_rgb_optical_frame")
    (send tmp-pose-stamped :pose tmp-pose)
    (send tmp-pose-stamped :header header)
    (ros::publish "attention_clipper/input/pose" tmp-pose-stamped)
    ))

(defun pub-cb (target-obj msg)
  (let ()
    ;; remap to map-frame and store it to *mapframe-objectdetection*
    (remap-to-map-frame-and-store-ObjectDetection target-obj msg)
    (when *calc-kalman*
      (cond
       ((send target-obj :stamp);;for avoiding tf error
        (publish-marker target-obj *marker-life* :text-name "raw"
                        :obj-color (float-vector 1 0.4 0.4))
        (simply-calc-covariance-and-pub-PoseStamped-and-PoseWithCovarianceStamped target-obj)
        (when (null (catch-outlier target-obj)) ;; avoiding divergence by outlier
          (apply-kalman-filter-to-mapframeobjectdetection)
          (publish-marker target-obj *marker-life* :object-name *filtered-objectdetection*
                          :publish-name "filtered" :obj-color (float-vector 0.4 0.4 1)
                          :text-color (float-vector 0 0 1) :text-name "filtered")
          (pub-PoseStamped-and-PoseWithCovarianceStamped target-obj)
          ))
       (t
        nil)
       ))))

(defun target-obj-cb (req)
  (let ((m (send req :response))
        (cds (make-coords))
        (tmp-coords (instance posedetection_msgs::Object6DPose :init))
        ts)
    (send tmp-coords :pose :orientation :w 1)
    (send tmp-coords :reliability 0.5)

    (unless (boundp '*tfl*)
      (setq *tfl* (instance ros::transform-listener :init)))
    (unless (send *tfl* :wait-for-transform *base-frame-id* *map-id* (ros::time 0) 1)
      (ros::ros-error "could not solve ~A to ~A" *base-frame-id* *map-id*))
    (setq ts (send *tfl* :lookup-transform *base-frame-id* *map-id* (ros::time 0)))
    (ros::ros-info "ts ~A" ts)
    (send cds :reset-coords)
    (cond
     ((and ts (send *map-frame-objectdetection* :objects))
      (send cds :move-to ts :local)
      (Let ((mf-obj-lst (send *map-frame-objectdetection* :objects))
            (tp (send req :type)))
        (dolist (obj mf-obj-lst)
          (when (or (not (stringp tp))
                    (substringp tp (send obj :type))) ;; found target
            (send cds :move-to (ros::tf-pose->coords (send obj :pose)) :local)
            (send tmp-coords :pose (ros::coords->tf-pose (send cds :copy-worldcoords)))
            (send tmp-coords :type (send obj :type))
            (ros::ros-info "in cb tmp-coords:(~A ~A ~A)"
                           (send (send (send tmp-coords :pose) :position) :x)
                           (send (send (send tmp-coords :pose) :position) :y)
                           (send (send (send tmp-coords :pose) :position) :z))))))
     (t
      (send tmp-coords :reliability 0)
      nil))
    (ros::ros-info "cds:~A" (send cds :worldcoords))
    (send m :object_pose tmp-coords)
    (ros::ros-info "reliability:~A type:~A" (send (send m :object_pose) :reliability) (send (send m :object_pose) :type))
    m))

(defun detect-switch-cb (msg)
  (ros::ros-debug "detect-switch data:~A" (send msg :data))
  (setq *detect-flag* (send msg :data))
  (when *detect-flag*
    ;; reflesh
    (setq *map-frame-objectdetection* (instance posedetection_msgs::ObjectDetection :init))
    (send *map-frame-objectdetection* :header :frame_id *map-id*) ;; avoiding header error in rviz
    ))

(ros::advertise "object_detection_marker_array" visualization_msgs::MarkerArray 5)
(ros::advertise "filtered" visualization_msgs::MarkerArray 5)
(ros::advertise "outlier" visualization_msgs::MarkerArray 5)
(ros::advertise "attention_clipper/input/pose" geometry_msgs::PoseStamped 1)
(ros::advertise "posewithcovstamped" geometry_msgs::PoseWithCovarianceStamped 1)
(ros::advertise "simplecov" geometry_msgs::PoseWithCovarianceStamped 1)
(ros::advertise-service "targetobj" posedetection_msgs::TargetObj #'target-obj-cb)
(ros::subscribe "input/image" sensor_msgs::Image #'attention-init)
(ros::subscribe "input/switch" std_msgs::Bool #'detect-switch-cb)

(ros::rate 10)
(while t
  (ros::spin-once)
  (cond
   (*detect-flag*
    (ros::subscribe "input/ObjectDetection"
                    posedetection_msgs::ObjectDetection
                    #'pub-cb *target-obj*))
   (t
    (ros::unsubscribe detection-topic)))
  (ros::sleep))
