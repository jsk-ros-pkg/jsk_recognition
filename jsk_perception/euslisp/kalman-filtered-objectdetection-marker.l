#!/usr/bin/env roseus

(load "package://euslisp/jskeus/irteus/kalmanlib.l")
(ros::load-ros-manifest "roseus")
(ros::load-ros-manifest "posedetection_msgs")
(ros::roseus "marker-publish")
(ros::roseus-add-msgs "posedetection_msgs")

(defvar *detection-topic* "/openni/rgb/ObjectDetection")
(defvar *base-frame-id* "/base_footprint")
(defvar *map-id* "/map")
(defvar *solve-tf* t)

(defvar map-frame-objectdetection (instance posedetection_msgs::ObjectDetection :init))
(send map-frame-objectdetection :header :frame_id *map-id*)

(defvar filtered-objectdetection (instance posedetection_msgs::ObjectDetection :init))
(send filtered-objectdetection :header :frame_id *map-id*)

(defparameter kfs-lst nil)

(defvar object-width nil)
(defvar object-height nil)
(defvar relative-pose-str nil)
(defclass kalman-filters
  :super propertied-object
  :slots (kfp kfr)
  )

(defmethod kalman-filters
  (:init
   (&key (pos (float-vector 0 0 0)) (yaw (float-vector 0)))
   (setq kfp (instance kalman-filter :init :state-dim 3 :measure-dim 3))
   (setq kfr (instance kalman-filter :init :state-dim 1 :measure-dim 1))
   (send kfp :x pos)
   (send kfr :x (float-vector yaw))
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
     (setq h1 (send kfp :proc z1))
     (setq h2 (send kfr :proc (float-vector z2)))
     (list h1 h2))
   )
  )

(defmethod coordinates
  (:stamp (&optional s) (if s (setf (get self :stamp) s)) (get self :stamp))
  (:frame_id (&optional f) (if f (setf (get self :frame_id) f)) (get self :frame_id)))

(defun detection-interface-objectdetection (force target-obj msg)
  (let ((obj-lst (send msg :objects))
        ;;(av (send *pr2* :angle-vector))
        (frame (send msg :header :frame_id))
        (tp (get target-obj :type))
        tmp-obj-lst stamp trans)
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
          (unless (send *tfl* :wait-for-transform *map-id* frame stamp 1)
            (ros::ros-error "could not solve ~A to ~A" *map-id* frame))
          (setq trans (send *tfl* :lookup-transform *map-id* frame stamp))
          (ros::ros-info "trans ~A" trans))
         (t (setq trans (make-coords))))

        (when trans
          (let ((tmp-obj (instance posedetection_msgs::Object6DPose :init)))
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
            (send tmp-obj :pose (ros::coords->tf-pose target-obj))
            (send tmp-obj :type (send obj :type))
            (setq tmp-obj-lst (append tmp-obj-lst (list tmp-obj)))
            (when (find-method target-obj :update-descendants)
              (send target-obj :update-descendants) ;; clean me
              )
            (unless (boundp '*tfb*)
              (setq *tfb* (instance ros::transform-broadcaster :init)))
            (send *tfb* :send-transform (ros::tf-pose->coords (send obj :pose)) frame (send target-obj :name))
            ;; target-obj should be on *base-frame-id* coordinates
            ;; (send *pr2* :angle-vector av)
            ))))
    (send map-frame-objectdetection :objects tmp-obj-lst)
    (send map-frame-objectdetection :header :stamp (send msg :header :stamp))
    ))

(defun publish-marker (target-obj marker-life &key (publish-name "/object_detection_marker_array")
                                  (frame *base-frame-id*) (object-name map-frame-objectdetection))
  (let ((mf-obj-lst (send object-name :objects))
        (tmp-tgtobj (make-cube 60 60 60))
        (tp (get target-obj :type)))
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (send tmp-tgtobj :move-to (ros::tf-pose->coords (send obj :pose)) :local)
        (ros::publish publish-name
                      (instance visualization_msgs::MarkerArray :init
                                :markers
                                (list (text->marker-msg
                                       (send obj :type)
                                       (send (send (ros::tf-pose->coords (send obj :pose)) :copy-worldcoords) :translate #f(-100 0 100))
                                       (send object-name :header)
                                       :ns (format nil "object_text_~A" (send obj :type)) :color #f(1 0 0)
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


(defun kalman-filter
  nil
  (let ((mf-obj-lst (send map-frame-objectdetection :objects))
        (flag t)
        )
    (dolist (obj mf-obj-lst)
      (dolist (kf kfs-lst)
        (cond
         ((equal (get kf :type) (send obj :type))
          (let ((ps (send (ros::tf-pose->coords (send obj :pose)) :pos))
                (yaw (elt (elt (rpy-angle (send (ros::tf-pose->coords (send obj :pose)) :rot)) 0) 0))
                z^
                )
            (setq flag nil)
            (setq z^ (send kf :proc ps yaw))
            (dolist (fo (send filtered-objectdetection :objects))
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
              (setq kfs-lst (append kfs-lst (list tmp-kfs)))
              (send filtered-objectdetection :objects (append (send filtered-objectdetection :objects) (list obj)))
              ))))
    (send filtered-objectdetection :header (send map-frame-objectdetection :header))
    ))

;; (defun calc-distribution (x sigma xa)
;;   (let* ((x-xa (v- x xa))
;;          (tt (* -0.5 (v. x-xa (transform (inverse-matrix sigma) x-xa))))
;;          (det (abs (- (* (aref sigma 0 0) (aref sigma 1 1)) (* (aref sigma 0 1) (aref sigma 1 0)))))
;;          (p (/ (exp tt) (* (* 2 pi) (expt det 0.5))))
;;          )
;;     ;; (ros::ros-info "inverse:~A exptt:~A exptdet:~A x-xa:~A tt:~A det:~A p:~A" (inverse-matrix sigma) (exp tt) (expt det 0.5)  x-xa tt det p)
;;     ;; (ros::ros-info "x-xa:~A tt:~A det:~A p:~A" x-xa tt det p)
;;     p
;;     )
;;   )

;; (defun calc-probability (x sigma xa)
;;   (let ((N 10)
;;         (S 0)
;;         (tmpx 0)
;;         (tmpy 0)
;;         )
;;     (dotimes (i N nil)
;;       (setf tmpx (- (+ (elt x 0) (/ (* 1.0 i) N)) 0.5))
;;       (dotimes (j N nil)
;;         (setf tmpy (- (+ (elt x 1) (/ (* 1.0 j) N)) 0.5))
;;         (setf S (+ S (/ (/ (calc-distribution (float-vector tmpx tmpy) sigma xa) (* N 1.0)) N)))
;;         ;;        (ros::ros-info "x:~A y:~A calcdist:~A" tmpx tmpy (calc-distribution (float-vector tmpx tmpy) sigma xa))
;;         )
;;       )
;;     S
;;     )
;;   )

;; (defun publish-2Dpointcloud (kfp)
;;   (let* ((P (cdr (assoc 'p_k (send kfp :slots))))
;;          (sigma (make-matrix 2 2))
;;          (xa (float-vector (elt (cdr (assoc 'x_k-1 (send kfp :slots))) 0) (elt (cdr (assoc 'x_k-1 (send kfp :slots))) 1)))
;;          )
;;     (ros::ros-info "P:~A xa:~A" P xa)
;;     (setf (aref sigma 0 0) (aref P 0 0))
;;     (setf (aref sigma 0 1) (aref P 0 1))
;;     (setf (aref sigma 1 0) (aref P 1 0))
;;     (setf (aref sigma 1 1) (aref P 1 1))
;;     (ros::ros-info "sigma: ~A ~A ~A ~A" (aref sigma 0 0) (aref sigma 0 1) (aref sigma 1 0) (aref sigma 1 1))
;;     (dotimes (y 3 nil)
;;       (let ((a nil))
;;         (dotimes (x 3 nil)
;;           (let ((xy (float-vector (+ (elt xa 0) (- x 1)) (+ (elt xa 1) (- y 1)))))
;;             (setq a (append a (list (calc-probability xy sigma xa))))
;;             )
;;           )
;;         (ros::ros-info "~A ~A ~A" (elt a 0) (elt a 1) (elt a 2))
;;         )
;;       )
;;     )
;;   )

(defun publish-attention (target-obj &key (object-name map-frame-objectdetection))
  (let ((mf-obj-lst (send object-name :objects))
        (tmp-posestamped (instance geometry_msgs::PoseStamped :init))
        (relative-pose-stream (make-string-input-stream relative-pose-str))
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
                              :translate (float-vector (* (/ object-height 2.0) 1000) (* (/ object-width 2.0) 1000) 0)))
    (dolist (obj mf-obj-lst)
      (when (or (not (stringp tp))
                (substringp tp (send obj :type))) ;; found target
        (let ((tmp-coords (send (ros::tf-pose->coords (send obj :pose)) :copy-worldcoords)))
          (send tmp-coords :transform inverse-trans)
          (send tmp-posestamped :pose (ros::coords->tf-pose tmp-coords))
          ;;(send tmp-posestamped :pose (send obj :pose))
          (send tmp-posestamped :header (send object-name :header))
          )))

    (ros::publish "/attention_clipper/input/pose" tmp-posestamped)
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
    (ros::publish "/attention_clipper/input/pose" tmp-pose-stamped)
    ))


(defun pub-cb (target-obj marker-life force msg)
  (let ((filter-obj))
    (detection-interface-objectdetection force target-obj msg)
    (cond
     ((send target-obj :stamp);;for tf error
      (kalman-filter)
      (publish-marker target-obj marker-life :frame *map-id*)
      (publish-marker target-obj marker-life :frame *map-id* :object-name filtered-objectdetection
                      :publish-name "/filtered")
      (publish-attention target-obj)
      ;;       (publish-2Dpointcloud kfp)
      ;; (ros::ros-info "cur-coords:(~A ~A ~A)"
      ;;                (send (send (send cur-coords :pose) :position) :x)
      ;;                (send (send (send cur-coords :pose) :position) :y)
      ;;                (send (send (send cur-coords :pose) :position) :z))
      )
     (t
      nil)
     )
    )
  )

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
     (ts
      (send cds :move-to ts :local)
      (let ((mf-obj-lst (send map-frame-objectdetection :objects))
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
      nil))
    (ros::ros-info "cds:~A" (send cds :worldcoords))
    (send m :object_pose tmp-coords)
    (ros::ros-info "reliability:~A type:~A" (send (send m :object_pose) :reliability) (send (send m :object_pose) :type))
    m))


(defun only-perception (&key ((:type atype) nil) (tf-force nil)
                             (publish-objectdetection-marker t)
                             (marker-life 30)
                             (detection-topic *detection-topic*)
                             (look-at nil)
                             ((:target-object target-obj) (make-cube 60 60 60))
                             )
  (let ((tgt-cds (send target-obj :copy-worldcoords))
        )
    (send target-obj :reset-coords)
    (setf (get target-obj :type) atype)
    (ros::subscribe detection-topic
                    posedetection_msgs::ObjectDetection
                    #'pub-cb target-obj marker-life tf-force)
    (ros::rate 10)
    (ros::advertise "/object_detection_marker_array" visualization_msgs::MarkerArray 5)
    (ros::advertise "/filtered" visualization_msgs::MarkerArray 5)
    (ros::advertise "/norm/pose" std_msgs::Float32 1)
    (ros::advertise "/norm/rot" std_msgs::Float32 1)
    (ros::advertise "/attention_clipper/input/pose" geometry_msgs::PoseStamped 1)
    (ros::advertise-service "targetobj" posedetection_msgs::TargetObj #'target-obj-cb)
    (setq object-width (ros::get-param "~object_width" 0.2))
    (setq object-height (ros::get-param "~object_height" 0.2))
    (setq relative-pose-str (ros::get-param "~relative_pose" "0 0 0 0 0 0 1"))
    (ros::subscribe "/openni_c2/rgb/image_rect_color"
                    sensor_msgs::Image #'attention-init)
    (ros::spin)
    )
  )


(only-perception)


