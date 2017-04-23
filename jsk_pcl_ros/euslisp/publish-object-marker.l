#!/usr/bin/env roseus

(load "package://jsk_pcl_ros/euslisp/generate-object-model.l")


(ros::roseus-add-msgs "visualization_msgs")

(ros::roseus "publish_object_marker")


;; setup ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun setup
  (&key
   (object-shape nil)
   )
  ;; setup object
  (unless object-shape
    (setq object-shape (ros::get-param "~object_shape" "cube")))
  (generate-object-model object-shape)
  (setq *msg* nil)
  ;; setup ros
  (ros::rate 20)
  (ros::subscribe "/element_matching_pose_estimation/output_pose" geometry_msgs::PoseStamped
                  #'pose-estimation-output-pose-cb)
  (ros::advertise "estimated_object_marker" visualization_msgs::Marker 1)
  (ros::advertise "interactive_object_marker" visualization_msgs::Marker 1)
  (ros::advertise-service "/print_estimation_output" std_srvs::Empty #'print-estimation-output-cb)
  )


;; main ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun main
  ()
  (let* (marker-msg)
    (while t
      (setq marker-msg (get-interactive-object-marker-msg))
      (ros::publish "interactive_object_marker" marker-msg)
      ;; (setq marker-msg (get-estimated-object-marker-msg))
      ;; (ros::publish "estimated_object_marker" marker-msg)
      (ros::spin-once)
      (ros::sleep)
      ))
  )


;; calback ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun pose-estimation-output-pose-cb
  (msg)
  (setq *msg* msg)
  (let* ((header
          (send msg :header))
         (marker-msg
          (object->marker-msg *obj* header
                              :coords (ros::tf-pose-stamped->coords msg)
                              :color (float-vector 1 0 0)
                              :alpha 0.5
                              ))
         )
    (ros::publish "estimated_object_marker" marker-msg)
    ))

(defun print-estimation-output-cb
  (req)
  (when *msg*
    (let* ((coords (ros::tf-pose-stamped->coords *msg*))
           (pos (send coords :pos))
           (rpy (car (rpy-angle (send coords :rot))))
           (frame-id (send (send *msg* :header) :frame_id))
           )
      (warning-message 1 ";; print estimation output ;;;;;;;;;;;;;;;;;;;;;;;;;~%")
      (warning-message 1 "(setq saved-coords (make-coords :pos (float-vector ~a ~a ~a) :rpy (list ~a ~a ~a)))~%"
                       (elt pos 0) (elt pos 1) (elt pos 2) (elt rpy 0) (elt rpy 1) (elt rpy 2))
      (warning-message 1 "(setq frame-id \"~a\")~%" frame-id)
      ))
  (send req :response)
  )


;; get marker message ;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-estimated-object-marker-msg
  ()
  (let* ((header
          (instance std_msgs::header :init
                    :frame_id "track_result"
                    :stamp (ros::time-now)))
         (msg
          (object->marker-msg *obj* header
                              :coords (make-coords)
                              :color (float-vector 1 0 0)
                              :alpha 0.5
                              ))
         )
    msg))

(defun get-interactive-object-marker-msg
  ()
  (let* ((header
          (instance std_msgs::header :init
                    :frame_id "interactive_object_marker"
                    :stamp (ros::time-now)))
         (msg
          (object->marker-msg *obj* header
                              :coords (make-coords)
                              :color (float-vector 0 1 0)
                              :alpha 0.5
                              ))
         )
    msg))


(setup)
(main)
