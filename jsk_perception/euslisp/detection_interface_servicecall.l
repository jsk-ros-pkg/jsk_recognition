(defun check-detection (&key (timeout 10) ((:type atype) nil) (tf-force nil)
                             (single nil) (speak t)
                             (speak-name)
                             (detection-topic *detection-topic*)
                             ((:target-object target-obj) (make-cube 60 60 60)))
  (let ((tgt-cds (send target-obj :copy-worldcoords)))
    (setf (get target-obj :type) atype)
    (ros::wait-for-service "targetobj")
    (when speak-name
      (send target-obj :speak-name speak-name))
    (when speak
      (if (and (boundp '*use-voicetext*) *use-voicetext*)
          (cond
           ((and (boundp '*use-english*) *use-english*)
            (speak-jp (format nil "I am looking for ~a." (send target-obj :speak-name))))
           (t (speak-jp (format nil "~aをさがしています" (send target-obj :speak-name)))))
        (speak-jp (format nil "~a を さがして います" (send target-obj :speak-name)))))
    (let* ((start-time (ros::time-now))
           (req (instance posedetection_msgs::TargetObjRequest :init))
           res)
      (ros::ros-info "check (timeout=~A, type=~A, tf-force=~A)" timeout atype tf-force)
      (while (ros::ok)
        ;;servicecall
        (send req :type atype)
        (setq res (ros::service-call "targetobj" req))
        (send target-obj :reset-coords)
        (send target-obj :move-to (ros::tf-pose->coords (send (send res :object_pose) :pose)))
        (send target-obj :name (send (send res :object_pose) :type))

        (ros::ros-info "current -> ~A (~A)" (send target-obj :copy-worldcoords) (send target-obj :name))
        (ros::ros-info "check detected ~A percent:~A"
                       (send target-obj :name)
                       (send (send res :object_pose) :reliability))
        (when (or single
                  (> (send (send res :object_pose) :reliability) 0))
          (cond
           ((or (not (stringp atype))
                (substringp atype (send target-obj :name))) ;; found target
            (ros::ros-info (format nil "ok - obj-type(~a)"  target-obj))
            (if (and (boundp '*irtviewer*) x::*display* (/= x::*display* 0))
                (send *irtviewer* :draw-objects))
            (when speak
              (if (and (boundp '*use-voicetext*) *use-voicetext*)
                  (cond
                   ((and (boundp '*use-english*) *use-english*)
                    (speak-jp (format nil "I have found ~a." (send target-obj :speak-name))))
                   (t (speak-jp (format nil "~aが見つかりました。" (send target-obj :speak-name)))))
                (speak-jp (format nil "~a が みつかりました" (send target-obj :speak-name)))))
            (return-from check-detection target-obj))
           (t
            (ros::ros-info (format nil "no - obj-type(~a)" (send target-obj :name)))
            ))
          ) ;; when (and reliability...
        (when (and (boundp '*irtviewer*) x::*display* (/= x::*display* 0))
          (send *irtviewer* :draw-objects)
          (x::window-main-one))
        (when (< timeout
                 (send (ros::time- (ros::time-now) start-time) :to-sec))
          (ros::ros-info "timeout")
          (when speak
            (if (and (boundp '*use-voicetext*) *use-voicetext*)
                (cond
                 ((and (boundp '*use-english*) *use-english*)
                  (speak-jp (format nil "I could not find ~a." (send target-obj :speak-name))))
                 (t (speak-jp (format nil "~aは見つかりませんでした。" (send target-obj :speak-name)))))
              (speak-jp (format nil "~a は みつかりませんでした" (send target-obj :speak-name)))))
          (send target-obj :reset-coords)
          (send target-obj :transform tgt-cds)
          (return-from check-detection nil))
        ) ;; while (ros::ok)
      target-obj
      )))

;; (do-until-key
;;  (ros::ros-info "time:~A" (ros::time-now))
;;  (check-detection :type "fridge" :speak nil)
;;  (unix:usleep 1000000))
