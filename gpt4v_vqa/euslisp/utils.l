(ros::load-ros-manifest "jsk_recognition_msgs")

(defparameter *vqa-action* nil)

(defun init-vqa-action (&key (timeout 10))
  (unless *vqa-action*
    (setq *vqa-action*
          (instance ros::simple-action-client :init
                    "/vqa/inference_server" jsk_recognition_msgs::VQATaskAction)))
  (send *vqa-action* :wait-for-server timeout)
  )

(defun run-vqa (question &optional msg_image &key (timeout 30))
  "run vqa action client. return answer string or nil if failed."
  (if (not (init-vqa-action))
    (return-from run-vqa nil))
  (let* (result answer
         (action-goal (instance jsk_recognition_msgs::VQATaskGoal :init)))
    (send action-goal :questions (list question))
    (if msg_image
        (send action-goal :image msg_image)
    )
    (send *vqa-action* :send-goal action-goal)
    (unless (send *vqa-action* :wait-for-result :timeout timeout)
      (send *vqa-action* :cancel-all-goals)
      (ros::ros-error "No result returned from /vqa action server")
      (return-from run-vqa nil))
    (setq result (send *vqa-action* :get-result))
    (if (and result (> (length (send result :result :result)) 0))
        (send (elt (send result :result :result) 0) :answer)
        nil)
    )
  )