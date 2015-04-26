(load "package://jsk_perception/euslisp/eusmodel_template_gen_utils.l")

;;
(load "models/irt-all-objects.l")

;; detection-object needs :name as frame_id

(defun make-sift-template-and-launch (launchfilename objects)
  ;; jsk->irt converted models are "name-object" class
  (dolist (obj objects)
    (when (null (send obj :name))
      (let ((class-name (string-downcase (send (class obj) :name))))
	(send obj :name (subseq class-name 0 (- (length class-name) (length "-object"))))
	)))
  ;; generate templates and launchfiles
  (gen-all-launch objects launchfilename)
  )

;;
;;
;;
(make-sift-template-and-launch
 "launch/eusmodel_detection_elevator-panels-eng8.launch"
 (list
  (instance elevator_call_panel-object :init)
  (instance elevator_inside_panel-object :init)
  ))

(make-sift-template-and-launch
 "launch/eusmodel_detection_elevator-panels-eng2.launch"
 (list
  (instance elevator_call_panel_eng2-object :init
	    :name "elevator_call_panel")
  (instance elevator_inside_panel_eng2-object :init
	    :name "elevator_inside_panel")
  ))


(exit)
