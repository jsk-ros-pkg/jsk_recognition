(load "package://eusmodel_sift_detector/euslisp/eusmodel_template_gen_utils.l")

;;
(load "package://euslib/rbrain/models/irt-all-objects.l")

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
  (gen-all-template objects :type "imagesift"))

;;
(make-sift-template-and-launch
 "launch/detection-eusmodels.launch"
 (list
;;(make-twinings)  (make-coffee-filter-pack)
  (instance twinings-object :init)
  (instance coffee-filter-object :init)
  ))

;;
(make-sift-template-and-launch
 "launch/panel-detection.launch"
 (list
;;(make-elevator-call-panel-body) (make-elevator-inside-panel-body)
  (instance elevator_call_panel-object :init)
  (instance elevator_inside_panel-object :init)
  ))
