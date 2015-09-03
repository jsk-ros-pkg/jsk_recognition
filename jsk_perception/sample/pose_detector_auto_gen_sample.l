#!/usr/bin/env roseus
;;
;; Sample of Adding Object / point_pose_extractor
;;
;;   subscribe topic -> /narrow_stereo/left/ImageFeature0D
;;   publish topic   -> /narrow_stereo/left/ObjectDetection
;;

;; < 1 >
;; . make irteus models with texture
;; . object should have unique name
(defun make-sony-rimokon ()
  (let ((obj (make-cube 45 190 25)))
    (setq aface (elt (send obj :faces) 1))
    (send obj :paste-texture-to-face aface
	  :file "sony-tv-controller.png")
    (send obj :name "sony_tv_controller")
    obj))

(defun make-sharp-rimokon ()
  (let ((obj (make-cube 48 200 25)))
    (setq aface (elt (send obj :faces) 1))
    (send obj :paste-texture-to-face aface
	  :file "sharp-tv-controller-ut.png")
    (send obj :name "sharp_tv_controller")
    obj))

(defun make-lipton-tea-box ()
  (let ((obj (make-cube 41 62 106)))
    (setq aface (send obj :face 3))
    (send obj :paste-texture-to-face aface
	  :file "lipton-milktea-box.png")
    (send obj :name "lipton_milktea_box")
    obj))

;; < 2 >
;; . generate launch file
(defun generate-launcher ()
  (load "package://jsk_perception/euslisp/eusmodel_template_gen_utils.l")
  (gen-all-launch (list (make-sony-rimokon) (make-sharp-rimokon))
		  "rimokon-pose.launch" :ns "/narrow_stereo/left" )
  (gen-all-launch (list (make-lipton-tea-box))
		  "milktea-box.launch" :ns "/narrow_stereo/left"))
(warn ";; (generate-launcher)~%")

;; < 3 >
;; . launch the generated file

;; < 4 >
;; . subscribe ObjectDetection topic
(defun test-detection-result ()
  (ros::roseus "test")
  (load "package://pr2eus/pr2-interface.l")
  (load "package://elevator_move_base_pr2/src/posedetectiondb-client.l")
  (pr2)
  (setq *ri* (instance pr2-interface :init))
  (setq objs (list (make-sony-rimokon) (make-sharp-rimokon))
	clients nil)
  (objects (append (list *pr2*) objs))
  ;;
  (dolist (obj objs)
    (push (instance posedetectiondb-client :init obj
		    :topic "/narrow_stereo/left/ObjectDetection"
		    :fixed-frame "/base_footprint")
	  clients))
  ;;
  (do-until-key
   (ros::spin-once)
   (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
   (x::window-main-one)
   (send *irtviewer* :redraw))
  ;;
  (send-all clients :unregister)
)
(warn ";; (test-detection-result)~%")

(generate-launcher)
(exit)

