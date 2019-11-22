;;
;; generate launch-file for posedetection
;; from eus object models.
;;

(load "euslaunch.l")

(defconstant *pkgname* "jsk_perception")


;;
;; generate template for SIFT matching
;;

(defun get-template-info (obj aface)
  (let ((teximg (send aface :get :gl-textureimage))
	(texcoh (send aface :get :gl-texturecoords))
	imgpath template template-include)
    (when (and teximg texcoh)
      (let* ((texco (send texcoh :list))
	     (imgfile (if teximg (send teximg :name)))
	     (path (pathname imgfile))
	     (origin (send (car (rassoc #f(0 0) texco)) :pvertex aface))
	     (vx (v- (send (car (rassoc #f(1 0) texco)) :pvertex aface) origin))
	     (vy (v- (send (car (rassoc #f(0 1) texco)) :pvertex aface) origin))
	     (width (norm vx)) (height (norm vy))
	     (trans-rot (make-matrix 3 3)))
        (setq vx (scale (/ 1.0 width) vx) vy (scale (/ 1.0 height) vy))
        (set-matrix-column trans-rot 0 vy) ;; ny
        (set-matrix-column trans-rot 1 vx) ;; nx
        (set-matrix-column trans-rot 2 (v* vy vx)) ;; nz
	(setq trans
	      (send (make-coords :pos origin :rot trans-rot)
		    :transformation (make-cascoords)
		    ))
	(setq template (ros::resolve-ros-path
			(format nil "package://~A/template/~A.~A" *pkgname* (send path :name) (send path :type))))
  (setq template-include
        (format nil "$(find ~A)/template/~A.~A" *pkgname* (send path :name) (send path :type)))
	(when (or (not (probe-file template)) (file-newer imgfile template))
          (if (null (probe-file (send (pathname template) :directory-string)))
              (unix::mkdir (send (pathname template) :directory-string)))
	  (unix::system (format nil "cp -f ~a ~a" imgfile template)))
	(list template-include width height trans)
	))))

(defun gen-detection-nodes (obj)
  (let (nodes)
    (setq nodes
	  (mapcar
	   #'(lambda(aface)
	       (let* ((teximg (send aface :get :gl-textureimage))
		      (imgpath (pathname (send teximg :name)))
		      (imgname (str+ (send imgpath :name) "." (send imgpath :type)))
		      (tmpl-info (get-template-info obj aface))
		      trans quat)
		 (when (and teximg tmpl-info)
		   (setq trans (elt tmpl-info 3))
		   (setq quat (matrix2quaternion (send trans :rot)))
		   (instance rosnode :init "jsk_perception" "point_pose_extractor"
			     :name (format nil "point_pose_extractor_~a" (send obj :name))
			     :params
      (list
       (instance rosparam :init :name "window_name" :value (send obj :name))
       (instance rosparam :init :name "template_filename" :value (car tmpl-info))
       (instance rosparam :init :name "object_width" :value (* (elt tmpl-info 1) 0.001))
       (instance rosparam :init :name "object_height" :value (* (elt tmpl-info 2) 0.001))
       (instance rosparam :init :name "relative_pose" :value (concatenate float-vector (scale 0.001 (send trans :pos)) (subseq quat 1) (subseq quat 0 1)))
       (instance rosparam :init :name "viewer_window" :value "false"))
			     ))))
	   (send obj :faces)))
    (remove nil nodes)))

(defun gen-all-launch (objects launch-file-name &key (ns nil))
  ;; generate launchfile
  (with-open-file
   (launch-f launch-file-name :direction :output)
   (format launch-f "<launch>")
   (dolist (obj objects)
     (let (obj-launch obj-include xmllist xmltext nodes)
       (setq obj-launch
	     (format nil "~A/template/_~x_~A.xml" (ros::resolve-ros-path (format nil "package://~A" *pkgname*))
		     (system::address obj)
		     (send obj :name)))
       (setq obj-include
             (format nil "$(find ~A)/template/_~x_~A.xml" *pkgname* (system::address obj) (send obj :name)))
       (setq nodes (gen-detection-nodes obj))
       (send-all nodes :namespace ns)
       (setq xmllist (append (list "launch" nil)
			     (send-all nodes :list)))
       (setq xmltext (simple-list2xml xmllist
				      :newline "~%" :attrfmt " ~A=\"~A\""))
       (with-open-file (f obj-launch :direction :output)
		       (format f xmltext))
       (format launch-f "<include file=\"~A\"/>~%" obj-include)
     ))
   (format launch-f "</launch>")
   ))


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;
;; Old functions
;;
#|
(defun need-generate-template (imagefile)
  (let* ((path (pathname imagefile))
	 (template (ros::resolve-ros-path
		    (format nil "package://~A/template/~A.~A.pp" *pkgname* (send path :name) (send path :type)))))
  (or (not (probe-file template)) (file-newer imagefile template))
  ))

(defun gen-template-node (filename type width height coords)
  (let* ((scaled-coords (send (send coords :copy-worldcoords) :locate
			      (scale 0.001 (send coords :pos)) :world))
	 (trans-vec (subseq (array-entity (send scaled-coords :4x4)) 0 12))
	 (mat-str (format nil "~A" trans-vec))
	 (frame-id (if (eq (char type 0) #\/) type (str+ "/" type))))
    (setq mat-str (subseq mat-str 3 (1- (length mat-str)))) ;; remove "#f()"
    (instance rosnode :init "posedetectiondb" "PointPoseExtraction.py"
	      :args (format nil "--imagefilename=~A --type=~A --object-width=~A --object-height=~A --transform=\\\'~A\\\'"
			    filename frame-id (* 0.001 width) (* 0.001 height) mat-str
			    ))
    ))

(defun gen-template-node-from-face (obj aface type)
  (let ((teximg (send aface :get :gl-textureimage))
	(texcoh (send aface :get :gl-texturecoords)))
    (when (and teximg texcoh)
      (let* ((texco (send texcoh :list))
	     (imgfile (if teximg (send teximg :name)))
	     (origin (send (car (rassoc #f(0 0) texco)) :pvertex aface))
	     (vx (v- (send (car (rassoc #f(1 0) texco)) :pvertex aface) origin))
	     (vy (v- (send (car (rassoc #f(0 1) texco)) :pvertex aface) origin))
	     (width (norm vx)) (height (norm vy))
	     (nz (v- (send aface :normal))) (nx (scale (/ 1.0 width) vx)) (ny (scale (/ 1.0 height) vy))
	     (trans-rot (make-matrix 3 3)))
	(set-matrix-column trans-rot 0 nx)
	(set-matrix-column trans-rot 1 ny)
	(set-matrix-column trans-rot 2 nz)
	(setq trans
	      (send (make-coords :pos origin :rot trans-rot)
		    :transformation (send obj :worldcoords)
		    ))
	(when (need-generate-template imgfile)
	  (gen-template-node imgfile type width height trans))
	))))

(defun gen-template-nodes (obj)
  (remove nil (mapcar
	       #'(lambda(aface)(gen-template-node-from-face
				obj aface
				(format nil "~A" (send obj :name))))
	       (send obj :faces))))

(defun gen-template-name-from-imagefile (imagefilename)
  (let ((path (pathname imagefilename)))
    (str+ (send path :directory-string)
	  "template_" (send path :name) "." (send path :type) ".pp")))

(defun all-texture-names (objs)
  (remove nil
	  (send-all (send-all (flatten (send-all objs :faces)) :get :gl-textureimage) :name))
  )

;; main
(defun gen-all-template (objects &key (pkg "imagesift") (type "imagesift"))
  ;;generate template
  (let (featnode featpipe pipe cnt)
    (setq featnode (instance rosnode :init pkg type))
    (setq featpipe (launch (list featnode)))
    (unix:sleep 5)
    (setq cnt (count-if #'(lambda(s)(substringp "PointPoseExtraction" s))
			(ros::rosnode-list)))
    (dolist (gnode (mapcan #'gen-template-nodes objects))
      (unix::sleep 1)
      (ros::ros-info "launch template generator")
      (pprint (send gnode :list))
      ;;
      (let* ((args (coerce (remove t (mapcar #'(lambda(c)(or (eq c #\\) c)) (coerce (send gnode :args) cons))) string))
	     (cmd (format nil "rosrun posedetectiondb PointPoseExtraction.py ~A" args)))
	(pprint cmd)
	(unix::system cmd))
      )
    (format featpipe "~%") ;; exit image feature node
    (while (read-line featpipe nil nil))
    (close featpipe))
  ;; mv template file
  (dolist (teximgname (all-texture-names objects))
    (let* ((basedir (ros::resolve-ros-path (format nil "package://~A/" *pkgname*)))
	   (template (gen-template-name-from-imagefile teximgname))
	   (imgpath (pathname teximgname))
	   (imgname (str+ (send imgpath :name) "." (send imgpath :type))))
      (when (need-generate-template teximgname)
	(unix:system (format nil "mv ~A ~A/template/~A.pp" template basedir imgname)))
      ))
  )

;;
;; generate launch file
;;

(defun gen-detection-nodes-old (obj) ;; this is old / for python macher
  (let (nodes)
    (setq nodes (mapcar
		 #'(lambda(aface)
		     (let* ((teximg (send aface :get :gl-textureimage))
			    (imgpath (pathname (send teximg :name)))
			    (imgname (str+ (send imgpath :name) "." (send imgpath :type))))
		       (when teximg
			 (instance rosnode :init "posedetectiondb" "PointPoseExtraction.py"
				   :args (format nil "--template=$(find ~A)/template/~A.pp --hidegui --errorthresh=0.006 --neighthresh=0.82 --dminexpected=10 --ransaciters=200"
						 *pkgname* imgname)
				   ))))
		 (send obj :faces)))
    (remove nil nodes)))
|#
