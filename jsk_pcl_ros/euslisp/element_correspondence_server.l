(load "geometric_distance.l")


(ros::roseus-add-msgs "jsk_recognition_msgs")
(ros::roseus-add-srvs "jsk_recognition_msgs")
(ros::roseus-add-msgs "geometry_msgs")
(ros::roseus-add-srvs "std_srvs")


(defclass element-correspondence-server
  :super propertied-object
  :slots (obj
          edge-min-dist-thre
          poly-min-dist-thre
          pose-min-dist-thre
          pose-correspondence-flag)
  )

(defmethod element-correspondence-server
  ;; setup
  (:init
   (_obj
    &key
    (edge-distance-threshold 1000)
    (polygon-distance-threshold 1000)
    (pose-distance-threshold 1000)
    (setq pose-correspondence-flag nil)
    )
   ;; setup object
   (setq obj _obj)
   (setq edge-min-dist-thre edge-distance-threshold)
   (setq poly-min-dist-thre polygon-distance-threshold)
   (setq pose-min-dist-thre pose-distance-threshold)
   ;; setup ros
   (ros::rate 100)
   (ros::advertise-service "get_element_correspondence"
                           jsk_recognition_msgs::ElementCorrespondence
                           #'send self :element-corresp-cb)
   (ros::advertise-service "/enable_pose_correspondence"
                           std_srvs::Empty #'send self :enable-pose-correspondence)
   (ros::advertise-service "/disable_pose_correspondence"
                           std_srvs::Empty #'send self :disable-pose-correspondence)
   )
  ;; main
  (:main
   ()
   (while t
     (ros::spin-once)
     (ros::sleep)
     )
   )
  ;; callback
  (:enable-pose-correspondence
   (req)
   (setq pose-correspondence-flag t)
   (send req :response))
  (:disable-pose-correspondence
   (req)
   (setq pose-correspondence-flag nil)
   (send req :response))
  (:element-corresp-cb
   (req)
   (let* ((res (send req :response))
          (input-pose (send req :input_pose))
          (input-edges (send req :input_edges))
          (input-polys (send req :input_polygons))
          (input-poses (send req :input_poses))
          (input-coords (ros::tf-pose-stamped->coords input-pose))
          (corresponding-edge-ids)
          (corresponding-edges-msg)
          (corresponding-poly-ids)
          (corresponding-polys-msg)
          (corresponding-coords-list)
          (corresponding-poses-msg)
          (edge-list)
          (poly-list)
          (coords-list)
          )
     ;; update coords
     (send obj :newcoords input-coords)
     (cond ((derivedp obj cascaded-link)
            (send obj :update-descendants)
            )
           (t
            (send obj :worldcoords)
            ))
     ;; edge
     (dolist (edge-msg (send input-edges :edges))
       (let* ((start-pos (ros::tf-point->pos (send edge-msg :start_point)))
              (end-pos (ros::tf-point->pos (send edge-msg :end_point)))
              (edge0 (make-line start-pos end-pos))
              )
         (push edge0 edge-list)
         ))
     (setq edge-list (reverse edge-list))
     (setq corresponding-edge-ids
           (send self :get-corresponding-edge-ids
                 :edge-list edge-list :min-dist-thre edge-min-dist-thre))
     (setq corresponding-edges-msg
           (send self :get-edge-array-msg
                 :edge-id-list corresponding-edge-ids :header (send input-edges :header)))

     (send res :corresponding_edges corresponding-edges-msg)
     ;; polygon
     (dolist (poly-msg (send input-polys :polygons))
       (let* ((vertices (mapcar #'ros::tf-point->pos (send poly-msg :polygon :points)))
              (poly0 (apply #'make-polygon vertices))
              )
         (push poly0 poly-list)
         ))
     (setq poly-list (reverse poly-list))
     (setq corresponding-poly-ids
           (send self :get-corresponding-polygon-ids
                 :poly-list poly-list :min-dist-thre poly-min-dist-thre))
     (setq corresponding-polys-msg
           (send self :get-polygon-array-msg
                 :poly-id-list corresponding-poly-ids :header (send input-polys :header)))
     (send res :corresponding_polygons corresponding-polys-msg)
     ;; pose
     (dolist (pose-msg (send input-poses :poses))
       (let* ((label0 (send pose-msg :label))
              (coords0 (ros::tf-pose-stamped->coords (send pose-msg :pose)))
              )
         (send coords0 :put :label label0)
         (push coords0 coords-list)
         ))
     (setq coords-list (reverse coords-list))
     (setq corresponding-coords-list
           (send self :get-corresponding-coords-list
                 :coords-list coords-list :min-dist-thre pose-min-dist-thre))
     (setq corresponding-poses-msg
           (send self :get-pose-array-msg
                 :coords-list corresponding-coords-list :header (send input-poses :header)))
     (send res :corresponding_poses corresponding-poses-msg)
     res)
   )
  ;; get correspondance
  (:get-corresponding-edge-ids
   (&key
    (edge-list)
    (min-dist-thre)
    )
   (let* (obj-edge-id-list)
     (dolist (edge0 edge-list)
       (let* ((obj-edge-id
               (find-nearest-edge-from-object edge0 obj :min-dist-thre min-dist-thre))
              )
         (push obj-edge-id obj-edge-id-list)
         ))
     (reverse obj-edge-id-list)
     ))
  (:get-corresponding-polygon-ids
   (&key
    (poly-list)
    (min-dist-thre)
    )
   (let* (obj-poly-id-list)
     (dolist (poly0 poly-list)
       (let* ((obj-poly-id
               (find-nearest-polygon-from-object poly0 obj :min-dist-thre min-dist-thre))
              )
         (push obj-poly-id obj-poly-id-list)
         ))
     (reverse obj-poly-id-list)
     ))
  (:get-corresponding-coords-list
   (&key
    (coords-list)
    (min-dist-thre)
    )
   (let* ((corresponding-coords-list)
          )
     (cond (pose-correspondence-flag
            (dolist (coords0 coords-list)
              (let* ((label0 (send coords0 :get :label))
                     (corresponding-coords
                      (find-if #'(lambda (c) (string= (send c :get :label) label0))
                               (send obj :get :coords-list)))
                     )
                (push corresponding-coords corresponding-coords-list)
                ))
            )
           (t
            (setq corresponding-coords-list (make-list (length coords-list)))
            ))
     (reverse corresponding-coords-list)
     ))
  ;; get response message
  (:get-edge-array-msg
   (&key
    (edge-id-list)
    (header)
    )
   (let* ((msg (instance jsk_recognition_msgs::EdgeArray :init))
          (edge-msg-list)
          )
     (dolist (edge-id edge-id-list)
       (cond (edge-id
              (let* ((obj-edge (elt (send obj :edges) edge-id))
                     (start-pos (send obj-edge :point 0))
                     (end-pos (send obj-edge :point 1))
                     (edge-msg
                      (instance jsk_recognition_msgs::Edge :init
                                :header header
                                :start_point (ros::pos->tf-point start-pos)
                                :end_point (ros::pos->tf-point end-pos)
                                ))
                     )
                (push edge-msg edge-msg-list)
                ))
             (t
              (let* ((edge-msg ;; invalid edge
                      (instance jsk_recognition_msgs::Edge :init
                                :header header
                                :start_point (ros::pos->tf-point (float-vector 0 0 0))
                                :end_point (ros::pos->tf-point (float-vector 0 0 0))
                                ))
                     )
                (push edge-msg edge-msg-list)
                )
              ))
       )
     (send msg :header header)
     (send msg :edges (reverse edge-msg-list))
     msg
     ))
  (:get-polygon-array-msg
   (&key
    (poly-id-list)
    (header)
    )
   (let* ((msg (instance jsk_recognition_msgs::PolygonArray :init))
          (polyst-msg-list)
          )
     (dolist (poly-id poly-id-list)
       (cond (poly-id
              (let* ((obj-poly (elt (send obj :faces) poly-id))
                     (vertices (butlast (send obj-poly :vertices)))
                     (poly-msg
                      (instance geometry_msgs::Polygon :init
                                :points (mapcar #'ros::pos->tf-point32 vertices)
                                ))
                     (polyst-msg
                      (instance geometry_msgs::PolygonStamped :init
                                :header header
                                :polygon poly-msg
                                ))
                     )
                (push polyst-msg polyst-msg-list)
                ))
             (t
              (let* ((poly-msg ;; invalid polygon
                      (instance geometry_msgs::Polygon :init
                                :points nil
                                ))
                     (polyst-msg
                      (instance geometry_msgs::PolygonStamped :init
                                :header header
                                :polygon poly-msg
                                ))
                     )
                (push polyst-msg polyst-msg-list)
                )
              ))
       )
     (send msg :header header)
     (send msg :polygons (reverse polyst-msg-list))
     msg
     ))
  (:get-pose-array-msg
   (&key
    (coords-list)
    (header)
    )
   (let* ((msg (instance jsk_recognition_msgs::PoseLabeledArray :init))
          (pose-labeled-msg-list)
          )
     (dolist (coords0 coords-list)
       (cond (coords0
              (let* ((pose-msg
                      (instance geometry_msgs::PoseStamped :init
                                :header header
                                :pose (ros::coords->tf-pose coords0)))
                     (pose-labeled-msg
                      (instance jsk_recognition_msgs::PoseLabeled :init
                                :pose pose-msg
                                :label (send coords0 :get :label)
                                ))
                     )
                (push pose-labeled-msg pose-labeled-msg-list)
                ))
             (t ;; invalid pose
              (let* ((pose-msg
                      (instance geometry_msgs::PoseStamped :init
                                :header header
                                :pose (ros::coords->tf-pose (make-coords))))
                     (pose-labeled-msg
                      (instance jsk_recognition_msgs::PoseLabeled :init
                                :pose pose-msg
                                :label ""
                                ))
                     )
                (push pose-labeled-msg pose-labeled-msg-list)
                )
              ))
       )
     (send msg :header header)
     (send msg :poses (reverse pose-labeled-msg-list))
     msg
     ))
  )


(defun ros::pos->tf-point32 (pos)
  "Convert euslisp pos to geoometry_msgs::Point, this also converts [mm] to [m]"
  (instance geometry_msgs::point32 :init
            :x (* 0.001 (elt pos 0)) :y (* 0.001 (elt pos 1)) :z (* 0.001 (elt pos 2))))
