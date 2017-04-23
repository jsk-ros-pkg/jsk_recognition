#!/usr/bin/env roseus

(ros::roseus-add-msgs "jsk_recognition_msgs")

(load "element_correspondence_server.l")
(load "package://jsk_pcl_ros/euslisp/generate-object-model.l")


(ros::roseus "element_correspondence_server")

(let* (object-shape
       edge-distance-threshold
       polygon-distance-threshold
       )
  (setq object-shape (ros::get-param "~object_shape" "cube"))
  (setq edge-distance-threshold (ros::get-param "~edge_distance_threshold" 1000))
  (setq polygon-distance-threshold (ros::get-param "~polygon_distance_threshold" 1000))
  (generate-object-model object-shape)
  (setq *ecs* (instance element-correspondence-server :init
                        *obj*
                        :edge-distance-threshold edge-distance-threshold
                        :polygon-distance-threshold polygon-distance-threshold
                        ))
  (send *ecs* :main)
  )
