#!/usr/bin/env roseus

(ros::load-ros-manifest "orbit_pantilt")
(ros::roseus "gazer")
(ros::wait-for-service "/orbit_pantilt/tilt_reset")
(ros::wait-for-service "/orbit_pantilt/pan_reset")
(ros::advertise "/orbit_pantilt/pan_tilt_command" orbit_pantilt::jointcommand 1)


(setq pan 0 tilt 0)
(format t ";; reset pan tilt~%")
(call-empty-service "/orbit_pantilt/pan_reset")
(unix::sleep 2)
(call-empty-service "/orbit_pantilt/tilt_reset")
(unix::sleep 2)

(setq *tfl* (instance ros::transform-listener :init))
(ros::rate 0.2)
(while (ros::ok)
  (send *tfl* :wait-for-transform "/orbit_base_link" "/base_link" (ros::time 0) 1)
  (setq *orbit-to-pr2* (send *tfl* :lookup-transform "/orbit_base_link" "/base_link" (ros::time 0)))
  (setq *xyz* (send *orbit-to-pr2* :pos))
  (setq *tan-xy* (/ (elt *xyz* 1) (elt *xyz* 0)))
  (setq *tan-xz* (/ (- (elt *xyz* 2)) (elt *xyz* 0)))
  (setq *angle-xy-rad* (atan *tan-xy*))
  (setq *angle-xz-rad* (atan *tan-xz*))
  (format t "~f ~f[deg]~%" (rad2deg *angle-xy-rad*) (rad2deg *angle-xz-rad*))

  (setq pan *angle-xy-rad*)
  (format t ";; pan  ~A " pan)
  (ros::publish "/orbit_pantilt/pan_tilt_command" (instance orbit_pantilt::jointcommand :init :pan pan :tilt tilt))
  (ros::sleep)

  (setq tilt *angle-xz-rad*)
  (format t ";; tilt ~A~%" tilt)
  (ros::publish "/orbit_pantilt/pan_tilt_command" (instance orbit_pantilt::jointcommand :init :pan pan :tilt tilt))

  (ros::sleep)
)
(print ";; done")
