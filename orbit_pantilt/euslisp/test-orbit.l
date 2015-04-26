#!/usr/bin/env roseus

(ros::load-ros-manifest "orbit_pantilt")

(ros::roseus "orbit_test")
(ros::wait-for-service "/orbit_pantilt/tilt_reset")
(ros::wait-for-service "/orbit_pantilt/pan_reset")
(ros::advertise "/orbit_pantilt/pan_tilt_command" orbit_pantilt::jointcommand 1)

(format t "~%")
(format t ";; r        : reset pan tilt~%")
(format t ";; p [pan]  : control pan~%")
(format t ";; t [pan]  : control tilt~%")
(format t ";; q        : quit~%")

(setq pan 0 tilt 0)
(while (ros::ok)
  (format t ">")
  (finish-output t)
  (setq l (read-line t))
  (if (> (length l) 0)
      (case (elt l 0)
        (#\r
         (format t ";; reset pan tilt~%")
         (call-empty-service "/orbit_pantilt/pan_reset")
         (unix::sleep 2)
         (call-empty-service "/orbit_pantilt/tilt_reset")
         (unix::sleep 2))
        (#\p
         (setq pan (read-from-string (subseq l 2)))
         (format t ";; pan  ~A~%" pan)
         (ros::publish "/orbit_pantilt/pan_tilt_command" (instance orbit_pantilt::jointcommand :init :pan (deg2rad pan) :tilt (deg2rad tilt))))
        (#\t
         (setq tilt (read-from-string (subseq l 2)))
         (format t ";; tilt ~A~%" tilt)
         (ros::publish "/orbit_pantilt/pan_tilt_command" (instance orbit_pantilt::jointcommand :init :pan (deg2rad pan) :tilt (deg2rad tilt))))
        (#\q
         (return))
        (t
         (format t ";; unknoqn command ~A~%" t))
        ))
  )
(print ";; done")

