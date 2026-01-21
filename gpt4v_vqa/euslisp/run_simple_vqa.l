#!/usr/bin/env roseus

(load "package://gpt4v_vqa/euslisp/utils.l")

(ros::roseus "run_simple_vqa")

(print "Question is \"Please describe the image briefly.\"")
(print (format nil "Answer is ~a" (run-vqa "Please describe the image briefly.")))
(exit)