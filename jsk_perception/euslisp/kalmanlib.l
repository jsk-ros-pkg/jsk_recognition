;;;
;;; kalman.l
;;; kalman filter class
;;;
;;; written by k-okada@jsk.t.u-tokyo.ac.jp
;;;

(list "$Id: kalmanlib.l 59620 2013-09-25 03:50:20Z nozawa $")

(defclass kalman-filter
  :super propertied-object
  :slots (A H I Q R x_k z_k P_k x_k-1 P_k-1 K S))

(defmethod kalman-filter
  (:init
   (&key (state-dim 2) (measure-dim 2) (q-variance 1e-5) (r-variance 0.01))
   (setq A (unit-matrix state-dim)) ;;(make-matrix state-dim state-dim))
   (setq H (make-matrix measure-dim state-dim))
   (setq I (unit-matrix state-dim))
   (dotimes (i (car (array-dimensions (make-matrix measure-dim state-dim))))
     (setf (aref H i i) 1))
   (setq Q (scale-matrix q-variance (unit-matrix state-dim)))
   (setq R (scale-matrix r-variance (unit-matrix measure-dim)))
   (setq z_k (make-array measure-dim :element-type :float))
   (setq x_k (transform (transpose H) z_k))
   (setq P_k (unit-matrix state-dim))
   self)
  (:A (&optional m) (if m (setq A m) A))
  (:H (&optional m) (if m (setq H m) H))
  (:Q (&optional m) (if m (setq Q m) Q))
  (:R (&optional m) (if m (setq R m) R))
  (:x (&optional v) (if v (setq x_k v) x_k))
  (:predict
   ()
   (setq x_k-1 x_k)
   (setq P_k-1 P_k)
   (setq x_k (transform A x_k-1))
   (setq P_k (m+ (m* A (m* P_k-1 (transpose A))) Q))
   x_k)
  (:correct
   (z) ; measurement
   (setq z_k z)
   ;; S = HPH^ + R
   (setq S (m+ (m* H (m* P_k (transpose H))) R))
   ;; K = P H^ S-1
   (setq K (m* (m* P_k (transpose H)) (inverse-matrix S)))
   ;; x = x + K y
   ;; y = z - H x
   (setq x_k (v+ x_k (transform K (v- z_k (transform H x_k)))))
   ;; P = (I - K H) P
   (setq P_k (m* (m- I (m* K H)) P_k))
   t)
  (:proc
   (z)
   (send self :predict)
   (send self :correct z)
   (transform H x_k)
   )
  (:error
   nil
   (let ((z (v- z_k (transform H x_k))))
     (* (sqrt (matrix-determinant S)) (exp (* 0.5 (v. (transform z (inverse-matrix S)) z))))))
  )


;; (defun kalman-demo
;;   nil
;;   (dotimes (i 4)
;;     (kalman-demo-impl i)
;;     (unix:sleep 1)))

;; (defun kalman-demo-impl
;;   (&optional arg)
;;   (let (x1o y1o k z z^)
;;     (if (not (boundp '*gnuplot*))
;;         (setq *gnuplot* (gnuplot)))

;;     (unless arg
;;       (format t ";; usage kalman-demo-impl {0,1,2,3}~%")
;;       (setq arg 0))
;;     (case arg
;;       (0
;;        (format t ";; data -> line, kalman -> Position model~%"))
;;       (1
;;        (format t ";; data -> line, kalman -> Position-Velocity model~%"))
;;       (2
;;        (format t ";; data -> sin,  kalman -> Position model~%"))
;;       (t
;;        (format t ";; data -> sin,  kalman -> Position-Velocity model~%")))

;;     (setq x1o '(float-vector))
;;     (setq y1o '(float-vector))
;;     (dotimes (i 100)
;;       (push i x1o)
;;       (cond
;;        ((< arg 2)
;;         (push (+ -0.37727 (random-gauss 0 0.1)) y1o))
;;        (t
;;         (push (+ -0.37727 (random-gauss 0 0.05) (* 0.2 (sin (/ i 5.0)))) y1o))
;;        ))
;;     (setq x1o (eval (reverse x1o)))
;;     (setq y1o (eval (reverse y1o)))

;;     (cond
;;      ((or (eq arg 0) (eq arg 2))
;;       (setq k (instance kalman-filter :init :state-dim 2)))
;;      (t
;;       (setq k (instance kalman-filter :init :state-dim 4 :r-variance 0.001))
;;       (send k :A #2f((1 0 1 0)(0 1 0 1)(0 0 1 0)(0 0 0 1)))))

;;     (with-open-file
;;      (f "data.dat" :direction :output)
;;      (dotimes (i (length y1o))
;;        (format f "~A ~A~%" (elt x1o i) (elt y1o i))))
;;     (send *gnuplot* :command "plot [0:100][-0.6:0] 'data.dat' title \"data\" with linespoints")

;;     (send *gnuplot* :command "set y2tics")
;;     (send *gnuplot* :command "set y2range [0:0.01]")
;;     (with-open-file
;;      (f "kalman.dat" :direction :output)
;;      (dotimes (i (length y1o))
;;        (setq z (float-vector (elt x1o i) (elt y1o i)))
;;        (setq z^ (send k :proc z))
;;        (format f "~A ~A ~A~%" (elt z^ 0) (elt z^ 1) (send k :error))
;;        ))
;;     (send *gnuplot* :command "replot 'kalman.dat' title \"kalman\" with linespoints")
;;     (send *gnuplot* :command "replot 'kalman.dat' using 3 axes x1y2 title \"error\" with linespoints")
;;     (send *gnuplot* :command "replot -0.37727")
;;     )
;;   )

