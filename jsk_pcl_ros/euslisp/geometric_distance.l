;; edge distance ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun find-nearest-edge-from-object
  (detect-edge
   obj
   &key
   (calc-edge-distance-func
    #'calc-edge-distance-based-on-object-vertices)
   (min-dist-thre)
   )
  "
  find nearest edge from edge and object
  argument:
    detect-edge: input edge (instance of euslisp edge or line class)
    obj: object instance
    calc-edge-distance-func: function to calculate edge distance
                             #'calc-edge-distance-based-on-object-vertices and #'calc-edge-distance-based-on-edge-points is supported.
    min-dist-thre: threshold of min distance to edge
  return vlaue:
    id of nearest edge (nil if no edge is near within threshold)
  "
  (let* ((dist-list
          (mapcar
           #'(lambda (edge0) (funcall calc-edge-distance-func edge0 detect-edge obj))
           (send obj :edges)))
         (min-dist (apply #'min dist-list))
         (min-dist-id (position min-dist dist-list))
         )
    (when (and min-dist-thre (>= min-dist min-dist-thre))
      (setq min-dist-id nil))
    min-dist-id
    ))

(defun calc-edge-distance-based-on-object-vertices
  (obj-edge
   detect-edge
   obj
   &key
   (use-bounding-box? nil)
   )
  "
  calculate distance between object edge and detected edge based on transformation distance of object vertices for matching
  argument:
    obj-edge: object edge (instance of euslisp edge class)
    detect-edge: detected edge (instance of euslisp edge or line class)
    obj: object instance
  return vlaue:
    distance of obj-edge and detect-edge
  "
  (let* ((from-coords (get-edge-midcoords obj-edge))
         (to-coords (get-edge-foot-coords obj-edge detect-edge))
         (dist (calc-coords-distance-based-on-object-vertices
                from-coords to-coords obj :use-bounding-box? use-bounding-box?))
         )
    dist
    ))


;; face distance ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun find-nearest-polygon-from-object
  (detect-poly
   obj
   &key
   (calc-polygon-distance-func
    #'calc-polygon-distance-based-on-object-vertices)
   (min-dist-thre)
   )
  "
  find nearest polygon from polygon and object
  argument:
    detect-poly: input polygon (instance of euslisp polygon or plane class)
    obj: object instance
    calc-polygon-distance-func: function to calculate polygon distance
                                #'calc-polygon-distance-based-on-object-vertices and #'calc-polygon-distance-based-on-polygon-points is supported.
    min-dist-thre: threshold of min distance to polygon
  return vlaue:
    id of nearest polygon (nil if no polygon is near within threshold)
  "
  (let* ((dist-list
          (mapcar
           #'(lambda (poly0) (funcall calc-polygon-distance-func poly0 detect-poly obj))
           (send obj :faces)))
         (min-dist (apply #'min dist-list))
         (min-dist-id (position min-dist dist-list))
         )
    (when (and min-dist-thre (>= min-dist min-dist-thre))
      (setq min-dist-id nil))
    min-dist-id
    ))

(defun calc-polygon-distance-based-on-object-vertices
  (obj-poly
   detect-poly
   obj
   &key
   (use-bounding-box? nil)
   )
  "
  calculate distance between object face and detected plane based on transformation distance of object vertices for matching
  argument:
    obj-poly: object face (instance of euslisp face class)
    detect-poly: detected plane (instance of euslisp face, polygon, or plane class)
    obj: object instance
  return vlaue:
    distance of obj-poly and detect-poly
  "
  (let* ((from-coords (get-polygon-centroid-coords obj-poly))
         (to-coords (get-polygon-foot-coords obj-poly detect-poly))
         (dist (calc-coords-distance-based-on-object-vertices
                from-coords to-coords obj :use-bounding-box? use-bounding-box?))
         )
    dist
    ))


;; coords distance ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun calc-coords-distance-based-on-object-vertices
  (from-coords
   to-coords
   obj
   &key
   (use-bounding-box? nil)
   )
  "
  calculate distance between two coords based on transformation distance of object vertices for matching
  argument:
    from-coords: coords1
    to-coords: coords2
    obj: object instance
  return vlaue:
    distance of from-coords and to-coords
  "
  (let* ((original-vertices
          (if use-bounding-box?
              (send (send (send obj :box) :body) :vertices) (send obj :vertices)))
         (transformed-vertices
          (mapcar
           #'(lambda (vertex)
               (send (send (make-coords :pos (copy-seq vertex)) :transform (send from-coords :transformation to-coords) from-coords) :worldpos))
           original-vertices))
         (transform-dist-list
          (mapcar #'(lambda (x y) (norm (v- x y))) transformed-vertices original-vertices))
         (transform-dist-sum
          (apply #'+ transform-dist-list))
         (transform-dist-ave
          (/ transform-dist-sum (length original-vertices)))
         )
    transform-dist-ave
    ))


;; geometric utils ;;;;;;;;;;;;;;;;;;;;;;;;;;;;
(defun get-edge-foot-coords
  (obj-edge detect-edge)
  "
  get foot coords on detected edge from object edge
  "
  (let* ((obj-edge-midcoords (get-edge-midcoords obj-edge))
         (pos (get-nearest-point-on-line (send obj-edge-midcoords :pos) detect-edge))
         (obj-edge-dir (send obj-edge-midcoords :rotate-vector (float-vector 0 0 1)))
         (detect-edge-dir (normalize-vector (send detect-edge :direction)))
         (z-axis (scale (if (< (v. obj-edge-dir detect-edge-dir) 0.0) -1.0 1.0) detect-edge-dir))
         (x-vec (send obj-edge-midcoords :rotate-vector (float-vector 1 0 0)))
         (y-axis (normalize-vector (v* z-axis x-vec)))
         (x-axis (normalize-vector (v* y-axis z-axis)))
         (rot (transpose (matrix x-axis y-axis z-axis)))
         (detect-edge-foot-coords (make-coords :pos pos :rot rot))
         )
    detect-edge-foot-coords
    ))

(defun get-edge-midcoords
  (edge0)
  "
  get midcoords of object edge
  "
  (let* ((pos (send edge0 :point 0.5))
         (x-axis (send edge0 :binormal (car (send edge0 :faces))))
         (z-axis (send edge0 :direction))
         (y-axis (v* z-axis x-axis))
         (rot (transpose (matrix x-axis y-axis z-axis)))
         (edge-midcoords (make-coords :pos pos :rot rot))
         )
    edge-midcoords
    ))

(defun calc-point-line-distance
  (point0 edge)
  "
  calculate distance between point and line
  "
  (let* ((nearest-edge-point (get-nearest-point-on-line point0 edge))
         (dist (norm (v- point0 nearest-edge-point)))
         )
    dist
    ))

(defun get-nearest-point-on-line
  (point0 line0)
  "
  get point on line, which is nearest to target point
  cf. http://www.infra.kochi-tech.ac.jp/takagi/Survey2/7Parameter.pdf
  "
  (let* ((line-pos (send line0 :point 0))
         (line-dir (send line0 :direction))
         (p_b point0)
         (p_a line-pos)
         (v line-dir)
         (nearest-line-ratio (/ (v. v (v- p_b p_a)) (norm2 v)))
         (nearest-line-point (v+ p_a (scale nearest-line-ratio line-dir)))
         )
    nearest-line-point
    ))

(defun get-polygon-foot-coords
  (obj-poly detect-poly)
  "
  get foot coords on detected polygon from object polygon
  "
  (let* ((obj-poly-centroid-coords (get-polygon-centroid-coords obj-poly))
         (pos (get-nearest-point-on-plane (send obj-poly-centroid-coords :pos) detect-poly))
         (obj-poly-normal (send obj-poly-centroid-coords :rotate-vector (float-vector 0 0 1)))
         (detect-poly-normal (normalize-vector (send detect-poly :normal)))
         (z-axis (scale (if (< (v. obj-poly-normal detect-poly-normal) 0.0) -1.0 1.0) detect-poly-normal))
         (x-vec (send obj-poly-centroid-coords :rotate-vector (float-vector 1 0 0)))
         (y-axis (normalize-vector (v* z-axis x-vec)))
         (x-axis (normalize-vector (v* y-axis z-axis)))
         (rot (transpose (matrix x-axis y-axis z-axis)))
         (detect-poly-foot-coords (make-coords :pos pos :rot rot))
         )
    detect-poly-foot-coords
    ))

(defun get-polygon-centroid-coords
  (poly0)
  "
  get centroid coords of object face
  "
  (let* ((pos (scale (/ 1.0 (length (send poly0 :vertices))) (reduce #'v+ (send poly0 :vertices))))
         (z-axis (send poly0 :normal))
         (x-vec (send (car (send poly0 :edges)) :direction))
         (y-axis (v* z-axis x-vec))
         (x-axis (v* y-axis z-axis))
         (rot (transpose (matrix x-axis y-axis z-axis)))
         (poly-centroid-coords (make-coords :pos pos :rot rot))
         )
    poly-centroid-coords
    ))

(defun get-nearest-point-on-plane
  (point0 plane0)
  "
  get point on plane, which is nearest to target point
  cf. http://www.eyedeal.co.jp/product/eyemLib_caliper.pdf
  "
  (let* ((plane-pos (scale (/ 1.0 (length (send plane0 :vertices))) (reduce #'v+ (send plane0 :vertices))))
         (plane-normal (send plane0 :normal))
         (n plane-normal)
         (d (- (v. n plane-pos)))
         (p point0)
         (nearest-plane-point (v- p (scale (+ (v. n p) d) n)))
         )
    nearest-plane-point
    ))
