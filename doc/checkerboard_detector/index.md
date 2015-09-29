checkerboard_detector
====================

Trouble Shooting
----------------
* Q. Estimated checker board pose is not correct

  A. First check debug image and all the detected corner points correctly superimposed on camera view.
  * If the detected corner points is **not correct**, you need to modify checker board grid size (`grid_size` parameters).
  * If the detected corner points is **correct**, confirm checker board size (`rect_size` parmaeters) and intrinsic camera paramter is calibrated well.
