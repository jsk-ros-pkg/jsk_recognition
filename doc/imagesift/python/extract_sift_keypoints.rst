Extract SIFT and visualize keypoints.
=====================================

Sample is available at ``imagesift/sample/sift_keypoints.py``


.. code-block:: python

  # load image
  >>> import os
  >>> import cv2
  >>> import rospkg
  >>> rp = rospkg.RosPack()
  >>> imgpath = os.path.join(rp.get_path('jsk_perception'), 'sample/ros_fuerte.jpg')
  >>> img = cv2.imread(imgpath, 0)  # gray-scale image

  >>> import imagesift

  # extract sift keypoints
  >>> frames, desc = imagesift.get_sift_keypoints(img)

  # draw keypoints on the image
  >>> out = imagesift.draw_sift_frames(img, frames)

  # view image with opencv2 window
  >>> cv2.imshow('sift image', out)
  >>> cv2.waitKey(0)


You will get output like:

.. image:: ./images/extract_sift_keypoints.png
