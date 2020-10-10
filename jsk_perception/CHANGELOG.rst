^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.15 (2020-10-10)
-------------------
* check if template/ direcotry exists, because this is auto-generated directory (`#2537 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2537>`_)

  * install within roseus_FOUND
  * check if template/ direcotry exists, because this is auto-generated directory

* Contributors: Kei Okada

1.2.14 (2020-10-09)
-------------------
* remove packages=['jsk_perceptoin'] (`#2536 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2536>`_)
* fix

```
  + /usr/bin/env PYTHONPATH=/opt/ros/melodic/lib/python2.7/dist-packages:/tmp/jsk_recognition-release/obj-x86_64-linux-gnu/lib/python2.7/dist-packages:/opt/ros/melodic/lib/python2.7/dist-packages:/home/k-okada/pynaoqi/pynaoqi-python2.7-2.5.5.5-linux64/lib/python2.7/site-packages CATKIN_BINARY_DIR=/tmp/jsk_recognition-release/obj-x86_64-linux-gnu /usr/bin/python2 /tmp/jsk_recognition-release/setup.py egg_info --egg-base /tmp/jsk_recognition-release/obj-x86_64-linux-gnu build --build-base /tmp/jsk_recognition-release/obj-x86_64-linux-gnu install --root=/tmp/jsk_recognition-release/debian/ros-melodic-jsk-perception --install-layout=deb --prefix=/opt/ros/melodic --install-scripts=/opt/ros/melodic/bin
  running egg_info
  creating /tmp/jsk_recognition-release/obj-x86_64-linux-gnu/jsk_perception.egg-info
  writing /tmp/jsk_recognition-release/obj-x86_64-linux-gnu/jsk_perception.egg-info/PKG-INFO
  writing top-level names to /tmp/jsk_recognition-release/obj-x86_64-linux-gnu/jsk_perception.egg-info/top_level.txt
  writing dependency_links to /tmp/jsk_recognition-release/obj-x86_64-linux-gnu/jsk_perception.egg-info/dependency_links.txt
  writing manifest file '/tmp/jsk_recognition-release/obj-x86_64-linux-gnu/jsk_perception.egg-info/SOURCES.txt'
  error: package directory 'jsk_perception' does not exist
  CMake Error at catkin_generated/safe_execute_install.cmake:4 (message):
  execute_process(/tmp/jsk_recognition-release/obj-x86_64-linux-gnu/catkin_generated/python_distutils_install.sh)
  returned error code
  Call Stack (most recent call first):
  cmake_install.cmake:41 (include)
  Makefile:97: recipe for target 'install' failed
```

* Contributors: Kei Okada

1.2.13 (2020-10-08)
-------------------
* fix logic to check chainer version (`#2534 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2534>`_)

  * add test to check `#2533 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2533>`_ regression

* Contributors: Kei Okada

1.2.12 (2020-10-03)
-------------------
* check if chainer is found before check version (`#2533 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2533>`_)

  * fixes http://build.ros.org/job/Nbin_uF64__jsk_perception__ubuntu_focal_amd64__binary/1/console and http://build.ros.org/job/Mbin_uB64__jsk_perception__ubuntu_bionic_amd64__binary/91/console

* Contributors: Kei Okada

1.2.11 (2020-10-01)
-------------------
* Add FCN8sDepthPredictionConcatFirst model to fcn_depth_prediction.py (`#2481 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2481>`_)

  * Update output file names
  * Read dataset directory from argument
  * Fix directory name of dataset extracted from tar ball
  * Flatten images for network input
  * Remove wrong transform of dataset from train_fcn_depth_prediction.py
  * Add training script of FCNDepthPredictionConcatFirst model
  * Move FCN8sDepthPrediction chainer models to jsk_recognition_utils
  * Add install script of mirror dataset
  * Fix typo of model path
  * Use cv2 version of colormap JET
  * Add trained model of FCN8sDepthPredictionConcatFirst
  * Add FCN8sDepthPredictionConcatFirst model to fcn_depth_prediction.py

* refactor sample launches in jsk_perception (`#2376 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2376>`_)
* Add nose mask publisher (`#2347 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2347>`_)
* [jsk_perception/people_pose_estimation_2d.py][jsk_perception/people_mask_publisher.py] Fix edge case bug (`#2465 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2465>`_)
* Publish ClusterPointIndices in ssd_object_detector.py (`#2467 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2467>`_)

  * add predict profilling message above cluster indices computation

* fix travis - skip noetic test into two jobs, using BUILD_PKGS - skip catkin_python_setup for indigo (`#2522 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2522>`_)
* Fix for  noetic / 20.04 (`#2507 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2507>`_)

  * jsk_perception/scripts: respect ROS_PYTHON_VERSION
  * support for opencv4 : jsk_perception
  * remove signals from find_package(Boost)
  * jsk_perception depends on roseus, but it sometimes hard to keep dependency
  * fix for python3, use 2to3 -f print, 2to3 -f except
  * upgrade package.xml to format=3, migrate to noetic with ROS_PYTHON_VERSION=2/3, use multiple ROS distro strategy http://wiki.ros.org/noetic/Migration

* more fix for `#2500 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2500>`_ (`#2502 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2502>`_)

  * fix print '' -> print('')

* fix print syntax in train_ssd.py (`#2500 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2500>`_)

  * fix print '' -> print('')

* [jsk_perception] support image with alpha in image_publisher (`#2479 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2479>`_)

  * fix image_publisher for loading grayscale image
  * use cv2 default type
  * add test for image with alpha channel
  * add sample for alpha image
  * fix for depth image
  * support image with alpha in image_publisher

* [jsk_perception] add program for training ssd with box annotation (`#2483 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2483>`_)
* show what should we do, if we have error on 'import chainer' (`#2491 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2491>`_)

  * use --clock for sample_image_cluster_indices_decomposer.launch, add --clock to sample_bounding_box_to_rect.launch does not work...
  * print how to intall cupy
  if you do not have cupy, it raises error
  ```
  [INFO] [1588763738.839739]: Read the image file: /home/k-okada/ws_recognition/src/jsk_recognition/jsk_perception/sample/object_detection_example_2.jpg
  [INFO] [1588763739.625133]: Loaded 43 labels
  Traceback (most recent call last):
  File "/home/k-okada/ws_recognition/src/jsk_recognition/jsk_perception/node_scripts/ssd_object_detector.py", line 207, in <module>
  ssd = SSDObjectDetector()
  File "/opt/ros/melodic/lib/python2.7/dist-packages/jsk_topic_tools/transport.py", line 26, in __call\_\_
  obj = type.__call_\_(cls, *args, **kwargs)
  File "/home/k-okada/ws_recognition/src/jsk_recognition/jsk_perception/node_scripts/ssd_object_detector.py", line 71, in __init\_\_
  chainer.cuda.get_device_from_id(self.gpu).use()
  File "/usr/local/lib/python2.7/dist-packages/chainer/backends/cuda.py", line 275, in get_device_from_id
  check_cuda_available()
  File "/usr/local/lib/python2.7/dist-packages/chainer/backends/cuda.py", line 138, in check_cuda_available
  raise RuntimeError(msg)
  RuntimeError: CUDA environment is not correctly set up
  (see https://github.com/chainer/chainer#installation).No module named cupy
  ``
  * show what should we do, if we have error on 'import chainer'
  ```
  Traceback (most recent call last):
  File "/home/k-okada/ws_recognition/src/jsk_recognition/jsk_perception/node_scripts/ssd_object_detector.py", line 26, in <module>
  import chainer
  File "/usr/local/lib/python2.7/dist-packages/chainer/__init_\_.py", line 10, in <module>
  from chainer import backends  # NOQA
  File "/usr/local/lib/python2.7/dist-packages/chainer/backends/__init_\_.py", line 1, in <module>
  from chainer.backends import cuda  # NOQA
  File "/usr/local/lib/python2.7/dist-packages/chainer/backends/cuda.py", line 77
  def shape(self) -> types.Shape:
  ^
  SyntaxError: invalid syntax
  ```
  c.f. https://github.com/jsk-ros-pkg/jsk_recognition/pull/2485

* add more arg INPUT_IMAGE (`#2492 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2492>`_)

  * arg name='INPUT_IMAGE' need to use default, instead of value, so that we can cheange the input name as ros args. 'value' is constant value and 'default' is default value, see http://wiki.ros.org/roslaunch/XML/arg

* jsk_perception/train_ssd.py fix error when out_dir is set (`#2493 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2493>`_)

* set chainer version less than 7.0.0 (`#2485 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2485>`_)

  * split test_bing to test_bing_output and test_bing_objectness
  * add time-limit to jsk_pcl_ros/test/test_linemod_trainer.test, jsk_perception/test/bing.test
  * jsk_perception/package.xml: node_scripts/pointit.py imports tf2_geometry_msgs
  * set time-limit=25 for timeout:30 tests
  * relax test conditions
  * set chainer version less than 7.0.0
  * jsjk_perception/train_ssd.py fix error when out_dir is set

* Fix test for consensus_tracking (`#2475 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2475>`_ from YutoUchimi/fix_consensus_tracking

* Parameterize frames, transformation and interpolation in virtual_camera_mono (`#2470 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2470>`_)

  * Change logger level of TransformException to WARN
  * Add test for virtual_camera_mono
  * Add sample for virtual_camera_mono
  * Parameterize virtual_camera_mono

* Convert audio data to spectrogram (`#2478 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2478>`_)

  * add unit to axis
  * remove unused files
  * add node to visualize spectrum
  * fix size of spectrogram
  * fix typo in launch
  * divide program into audio_to_spectrum and spectrum_to_spectrogram
  * fix comment
  * add test
  * use rosbag with /audio of 300Hz
  * use timer callback to publish spectrogram constantly
  * update comments and name of parameter
  * add sample program to convert audio message  to spectrogram

* Add train script and sample for SSD (`#2471 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2471>`_)

  * [jsk_perception] add program for training ssd with box annotation
  * use cv2 for cv_resize_backend
  * add classnames for ssd
  * add trained model in install_trained_data.py

* Add queue_size and slop param to TileImages (`#2453 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2453>`_)
* Fix label_id division by 256 -> 255 (`#2455 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2455>`_)

  * Fix label_id division by 256 -> 255
    Since `len(colormap)` is `255`, % 256 is wrong since it can return 255
    which raises IndexError.

* fix generate_readme.py and update readme (`#2442 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2442>`_)
* Publish human skelton msgs in OpenPose node (`#2437 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2437>`_)

  * add lines considering shoulder when predicting face region
  * add LIMB_PART param
  * enable to create nose mask image
  * [jsk_perception/node_scripts/people_pose_estimation_2d.py] fix edge case
  * [jsk_perception/node_scripts/people_mask_publisher.py] fix edge case

* Fix tile_image.py for Python3 (`#2452 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2452>`_)

* Fix label_image_decomposer.py for Python3 (`#2454 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2454>`_)
* Update to slic d77d6e8 (`#2450 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2450>`_)
* mask_rcnn_instance_segmentation: support loading yaml from file (`#2413 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2413>`_)
* pointit: add option '~use_arm' to select arm for pointing (`#2415 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2415>`_)
* Add sample, test and doc (`#2440 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2440>`_)

  * Fix condition of fatal message
  * Keep backward compatibility for ~dist_threshold
  * Add test for kalman-filtered-objectdetection-marker.l
  * Add sample for kalman-filtered-objectdetection-marker.l
  * Change permission of kalman-filtered-objectdetection-marker.l: 644->755
  * Update sample for RobotToMaskImage
  * Add sample for CollisionDetector
  * Merge branch 'master' into kinfu-fix
  * updae people_pose_estimation_2d.test
  * add visualization link in commentout
  * Add test for RobotToMaskImage
  * Add minimal sample for RobotToMaskImage, which is only for testing
  * Add test for SlidingWindowObjectDetector
  * Add sample for SlidingWindowObjectDetector
  * Support overriding parameters in manifest file
  * Add sample for sliding_window_object_detector_trainer_node
  * Add params for fg/bg training dataset image topics and output manifest file
  * Add test for ColorHistogramLabelMatch
  * Add sample for ColorHistogramLabelMatch
  * Add test SingleChannelHistogram
  * Add sample for SingleChannelHistogram
  * Explicitly depend of topic_tools because sample_polygon_array_color_histogram.launch uses this
  * Add test for PolygonArrayColorLikelihood
  * Add sample for PolygonArrayColorLikelihood
  * Suppress very long log of downloading pretrained weight in sample_deep_sort_tracker.launch
  * Add test for PolygonArrayColorHistogram
  * Add sample for PolygonArrayColorHistogram
  * Support selecting histogram index by rosparam in unwrap_histogram_with_range_array.py
  * Build SnakeSegmentation only when OpenCV<3
  * Add test for UnapplyMaskImage
  * Add sample for UnapplyMaskImage
  * Add test for TabletopColorDifferenceLikelihood
  * Add sample for TabletopColorDifferenceLikelihood
  * Add test for SnakeSegmentation
  * Add sample for SnakeSegmentation
  * Add test for Skeletonization
  * Add sample for Skeletonization
  * Add test for SaliencyMapGenerator
  * Add sample for SaliencyMapGenerator
  * Add test for ROIToRect
  * Add sample for ROIToRect
  * Fix output polygon vertices for ROIToRect
  * Add test for ROIToMaskImage
  * Add sample for ROIToMaskImage
  * Add test for RectToROI
  * Add sample for RectToROI
  * Add test for RectToMaskImage
  * Fix ROSTimeMoveBackward before publishing output in sample_rect_to_mask_image.launch
  * Fix point index for bottom right point of rectangle in rect_to_mask_image.cpp
  * Add test for ProjectImagePoint
  * Add sample for ProjectImagePoint
  * Add test for PolygonToMaskImage
  * Add sample for PolygonToMaskImage
  * Add test for PolygonArrayToLabelImage
  * Add sample for PolygonArrayToLabelImage
  * Add test for MaskImageToROI
  * Add sample for MaskImageToROI
  * Add test for GrabCut
  * Add sample for GrabCut
  * Disable fast_rcnn.test
  * Add test for FisheyeToPanorama
  * Add sample for FisheyeToPanorama
  * Add test for GaussianBlur
  * Add sample for GaussianBlur
  * Add test for YCCDecomposer
  * Add sample for YCCDecomposer
  * Add test for LabDecomposer
  * Add sample for LabDecomposer
  * Add test for RGBDecomposer
  * Add sample for RGBDecomposer
  * Add test for HSVDecomposer
  * Add sample for HSVDecomposer
  * Add test for morphological operators
  * Add sample for morphlogical operators such as ErodeMaskImage, Opening, MorphlogicalGradient, TopHat
  * Add test for pointit.py
  * Add sample for pointit.py
  * Remove unused import in pointit.py
  * Remove unused computation in get_marker func in pointit.py
  * Fix tf2 listener
  * Fix return value in find_pose func in pointit.py
  * Add test for unwrap_histogram_with_range_array.py
  * Add sample for unwrap_histogram_with_range_array.py
  * Add test for solidity_rag_merge.py
  * Add sample for solidity_rag_merge.py
  * Support networkX>=2 and scikit-image>=0.13 in solidity_rag_merge.py
  * Add test for non_maximum_suppression.py
  * Add sample for non_maximum_suppression.py
  * Add ROS topic API for non_maximum_suppression.py
  * pointit: add option '~use_arm' to select arm for pointing
  * mask_rcnn_instance_segmentation: support loading yaml from file
  * add jsk_perception/SubtractMaskImage
  * fix typo in sample_face_pose_estimation.launch
  * GPU -> gpu in face_pose_estimation.launch
  * use args in sample launch: GPU -> gpu
  * remove test_mode from sample_face_pose_estimation.launch
  * remove test_mode in sample_ssd_object_detector.launch
  * Do not use deprecated param in sample_pointit.launch
  * Fix use of deprecated param ~dist_threshold

* fixes scope bug on point_pose_extraction (`#2414 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2414>`_)
* [jsk_perception] Add trained maskrcnn model for 73b2 kitchen (`#2423 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2423>`_)

  * update kitchen pretrained model (`#9 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/9>`_)
  * [jsk_perception] Add trained maskrcnn model for 73b2 kitchen
  * add sample launch file using 73b2 kitchen model
  * update kitchen pretrained model
  * add sample launch for kitchen dataset

* update to use jsk_travis 0.5.0 (`#2439 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2439>`_)
  * skip deep_sort_tracker.test on indigo
  https://travis-ci.org/jsk-ros-pkg/jsk_recognition/jobs/549216064#L8697-L8733
  downloading SSD data(ssd300_voc0712_converted_2017_06_06.npz) failes with
  ```
  IOError: [Errno socket error] [Errno 1] _ssl.c:510: error:14077410:SSL routines:SSL23_GET_SERVER_HELLO:sslv3 alert handshake failure'
  ```
  do we need to update Python to 2.7.9? for indidgo ????
  https://stackoverflow.com/questions/54413685/insecureplatform-warning


  * Do not mix tab and space for indentation
  * Add test for mask_rcnn_instance_segmentaion.py, but comment out testing because GPU required
  * Add test for image_time_diff.py
  * Add sample for image_time_diff.py
  * Avoid crashing when ROS time moved backward in image_time_diff.py
  * Fix AttributeError in image_time_diff.py
  * Add test for fcn_depth_prediction, but do not run because unstable
  * Add test for fast_rcnn.py
  * Add test for binpack_rect_array.py
  * Add sample for binpack_rect_array.py
  * Add test for apply_context_to_label_probability
  * Add gpu arg to sample_apply_context_to_label_probability.launch
  * fix typo: skelton -> skeleton
  * publish skelton in people_pose_estimation_2d

* Add Mask R-CNN model trained with COCO dataset (~80 classes) (already included VOC model only detects ~20 classes) (`#2427 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2427>`_)
* MaskImageToPointIndices: support multi channel mask image (`#2409 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2409>`_)

  * fix mask rcnn 73b2 model classname typo (`#8 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/8>`_)

* point_pose_extractor: fix bug on scope
* point_pose_extractor: fill reliability

* Add sample for MaskImageToPointIndices

* add jsk_perception/SubtractMaskImage (`#2411 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2411>`_)

  * Fix typo of main node name

* Re-enable bing.test (`#2418 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2418>`_)

  * Fix target name of bing for testing

* Contributors: Fuki Furuta, Kei Okada, Kentaro Wada, Naoya Yamaguchi, Shingo Kitagawa, Yoshiki Obinata, Yuki Furuta, Yuto Uchimi, Iory Yanokura, Hideaki Ito, Taichi Higashide

1.2.10 (2019-03-27)
-------------------
* Fix error on setting device number other than 0 on multiple gpu env. (`#2412 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2412>`_)

  * face_pose_estimation: support multi gpu env
    mask_rcnn_instance_segmentation.py: support multi gpu env
    people_pose_estimation_2d.py: support multi gpu env
    ssd_object_detector.py: support multi gpu env

* Re-enable draw_classification_result.test (`#2401 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2401>`_)

  * Re-enable draw_classification_result.test
  * Increase slop for bof_histogram_extractor

* Re-enable color_histogram.test( `#2400 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2400>`_)

* Contributors: Yuki Furuta, Yuto Uchimi

1.2.9 (2019-02-23)
------------------

1.2.8 (2019-02-22)
------------------

1.2.7 (2019-02-14)
------------------
* [jsk_perception/ssd_object_detector.py] Add header for publishing result image (`#2367 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2367>`_ )
* [jsk_perception] Add deep_sort_tracker_node.py (`#2351 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2351>`_ )

  * [jsk_perception/deep_sort_net.py] Fixed deep_sort_net import
  * [jsk_perception/test/deep_sort_tracker.test] Disable gpu in test
  * [jsk_perception/sample_deep_sort_tracker.launch] Refactor
  * [jsk_perception/deep_sort_tracker_node.py] Modified import file not to depend on tensorflow
  * Revert "[jsk_perception/deep_sort_tracker] Add dependencies of tensorflow"
    This reverts commit 7dac944cfc9292d81b8bdb90d89e8100eda2bf3a.
  * [jsk_perception/deep_sort_tracker] Add dependencies of tensorflow
  * [jsk_perception/deep_sort_tracker] Install git submodule directory to node_scripts/deep_sort/deep_sort
  * [jsk_perception/deep_sort_tracker_node.py] Add target_labels param to specify input labels/recst
  * [jsk_perception/deep_sort_tracker_node.py] Add test
  * [jsk_perception/deep_sort_tracker_node.py] Renamed publish image topic vis -> viz
  * [jsk_perception/deep_sort_tracker_node.py] Publish labelarray
  * [jsk_perception/sample/deep_sort_tracker] Add pretrained model load
  * [jsk_perception/deep_sort_tracker_noder.py] Add node
  * [jsk_perception/deep_sort_tracker_node.py] Add sample
  * [jsk_perception/install_trained_data.py] Add deepsort trained model
  * [jsk_perception] Add deep_sort by gitsubmodule

* [doc] [jsk_perception] Add documentation (`#2385 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2385>`_ )

  * Rewrite matchtemplate.py with cv2
  * Add test for matchtemplate.py
  * Enable random_forest_server.test only in indigo.
  * Add sample for matchtemplate.py
  * Fix conversion for latest cv_bridge: imgmsg <-> cv2 <-> cv
  * Add test for fisheye_ray.py
  * Add sample for fisheye_ray.py
  * Fix for undefined global variable in fisheye_ray.py
  * Add test for random_forest_server
  * Publish ~output/debug_image in random_forest_client_sample.py
  * Fix for executing RandomForestClassifier
  * Remove unused sklearn module which causes ImportError in sklearn>=0.20
  * Add ~slop param to bof_histogram_extractor
  * Show viewer if gui:=true in sample_background_subtraction
  * Remove unused remapping in sparse_image.test
  * Fix sparse_image_encoder/decoder sample

* [jsk_perception] Support fcn8s_atonce model in fcn_object_segmentation.py (`#2375 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2375>`_ )

  * Fix typo: fcn8s_atonce -> fcn8s_at_once
  * Support fcn8s_atonce model in fcn_object_segmentation.py

* [jsk_perception] fix load path for kalmanlib.l (`#2377 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2377>`_ )
* [doc] [jsk_perception] [jsk_recognition_utils] Add guide to image recognition with deep learning (`#2365 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2365>`_)

  * Add doc for image annotation
  * Add annotate_images_with_labelme to index
  * Add dataset class for semantic segmentation
  * Add install_learning_datasets script
  * Download datasets during catkin build
  * Add .gitignore in learning_datasets/
  * Add train_fcn script
  * Set default learning_rate to valid value
  * Enable plotting from remote host as well
  * Add doc for training FCN
  * Add doc for starting deep learning with image dataset
  * Add how to create dataset, where to store it in documentation
  * Dump param for fcn_object_segmentation.py
  * Add InstanceSegmentationDataset
  * Add train script for Mask-RCNN
  * Fix model_name and outputs in train_fcn.md
  * Add doc for training Mask-RCNN

* Contributors: Kei Okada, Yuki Furuta, Yuto Uchimi, Iori Yanokura

1.2.6 (2018-11-02)
------------------
* Add hand pose detection (`#2324 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2324>`_)
  * [jsk_perception/people_pose_estimation.py] Fixed for cpu inference
  * [jsk_perception/people_pose_estimation.py] Diable train and enable_backprop
  * [jsk_perception/people_pose_estimation_2d] Add hand width offset
  * pointit: add handle exception on tf2
  * pointit: add min threshold
  * jsk_perception: add pointit
  * people_pose_estimation_2d: support hand detection

* [jsk_perception] Add human mesh recovery(estimate people 3d pose from 2d image) (`#2332 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2332>`_)
  * clean up jsk_perception/scripts/install_trained_data.py around if _chainer_available
  * [jsk_perception/human_mesh_recovery] Refactor
  * [jsk_perception/human_mesh_recovery] Add test
  * [jsk_perception/human_mesh_recovery] Add sample
  * [jsk_perception/human_mesh_recovery] Add install model file code
  * [jsk_perception/human_mesh_recovery] Add node

* [jsk_perception/openpose] Add resize image (`#2300 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2300>`_)
  * [jsk_perception/openpose] Fixed logic
  * [jsk_perception/openpose] Add warning
  * [jsk_perception/openpose] Add resize image

* [jsk_perception/ssd_object_detector] Add hand pretrained model (`#2333 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2333>`_)

* Fix install destination (`#2345 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2345>`_)
  * Install 'node_scripts', 'scripts', 'test' into SHARE_DESTINATION

* [jsk_perception/sample_mask_rcnn] Fixed typo. fps -> rate (`#2353 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2353>`_)

* [jsk_perception/mask_rcnn_instance_segmentation.py] Publish rects and class (`#2350 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2350>`_)

* [jsk_perception/point_pose_extractor.cpp] Correct grammer. 'could not found' -> 'could not find' (`#2349 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2349>`_)
* [jsk_perception/image_publisher.py] Add fov parameter for publishing valid camera info parameters (`#2340 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2340>`_)
  * [jsk_perception/image_publisher.py] Add warning when not specified fovx and fovy at the same time
  * [jsk_perception/sample_image_publisher.launch] Add fov parameter for kinectv2
  * [jsk_perception/image_publisher.py] Add fov parameter for camera info

* [jsk_perception/sample_bof_object_recognition.launch] Fixed path of trained bof data(`#2337 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2337>`_)
  * [jsk_perception/install_trained_data.py] Add trained bof data for sklearn==0.20.0

* fix for jsk-ros-pkg/jsk_common/pull/1586 (`#2311 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2311>`_)
  * to avoid add_custom_target cannot create target install_sample_data because another target with the same name already exists errors
  
* Use diagnostic nodelet for EuclideanClustering and other nodelets (`#2301 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2301>`_)
  * jsk_pcl_ros: euclidean_clustering: use dianogistc nodelet
    Use DiagnosticNodelet::updateDiagnostic preferrably
  
* support SSD512 for ssd_object_detector (`#2305 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2305>`_)
  * move ssd_train_dataset to scripts

* [jsk_perception/face_pose_estimation] Fixed orientation of face pose (`#2304 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2304>`_)
  * [jsk_perception/face] Modified rviz
  * [jsk_perception/face] Add debug image of face pose
  * [jsk_perception/face] Fixed orientation of publish pose
  * [jsk_perception/face] Fixed pretrained model loader

* Enable Openpose Node for chainer 4.0.0 (`#2295 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2295>`_)
  * [jsk_perception/scripts] Modified url
  * [jsk_perception/scripts] Modified format
  * [jsk_perception/scripts] Modified openpose's weight
  * [jsk_perception] Modified openpose

* [jsk_perception] install config dir (`#2294 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2294>`_)
* Update chainer_mask_rcnn to 0.3.0 (`#2293 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2293>`_
* Fix for AssertionError in fast_rcnn.py (`#2281 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2281>`_)
  * Ignore whether cuda is available or not in fast_rcnn.py
  * Allow ~gpu as rosparam in fast_rcnn
  * Fix for AssertionError in fast_rcnn.py

* Re-enable tests which use chainer inside them (`#2280 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2280>`_)
  * Re-enable all tests which use chainer
  * Re-enable tests which use chainer inside them

* Set required=true for samples to fast finish in testsMerge pull request (`#2274 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2274>`_)
* Refactor cmake of jsk_perception (`#2275 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2275>`_)
  * Apply Eigen -> Eigen3 migration (Eigen also works)  http://wiki.ros.org/jade/Migration
  * Remove no need libsiftfast dependency

* fix travia and reduce dependency for jsk_pcl_ros (`#2276 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2276>`_)
  * skip test for `#2272 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2272>`_
  * Set required=true for samples to fast finish in tests
    Sometimes the test fails because of unexpected errors.
    In that case, it is better that the test quickly finish with errors.
  * skip more tests

* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Riku Shigematsu, Shingo Kitagawa, Yuto Uchimi, Iori Yanokura

1.2.5 (2018-04-09)
------------------
* Add MaskRCNNInstanceSegmentation node (`#2257 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2257>`_)
  * MaskRCNN node publishes label imgs that have class and instance id
  * Add ~bg_label to label_image_decomposer which is not colorized
  * Add ~cval param to apply_mask_image
  * Add MaskRCNNInstanceSegmentation node

* Improve topic name visualization in tile_image.py (`#2256 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2256>`_)
  * Comment out draw_classification_result test
  * Improve visualization in tile_image.py
    - Use FONT_HERSHEY_SIMPLEX.
    - Adjust font_scale according to the new font.

* [jsk_perception/draw_classification_result.py] use LINE_AA for opencv3 in kinetic (`#2247 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2247>`_)
  * enable draw_classification_result test
  * remove unused variables and imports
  * use LINE_AA for opencv3 in kinetic

* Add fcn_depth_prediction node (`#2244 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2244>`_)
  * [jsk_perception] Fix function name in fcn_depth_prediction.py
  * [jsk_perception] Add sample of fcn_depth_prediction
  * [jsk_perception] Add trained data for fcn_depth_prediction to install_trained_data
  * [jsk_perception] Add fcn_depth_prediction node
* [jsk_perception/fast_rcnn.py] fast_rcnn node to follow chainer-v2 version (`#2249 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2249>`_)
  * add MODEL arg for fast rcnn launch
  * check chainer version for volatile variable

* [jsk_perception/label_image_decomposer.py] check img.ndim for gray scale image (`#2248 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2248>`_)
  * check img.ndim for gray scale image

* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Shingo Kitagawa, Yuto Uchimi

1.2.4 (2018-01-12)
------------------
* jsk_perception: install template dir (`#2222 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2222>`_)
* Contributors: Yuki Furuta

1.2.3 (2017-11-23)
------------------
* jsk_perception: add face_pose_estimation (`#2207 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2207>`_)
* jsk_perception: people_pose_estimation_2d.py: add option not to synchronize camera info

* jsk_perception: use 'find' in generated eusmodel launch file (`#2215 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2215>`_)
* add timestamp for diff_image (`#2216 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2216>`_)
* jsk_percetion: add ssd object detector (`#2204 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2204>`_ from furushchev/ssd)
* Drop hydro from CI on Travis (`#2217 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2217>`_)
  * Remove color_histogram test that won't work on Travis

* Capability of specifying shape for tiling images (`#2208 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2208>`_)
  * Refactor tile_image.py about self._shape
  * Validate ~shape param of tile_image.py
    - modified:   tile_image.py
  * Capability of specifying shape for tiling images

* Add ~alpha param to label_image_decomposer to tune the overlay (`#2211 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2211>`_)
  * Make ~alpha param of label_image_decomposer to dynparam
  * Add ~alpha param to label_image_decomposer to tune the overlay
  * Add option to visualize label image without sync by ~only_label option

* jsk_perception: people_pose_estimation_2d.py: unsynchronize camera info (`#2206 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2206>`_)
* Add node for visualization of (labeled) rectangle region on 2D image (`#2205 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2205>`_)
  * jsk_perception: draw_rects: disable resubscribing on hydro
  * jsk_perception: use jsk_recognition_msgs::Rect for rect instead of geometry_msgs::PolygonStamped
  * jsk_perception: add nodelet for drawing rects on image
  * jsk_perception: use classification result for FastRCNN

* Split test of fcn_object_segmentation to avoid MemoryError Because loading 2 FCN8s model is too heavy on PCs with small memories. (`#2200 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2200>`_)
* [jsk_perception, slic_super_pixels] add parameter, publish_debug_images (`#2181 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2181>`_)
* Regional feature based object recognition using ResNet (`#2172 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2172>`_)
  * Rename to regional_feature_based_object_recognition
  * Remove params pretrained_model and mean_file
  * Sort add_rostest
  * Add test for feature_based_object_recognition
  * Download files and make the sample work
  * Add ResNetFeature
  * Fix bug in feature_based_object_recognition
  * Add feature based object recognition node
  * Large color variation in draw_classification_result
  * Display image even though some topics have not come yet
  * Fix nan values in ProbabilityImageClassifier

* node_scripts/apply_context_to_label_probability: make sure candidates is list  because it can be tuple, which cause error (`#2185 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2185>`_)
* Fix ignore_labels out of range for the input label/proba image (`#2184 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2184>`_)
  * Update sample of label/probability_image_classifier
* Fixes on probabilistic image classifier (`#2177 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2177>`_)
  * If no candidates, candidates_fixed should be ignored
* src/bounding_box_to_rect.cpp: Convert bounding box to mask (`#2176 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2176>`_)
  * Add sample_rect_to_mask_image.launch
  * support BoundingBox as input topic type as well as BoundingBoxArray

* jsk_perception: fix indent in creating people pose (`#2179 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2179>`_)


* Contributors: Yuki Furuta, Kei Okada, Kentaro Wada, Naoki Hiraoka, Shingo Kitagawa, Yohei Kakiuchi, Yuto Uchimi

1.2.2 (2017-07-23)
------------------
* add bg_label in apply_context_to_label_probability (`#2175 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2175>`_)
  * Remove no need ~use_topic flag
  * Refactor to handle fixed candidates in ApplyContextToLabelProbability
  * add bg_label in apply_context_to_label_probability

* fix bug in label_image_classifier (`#2174 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2174>`_)
  * Update label_image_classifier.py
  * fix bug in label_image_classifier

* Contributors: Kentaro Wada, Shingo Kitagawa

1.2.1 (2017-07-15)
------------------
* If chainer is not installed, use v2 (`#2167 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2167>`_)
  * chainer can not install in ros build firm

* Contributors: Kei Okada

1.2.0 (2017-07-15)
------------------
* [jsk_perception][people_pose_estimation_2d] publish image only when subscribed (`#2164 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2164>`_)

* Enhance PeoplePoseEstimation2D (`#2162 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2162>`_)
  * Fix run_depend on rviz
  * Install different pre-trained model according to the version of chainer
  * Support 16UC1 depth image in PeoplePoseEstimation2D
  * Visualize people 3D pose on rviz in sample
  * Add orientation to people 3d pose
  * Create point cloud in play_rosbag_people.xml
  * Fix AttributeError of argsort in cupy == 1.0.1

* [jsk_perception][jsk_recognition_utils] support chainer-v2 in alexnet and vgg16 (`#2153 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2153>`_)
  * enable alexnet and vgg test
  * fix syntax in vgg16_object_recognition
  * alexnet and vgg16 support chainer-v2

* Contributors: Kentaro Wada, Shingo Kitagawa, Yuki Furuta

1.1.3 (2017-07-07)
------------------
* [jsk_perception] add FCN-based classifiers (`#2142 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2142>`_)
  * make FCN-based classifiers pass test
  * mask_image_generator run only when use_mask=true
  * add voc_target_names yaml
  * FCN-based classifiers publish full result
  * add sample and test of fcn-based classifiers
  * add probability_image_classifier node
  * add label_image_classifier node

* [jsk_perception] squeeze mask to image dim=2 in fcn segmentation (`#2144 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2144>`_)
  * check mask ndim before squeeze
  * add use_mask sample and test for FCN segmentation
  * fix typo in fcn segmentation
  * squeeze mask to image dim=2 in fcn segmentation

* [jsk_perception/polygon_to_mask] add error message of frame_id (`#2125 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2125>`_)
  * [jsk_perception/polygon_to_mask_image] add error message when frame_id is not correct.

* [jsk_perception] apply candidates node supports topic update (`#2143 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2143>`_)
  * node_scripts/apply_context_to_label_probability: update Label msg API
  * node_scripts/apply_context_to_label_probability: apply candiates support topic update

* [jsk_perception] PeoplePoseEstimation2D (`#2115 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2115>`_)
  * [jsk_perception][people_pose_estimation_2d.py] keep compatibility chainer v1
  * [jsk_perception/people_pose_estimation_2d] Fixed missed numpy/cupy type
  * [jsk_perception/people_pose_estimation_2d] Changed sample bag file
  * [jsk_perception/people_pose_estimation_2d] Add people_mask_publisher
  * [jsk_perception/people_pose_estimation_2d] Publishe 2d image pose
  * [jsk_recogntion_msgs/PoseArray] Add score
  * [jsk_perception/people_pose_estimation_2d] Fixed install sample bag
  * [jsk_perception/people_pose_estimation_2d] Delete duplicated code
  * [jsk_perception/people_pose_estimation_2d] Modified type of PeoplePose.msg
  * [jsk_perception/people_pose_estimation_2d] Fiexed publish img encodings
  * [jsk_perception/people_pose_estimation_2d] Add test

* [jsk_perception/people_pose] Fixed typo and publish rect images. (`#2146 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2146>`_ )
  * [jsk_perception/people_pose] Refactor. Delete unnecessary code
  * [jsk_perception/people_pose] Bug fix. Publish rectified image
  * [jsk_perception/people_pose] Fix typo
  * [jsk_perception/people_pose] Delete pcl dependencies

* [jsk_perception/draw_rect_array.py] check polygon_msg list size (`#2114 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2114>`_ )
* [jsk_perception/mask_image_to_rect.cpp] check indices size before execute boundingRect (`#2113 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2113>`_ )
  * [jsk_perception] check indices size before execute boundingRect
  * jsk_perception/src/mask_image_to_rect.cpp: publish topic even if list is empty

* Contributors: Yuki Furuta, Kanae Kochigami, Masaki Murooka, Shingo Kitagawa, Iori Yanokura

1.1.2 (2017-06-16)
------------------
* label_image_decomposer.py: Faster and better visualization of segmentation (`#2109 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2109>`_ )
* fcn_object_segmentation.{launch,py} : Support .npz in chainermodel (https://github.com/jsk-ros-pkg/jsk_recognition/commit/19d7a2ac09bab2b470a8b06e0ed98d072b4958d4)
* fcn_object_segmentation.{launch,py} : Show deprecated warning for ~model_h5 in fcn_object_segmentation https://github.com/jsk-ros-pkg/jsk_recognition/commit/8d9be278a4ce019f4e026883a30785be874c6a16
* Support chainer v2 in fcn_object_segmentation.py  (`#2107 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2107>`_ )
* tile_image.py : Improve visualization in sample_fuse_depth_image https://github.com/jsk-ros-pkg/jsk_recognition/commit/6caa4c6f5039cb49cf0d07f43a6954a287b8ed35
* Stop using deprecated logging func in jsk_topic_tools (`#2097 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2097>`_ )
  * Stop using deprecated jsk_logxxx
* Refactor cmake to find robot_self_filter (`#2089 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2089>`_ )
* [jsk_percption][jsk_recogniton_utils] add imagenet_object_recognition launch and its sample (`#2085 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2085>`_ )
  * add sample_imagenet_object_recognition launch
  * use imagenet launch in alexnet sample launch
  * add imagenet_object_recognition.launch
  * move imagenet_target_names in config
  * install bvlc_vgg16 chainermodel
  * format API in vgg16: model_h5 -> model_file
  * format Alex -> AlexNet
* [jsk_perception] add AlexNet object recognition node (`#2083 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2083>`_ )
 * inherit VGG16ObjectRecognition in AlexNet
  * rename alex to alexnet
  * mv imagenet_target_names.yaml in sample/config
  * add test for alex_object_recognition
  * add sample for alex_object_recognition
  * add alex_object_recognition node
* jsk_perception/test/bof_histogram_extractor.test: increase time-limit for test_bof_histogram_extractor (`#2079 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2079>`_)
* fix typo in fcn_object_segmentation (`#2076 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2076>`_)
  * Improve the location of squeezing batch axis https://github.com/jsk-ros-pkg/jsk_recognition/commit/ddf46101d2d02e7bd18261542a2bacb456bf6e11
* Remove unexpectedly introduced torch rosdep key (`#2074 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2074>`_)
* FilterMaskImageWithSize: Filter mask image with its size  (`#2062 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2062>`_)
  * Add flag of ~use_reference to minimize overhead of synchronizing
    - modified:   ../doc/jsk_perception/nodes/filter_mask_image_with_size.md
    - modified:   include/jsk_perception/filter_mask_image_with_size.h
    - modified:   sample/sample_filter_mask_image_with_size.launch
    - modified:   src/filter_mask_image_with_size.cpp
  * filter_mask_image_with_size.cpp: Improve rosinfo https://github.com/jsk-ros-pkg/jsk_recognition/commit/5b5455c46f8397d6aa7e1c3d3501e87bf39326ca
  * Add sample, test & doc for FilterMaskImageWithSize https://github.com/jsk-ros-pkg/jsk_recognition/commit/14931792da009ef9468bc1ec3d6419005aca9335
    -	new file:   doc/jsk_perception/nodes/filter_mask_image_with_size.md
    -	new file:   doc/jsk_perception/nodes/images/filter_mask_image_with_size.gif
    -	modified:   jsk_perception/CMakeLists.txt
    -	new file:   jsk_perception/sample/sample_filter_mask_image_with_size.launch
    -	new file:   jsk_perception/test/filter_mask_image_with_size.test
  * Filter mask image with its size
    Modified:
    - jsk_perception/CMakeLists.txt
    - jsk_perception/include/jsk_perception/multiply_mask_image.h
    - jsk_perception/plugins/nodelet/libjsk_perception.xml
    Added:
    - jsk_perception/cfg/FilterMaskImageWithSize.cfg
    - jsk_perception/include/jsk_perception/filter_mask_image_with_size.h
    - jsk_perception/src/filter_mask_image_with_size.cpp
* Add ~approximate_sync param to ConsensusTracking  (`#2067 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2067>`_)
  Modified:
  - doc/jsk_perception/nodes/consensus_tracking.rst
  - jsk_perception/include/jsk_perception/consensus_tracking.h
  - jsk_perception/src/consensus_tracking.cpp
* FlowVelocityThresholding: Thresholding with velocity of optical flow (`#2060 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2060>`_ )
  * Add sample/test for FlowVelocityThresholding
    -	new file:   jsk_perception/nodes/flow_velocity_thresholding.md
    -	new file:   jsk_perception/nodes/images/flow_velocity_thresholding.gif
    -	modified:   ../jsk_perception/CMakeLists.txt
    -	new file:   ../jsk_perception/sample/sample_flow_velocity_thresholding.launch
    -	new file:   ../jsk_perception/test/flow_velocity_thresholding.test
  * Thresholding with velocity of optical flow
    -	modified:   CMakeLists.txt
    -	new file:   cfg/FlowVelocityThresholding.cfg
    -	new file:   include/jsk_perception/flow_velocity_thresholding.h
    -	modified:   plugins/nodelet/libjsk_perception.xml
    -	new file:   src/flow_velocity_thresholding.cpp
* Generate README by script (`#2064 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2064>`_ )
* fix typo in fcn_object_segmentation.py (`#2063 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2063>`_ )
* Add ~queue_size param to MultiplyMaskImage (`#2061 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2061>`_ )
  Modified:
  - doc/jsk_perception/nodes/multiply_mask_image.md
  - jsk_perception/src/multiply_mask_image.cpp
* Enhance fcn_object_segmentation.py with PyTorch backend (`#2051 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2051>`_ )
  * Optimization for faster processing
    - modified: jsk_perception/node_scripts/fcn_object_segmentation.py
  * Fix api of fcn_object_segmentation.py with PyTorch
    - modified: jsk_perception/node_scripts/fcn_object_segmentation.py
  * Raise error for unavailable torch & torchfcn
  * Remove install_pytorch.sh
  * Revert "Install packages to devel space"
    This reverts commit 40e068fc6788087c3a11f914269e93a4538be72e.
  * Fix method
  * Install packages to devel space
    - new file:   install_pytorch.py
    - deleted:    install_pytorch.sh
  * Install PyTorch for CUDA8.0 with rosdep
  * Add instruction of installing torchfcn
  * Remove not needed lines
* [jsk_perception] Add concave_hull_mask_image (`#2045 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2045>`_ )
  * [jsk_perception/concave_hull_mask_image] Fixed header
  * [jsk_perception/concave_hull_mask_image] Fixed consistency of cfg files
  * [jsk_perception/concave_hull_mask_image] Fixed max area size
  * [jsk_perception/concave_hull_mask_image] Fixed cfg for limit of contour area size for inf
  * [jsk_perception/concave_hull_mask_image] Fixed namespace of filter2D
  * [jsk_perception/concave_hull_mask_image] Fixed include header lists
  * [jsk_perception/concave_hull_mask_image] Fixed year

* [jsk_perception/apply_mask_image] Add negative option (`#2025 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2025>`_ )
* [jsk_perception][detection_interface.l] fix: changing object name  affects unexpected side effect (`#1974 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1974>`_ )
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Yuki Furuta, Iory Yanokura

1.1.1 (2017-03-04)
------------------

1.1.0 (2017-02-09)
------------------

1.0.4 (2017-02-09)
------------------
* package.xml: python-chainer -> python-chainer-pip (`#2014 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/2014>`_)
* Contributors: Kentaro Wada

1.0.3 (2017-02-08)
------------------
* Fix cpp format of consensus_tracking(`#1999 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1999>`_)
* Contributors: Kentaro Wada

1.0.2 (2017-01-12)
------------------
* fix typo in vgg16_object_recognition (`#1990 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1990>`_)
* No longer required python-gdown dependency
  Because python-gdown-pip is installed via jsk_data (`#1989 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1989>`_)
* Disable bing test on Travis (`#1985 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1985>`_)
  Currently the node `bing` seems not used/changed frequently
  because it requires opencv3, and I have no time to analyze the
  unstable test on Travis/Jenkins. That's why I'm disabling it.
  For `#1962 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1962>`_
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa

1.0.1 (2016-12-13)
------------------
* jsk_perception/node_scripts/speak_when_label_found.py: Speak when target labels are found ( `#1923 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1923>`_)
* Contributors: Kentaro Wada

1.0.0 (2016-12-12)
------------------
* Fix for kinetic build (`#1943 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1943>`_)
* Add missing packages(jsk_data, opencv_apps) to find_package (`#1984 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1984>`_)
* Add test & sample

  * calc_flow   (`#1959 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1959>`_)
  * background_subtraction   (`#1959 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1959>`_)
  * mask_image_to_rect   (`#1961 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1961>`_)
  * Add test & sample for grid_label  (`#1960 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1960>`_)
  * Add sample for colorize_float_image (`#1956 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1956>`_)

* Draw rects on image with PolygonStamped input (`#1961 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1961>`_)
* sample/sample_rect_array_actual_size_filter.launch : Fix typo of sample data path (`#1955 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1955>`_)
* colorize_float_image.cpp : Fill black color to nan region (`#1956 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1956>`_)
* scripts/install_sample_data.py : Fix wrong filename in install_sample_data.py (`#1954 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1954>`_)
* remove depends to driver_base (`#1943 <https://github.com/jsk-ros-pkg/jsk_recognition/pull/1943>`_)
* Contributors: Kei Okada, Kentaro Wada

0.3.29 (2016-10-30)
-------------------
* CMakeLists.txt: install nodelet.xml: for get to care about install process in #1929
* Contributors: Kei Okada

0.3.28 (2016-10-29)
-------------------
* [Major Release] Copy jsk_pcl_ros/srv and  jsk_perception/srv files to jsk_recognition_msgs (`#1914 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1914>`_ )
* Copy deprecated srv files to jsk_recognition_msgs
  - jsk_pcl_ros/srv -> jsk_recognition_msgs/srv
  - jsk_perception/srv -> jsk_recognition_msgs/srv
  TODO
  - 1. Migrate current code for srv files in jsk_recognition_msgs
  - 2. Remove srv files in jsk_pcl_ros and jsk_perception
* Contributors: Kei Okada, Kentaro Wada

0.3.27 (2016-10-29)
-------------------
* Fix rosdep installation for jsk_perception with pip (`#1883 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1883>`_ )
  * Fix pip installation with libleveldb-dev installation
* Publish only masks by split_fore_background.py (`#1791 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1791>`_ )

  * Stabilize split_fore_background.test
  * Fix nan region as mask 0 region
  * Remove synchronization in split_fore_background.py

* Remove extract_images_sync that merged in image_view (`#1633 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1633>`_ )
* Remove not used codes: image_saver_sync, publish_header (`#1651 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1651>`_ )

   * they will be merged in image_view package.
   * for https://github.com/jsk-ros-pkg/jsk_recognition/issues/1648#issuecomment-217344813

* Contributors: Kei Okada, Kentaro Wada

0.3.26 (2016-10-27)
-------------------
* Stop using deprecated jsk_topic_tools/log_utils.h (`#1933 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1933>`_)
* Fix unparsable nodelet pluginlib xml file (`#1929 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1929>`_)

* libcmt: Node to track object on 2D image: ConsensusTracking (`#1918 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1918>`_)

  * jsk_perception ConsensusTracking depends on libcmt which is not released on hydro
  * libcmt 2.0.17 has been released (`#1924 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1924>`_)
  * check if header file is installed, before 2.0.17
  * Fix encoding conversion of ROSMsg <-> cv::Mat
  * Add test for consensus_tracking
  * Install sample data for consensus_tracking
  * Add sample of consensus tracking
  * Check window is initialized to start tracking
  * Synchronize polygon and image to set initial tracking window
  * Rename to sample/sample_consensus_tracking.launch
  * Fix coding style of consensus_tracking (follow existing code)
  * Fix year for license
  * Fix name of nodelet of ConsensusTracking
  * Fix place of pkg_check_modules in CMakeLists
  * use package-config version libcmt
  * publish mask image generated from result
  * [jsk_perception] add README and set_rect subscriber which will restart tracking
  * [jsk_perception] add cmt_nodelet depending on libcmt

* Fix for alphabetical order in package.xml (`#1908 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1908>`_)

* apply_context_to_label_probability: Node to apply context to label probability (`#1901 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1901>`_)
  * Add sample for apply_context_to_label_probability
  * Visualize label_names in label_image_decomposer
  * Use default GPU=0 in sample_fcn_object_segmentation.launch
    Because it does not work with GPU=-1, CPU mode.
  * Apply context to label probability

* Stabilize jsk_perception/sklearn_classifier.test (`#1877 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1877>`_)
* Stabilize jsk_perception/bing.test (`#1877 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1877>`_)
* label_image_decomposer.py: Stop using scipy fromimage that is not supported by apt version (`#1890 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1890>`_)
* Make the test pass (`#1897 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1897>`_)
  * Stabilize test for label_image_decomposer
  * Stabilize test for sklearn_classifer
  * Stabilize test for bof_histogram_extractor
  * Comment out unstable test on travis
* Add quality to heightmap (`#1886 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1886>`_)
  * [colorize_float_image] fix document and change parameter name.
  * [jsk_perception, colorize_float_image] fix to handle multi channel image
* fcn_object_segmentation.py: Set bg label for uncertain region of FCN prediction (`#1881 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1881>`_)
* Contributors: Kei Okada, Kentaro Wada, Yohei Kakiuchi, Yuto Inagaki

0.3.25 (2016-09-16)
-------------------

0.3.24 (2016-09-15)
-------------------
* CMakeLists.txt : jsk_data is required in build time, used in scripts/install_sample_data
* Contributors: Kei Okada

0.3.23 (2016-09-14)
-------------------
* euslisp/eusmodel_template_gen_utils.l: create directory if tepmlate path is not found
* CMakeLists.txt : Makefile.slic is no longer used
* Contributors: Kei Okada

0.3.22 (2016-09-13)
-------------------
* Basically, if the angle is less than 0, just add 180. Likewise if the angle is greater than 180, just subtract by 180. https://github.com/jsk-ros-pkg/jsk_recognition/pull/1593/files#r77976906
* Sobel operator with higher kernel can give better response https://github.com/jsk-ros-pkg/jsk_recognition/pull/1593#discussion_r77976333
* [jsk_perception] slic as submodule
* sparse_image_encoder.cpp: need to escape %
* remove orientationistogram is not used
* set defiend values to protected member variables
* add doc for image_time_diff.py
* [jsk_perception] Remain executable API for nodes which is moved to opencv_apps
  Delete deprecated API's cfg and src files.
* Declare jsk_add_rostest in all distros
* Add jsk\_ prefix for local macros
* Refactor: jsk_perception_add_rostest -> _add_rostest
* Refactor: jsk_perception -> ${PROJECT_NAME}
* Refactor: jsk_perception_nodelet -> _add_nodelet
* Sort service files
* Fix if block syntax
  - Use endif()
  - Use quote "" for VERSION_GREATER
* Fix missing CATKIN_DEPENDS of posedetection_msgs
* Fix node executables installation by introducing macro
* Organize cmake setup order
  1. Initialization
  2. Download
  3. Catkin setup
  4. Build
  5. Install
  6. Test
* Add sample/test for blob_detector (#1849)
  * Add sample/test for blob_detector
  * Rename mask image file for understandable name
* Fix special character for double to print (#1836)
  * Fix special character for double to print
  * Add unit for percentage in sparse_image_encoder info printing
* Add sample & test for color_histogram node
* Fix image dimension robustness in ExtractImageChannel
* [jsk_perception/src/polygon_to_mask_image.cpp] add warning message when no camera info is available.
* Add test for extract_image_channel.py
* Add sample for extract_image_channel.py
* Extract image channel for channel value in rosparam
* disable global set ssl verification  to fase
* Add test for RectArrayToDensityImage
* Add sample for RectArrayToDensityImage
* Add sample for selective_search.py
* Convert rect array to density image
* Publish probability image in fcn_object_segmentation.py
* Publish whole black mask if no contour is found
* Use matplotlib.use('Agg') to make it work on server (without window)
* Update sample/test for drawn label names in label_image_decomposer
* Decompose labels with their names listed as legend
* Test LabelToMaskImage
* Add sample for LabelToMaskImage
* Node to convert label to mask image
* Use std::vector instead of cv::vector for OpenCV3
* Get bounding object mask image from noisy mask image
* replace cv::vector to std::vector
* enable to use cv::vector in opencv-3.x
* Merge pull request #1740 from wkentaro/fcn
  Fully Convolutional Networks for Object Segmentation
* [jsk_perception/src/virtual_camera_mono.cpp] process only when subscribed
* [jsk_perception/fast_rcnn] Modified avoiding size of rects is 0 case
* Catch error which unexpected size of mask
* Use larger buff_size to process input message with queue_size=1
* Use mask image to enhance the object recognition result
* Use timer and load img file when reconfigured in image_publisher
* Add python-fcn-pip in package.xml
* Add fcn_object_segmentation.launch
* Large size buff_size is required for taking time callback
* Test fcn_object_segmentation.py
* Sample for fcn_object_segmentation.py
* Fully Convolutional Networks for Object Segmentation
* Use small sized image for stable testing
* Make test for sklearn_classifier stable
* Make test for label_image_decomposer stable
* Add sample for slic_super_pixels
* Download trained_data in multiprocess
* Stop drawing boundary on label_image_decomposer
  - Not so pretty
  - Maybe Takes time
* Skip when no contours in BoundingRectMaskImage
* Test RectArrayActualSizeFilter
* Add sample for RectArrayActualSizeFilter
* Fix RectArrayActualSizeFilter in terms of size filtering
* Merge pull request #1731 from wkentaro/warn-no-test
  Warnings for without test node/nodelets
* Merge pull request #1732 from wkentaro/test-with-bof
  Add test for bof_histogram_extractor.py and sklearn_classifier.py
* jsk_perception/CMakeList.sxt: eigen_INCLUDE_DIRS must be located after catkin_INCLUDE_DIRS
* [jsk_perception] fix bug in solidity_rag_merge
* [polygon_array_color_histogram, polygon_array_color_likelihood] add queue size for message filter
* Warnings for without test node/nodelets
* Add test for bof_histogram_extractor.py and sklearn_classifier.py
* [polygon_array_color_likelihood] add code for reading yaml with latest yaml-cpp
* [jsk_pcl_ros] Fix mistake of rect_array_actual_size_filter
* Add sample for label_image_decomposer and use it in testing
* Add test, sample, and documentation for OverlayImageColorOnMono
* Add dynamic reconfigure for OverlayImageColorOnMono
* Implement OverlayImageColorOnMono
* Merge pull request #1697 from wkentaro/rectify-mask-image
  Implement ConvexHullMaskImage
* Add sample for mask_image_to_label.py
* Rename publish_fixed_images.launch -> sample_image_publisher.launch
* Use natural name of rqt_gui perspective for bof_object_recognition sample
* Add sample & test for BoundingRectMaskImage
* Implement BoundingRectMaskImage
* Add sample & test for ConvexHullMaskImage
* Implement ConvexHullMaskImage
* Add sample & test for BoundingRectMaskImage
* Implement BoundingRectMaskImage
* Add sample & test for MultiplyMaskImage
* Add sample & test for AddMaskImage
* Fix wrong mask size generated by MaskImageGenerator
  Fix #1701
* Add sample & test for MaskImageGenerator
* Add sample for apply_mask_image
* Install trained_data all time with dependency on ALL
* Merge pull request #1658 from wkentaro/color_pyx
  [jsk_recognition_utils] Add label color utility function
* Add test for 'rect_array_to_image_marker.py'
* Use labelcolormap in 'rect_array_to_image_marker.py'
* Use labelcolormap in 'draw_rect_array.py'
* Rename download_trained_data -> install_trained_data.py
  To follow install_test_data.py.
* Comment out test for vgg16_object_recognition does not work in Jenkins
* Install h5py via rosdep and apt
* Install vgg16 trained model
* Recognize object with VGG16 net
* Rename vgg16 -> vgg16_fast_rcnn
* Fix typo in bof_histogram_extractor.py
* Implement drawing node of classification result
* Rename fast_rcnn_caffenet -> fast_rcnn
* Remove dependency on rbgirshick/fast-rcnn
* CMakeLists.txt:  on Hydro  contains /opt/ros/hydro/include so we need to add after catkin_INCLUDE_DIRS
* Merge pull request #1627 from wkentaro/use-jsk_data
  [jsk_perception] Use jsk_data download_data function for test_data
* Merge pull request #1628 from wkentaro/download-jsk_data-trained-data
  [jsk_perception] Download trained_data with jsk_data function
* Use jsk_data download_data function for test_data
* Download trained_data with jsk_data function
* Add roslaunch_add_file_check with add_rostest
* Comment out bof_object_recognition.test because of no resolved imagesift depends
* Support latest sklearn in BoF feature extraction
* Make jsk_perception depend on imagesift for BoF
* Migrate completely jsk_perception/image_utils.h to jsk_recognition_utils/cv_utils.h
* Stable ros version check by STRGREATER
* Deprecated create_feature0d_dataset.[py,launch]
  Please use create_sift_dataset.py.
* Make it stable image_cluster_indices_decomposer.test
* Make selective_search.test be stable
* Make slic_super_pixels.test be stable
* Make colorize_float_image.test be stable
* Make colorize_labels test stable
* Make apply_mask_image.test be stable
* Make bof_object_recognition.test stable
* Make kmeans.test be stable
* Make bing.test be stable
* Make jsk_perception depend on image_view2 for ImageMaker2 message
* Fix opencv version condition for bing.test (#1638)
* [jsk_perception] Test tile_image.py (#1635)
  * Follow name convention sample_tile_image.launch
  * Test tile_image.py
* Test colorize_float_image (#1636)
* Test mask_image_to_label.py (#1634)
* [jsk_perception] Add test for BoF object recognition sample (#1626)
  * Refactor: BoF object recognition sample filname
  * Add test for BoF object recognition sample
* Test apply mask image (#1615)
  Modified:
  - jsk_perception/CMakeLists.txt
  Added:
  - jsk_perception/test/apply_mask_image.test
* Add rqt_gui perspective file for BoF sample (#1622)
* Test colorize labels (#1614)
  Modified:
  - jsk_perception/CMakeLists.txt
  Added:
  - jsk_perception/test/colorize_labels.test
* Condition to find OpenCV 3 (> 2.9.9) (#1603)
* Test KMeans (#1612)
  Modified:
  - jsk_perception/CMakeLists.txt
  Added:
  - jsk_perception/test/kmeans.test
* Compile some nodes only when OpenMP found (#1604)
* Stop passing -z flag to ld with clang (#1602)
* [jsk_perception] Find OpenMP as an optional module (#1600)
  * Find OpenMP as an optional module
  * Fix indent of cmake
* Refactoring: Rename test file for consistency (#1611)
* [jsk_perception] Test image_publisher.py (#1613)
  * Refactoring: remap ~output/camera_info to ~camera_info
  This is a natural output topic design especially for image_pipeline package.
  * Test image_publisher.py
  Added:
  - jsk_perception/test/image_publisher.test
* [jsk_perception] BING: Binarized Normed Gradients for Objectness Estimation at 300fps (#1598)
  * Add trained_data/
  * Add bing
  * Download trained_data for bing
  * Documentation about bing
  * Add test and sample for bing
  * Download trained_data for bing automatically
* Add trained_data/ (#1597)
* clf save directory fixed (#1539)
* [jsk_perception/image_cluster_indices_decomposer] fix typo (#1592)
* Contributors: Kei Okada, Kentaro Wada, Kim Heecheol, Masaki Murooka, Ryohei Ueda, Shingo Kitagawa, Shintaro Hori, Yohei Kakiuchi, Yuki Furuta, Iori Yanokura, Hiroto Mizohana

0.3.21 (2016-04-15)
-------------------

0.3.20 (2016-04-14)
-------------------
* Add sample/test for image_cluster_indices_decomposer.py (`#1580 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1580>`_)
* Add sample and test for BoundingBoxToRect (`#1577 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1577>`_)
  * Add sample for BoundingBoxToRect
  Modified:
  - jsk_perception/CMakeLists.txt
  Added:
  - jsk_perception/sample/sample_bounding_box_to_rect.launch
  - jsk_perception/scripts/install_sample_data.py
  - jsk_perception/test_data/.gitignore
  * Add test for BoundingBoxToRect
  * add an example to the documentation
  * modified document
* [jsk_perception/bounding_box_to_rect] add rosparam approximate sync and queue_size (`#1583 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1583>`_)
  * [jsk_perception/bounding_box_to_rect] add approximate sync and queue_size param
  * [jsk_perception/bounding_box_to_rect] add parameters in doc
* Visualize ClusterPointIndices for image (`#1579 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1579>`_)
* Install python executables
  * Install python executables
* Refactor: Make test filenames consistent
* Fix typo in 'test/test_split_fore_background.test'
* Merge pull request `#1568 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1568>`_ from wkentaro/draw-rect-array
  [jsk_perception/draw_rect_array.py] Draw rect_array onto a image
* Add test for jsk_perception/draw_rect_array.py
  Modified:
  - jsk_perception/CMakeLists.txt
  Added:
  - jsk_perception/test/draw_rect_array.test
* Documentize draw_rect_array.py
* Draw rect_array onto a image
  Added:
  - jsk_perception/node_scripts/draw_rect_array.py
* Add example for fast_rcnn_caffenet.py
* Subscribe rect_array as object location proposals
* Test jsk_perception/selective_search.py
* Pass RGB image to dlib.find_candidate_object_locations
  Modified:
  - jsk_perception/node_scripts/selective_search.py
* [jsk_perception] include opencv header in rect_array_actual_size_filter.h
* [jsk_perception] Add RectArrayActualSizeFilter
  Filtering array of rectangle regions based on actual size estimated from
  depth image.
* Remove duplicated roslint in test_depend
  Modified:
  - jsk_perception/package.xml
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Shingo Kitagawa, Yusuke Niitani

0.3.19 (2016-03-22)
-------------------
* remove rosbuild from run/build depend
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* Contributors: Kei Okada

0.3.18 (2016-03-21)
-------------------
* jsk_perception/CMakeLists.txt: remove depends to rosbuild
* Contributors: Kei Okada

0.3.17 (2016-03-20)
-------------------
* remove dynamic_reconfigure.parameter_generator, which only used for rosbuild
* [jsk_perception] binpack_rect_array.py to enumerate jsk_recognition_msgs/RectArray
* [jsk_perception] Add selective_search.py
* [jsk_perception] Use timer callback to speed up tile_image with no_sync:=true
* [jsk_perception] Cache concatenated image to speed up
* Contributors: Kei Okada, Ryohei Ueda

0.3.16 (2016-02-11)
-------------------
* Merge pull request `#1531 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1531>`_ from k-okada/sed_package_xml
  .travis.yml: sed package.xml to use opencv3
* remove image_view2 from find_package(catkin)
* [jsk_perception/CMakeLists.txt] call one of find_package or pkg_check_modules for robot_self_filter.
* [jsk_perception] Set queue_size=1 for tile_image.py
* [jsk_perception] Fix variable names in edge_detector.cpp
* [jsk_perception] Publish result after initialization
* Contributors: Kei Okada, Masaki Murooka, Ryohei Ueda

0.3.15 (2016-02-09)
-------------------
* U and V has strange library options; https://github.com/ros/rosdistro/pull/10436#issuecomment-180763393
* [jsk_perception] Do not subscribe camera info in calc_flow
* [jsk_perception] Add more 2d feature samples
* Fix label probabilities output message
  Modified:
  - jsk_perception/node_scripts/sklearn_classifier.py
* Add queue_size option for bof_histogram_extractor
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.14 (2016-02-04)
-------------------
* Merge pull request #1513 from garaemon/bounding-box-to-rect-array
  [jsk_perception] BoundingBoxToRectArray and rect_array_to_image_marker.py
* Add ~queue_size option for synchronization
  Modified:
  - jsk_perception/include/jsk_perception/apply_mask_image.h
  - jsk_perception/src/apply_mask_image.cpp
* [jsk_perception/ApplyMask] Add option to clip mask image
  Modified:
  - jsk_perception/include/jsk_perception/apply_mask_image.h
  - jsk_perception/src/apply_mask_image.cpp
* [jsk_perception/tile_image.py] Add ~no_sync parameter to disable
  synchronization of input topics.
* [jsk_perception] Skip for empty sift features
  Modified:
  - jsk_perception/node_scripts/bof_histogram_extractor.py
* [jsk_perception] BoundingBoxToRectArray and rect_array_to_image_marker.py
* [jsk_perception] [kalman-filtered-objectdetection-marker.l] fix code
* added default num_threads\_ value and modified readme.md
* Merge branch 'master' of https://github.com/jsk-ros-pkg/jsk_recognition into saliency_map_generator
  Conflicts:
  jsk_perception/CMakeLists.txt
* [jsk_perception] Except index error on SolidityRagMerge
  Modified:
  - jsk_perception/node_scripts/solidity_rag_merge.py
* parallelized main loop
* [jsk_perception/bof_histogram_extractor.py] Skip if only background image
* [jsk_perception] Skip empty image
* [jsk_perception] Publish info in sample launch file
  Modified:
  - jsk_perception/sample/publish_fixed_images.launch
* [jsk_perception] Stop using deprecated PLUGINLIB_DECLARE_CLASS
  Modified:
  - jsk_perception/src/color_histogram.cpp
  - jsk_perception/src/edge_detector.cpp
  - jsk_perception/src/hough_circles.cpp
  - jsk_perception/src/sparse_image_decoder.cpp
  - jsk_perception/src/sparse_image_encoder.cpp
* [jsk_perception] Add solidity_rag_merge
  This is to find image region with high solidity.
  Firstly, I will use this for vacuum gripper's approach point
  decision making.
  Added:
  - jsk_perception/node_scripts/solidity_rag_merge.py
* [jsk_perception] Set header correctly
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* Merge pull request #1457 from wkentaro/fix-unconfigured-cmake-packagexml
  [jsk_perception] Fix unconfigured cmake and manifest
* Merge pull request #1455 from wkentaro/publish-label-fg-bg
  [jsk_perception] Publish label fg/bg decomposed masks
* [jsk_perception] Check ROS_DISTRO for find_package of robot_self_filter
* [jsk_perception] Fix unconfigured cmake and manifest
  Modified:
  - jsk_perception/CMakeLists.txt
  - jsk_perception/package.xml
* [jsk_perception] Keep original encoding and scale to visualize
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] ColorizeLabels info -> debug
  Modified:
  - jsk_perception/src/colorize_labels.cpp
* [jsk_perception] Add roslint_cpp not as rostest
  Modified:
  jsk_perception/CMakeLists.txt
* [jsk_perception] Publish label fg/bg decomposed masks
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* Merge pull request #1398 from wkentaro/roslint-test-for-node-scripts
  [jsk_perception] Run roslint for python code
* [jsk_perception] Visualize label in label_image_decomposer.py
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] Read reference color histogram from a yaml file in PolygonArrayColorLikelihood
  to avoid race condition between input topics
  Modified:
  - doc/jsk_perception/nodes/polygon_array_color_likelihood.md
  - jsk_perception/CMakeLists.txt
  - jsk_perception/include/jsk_perception/polygon_array_color_likelihood.h
  - jsk_perception/package.xml
  - jsk_perception/src/polygon_array_color_likelihood.cpp
* [jsk_perception] Keep original resolution if all the input images has
  same shape and add ~draw_input_topic parameter to draw topic name on
  the tiled images
  Modified:
  - jsk_perception/node_scripts/tile_image.py
  - jsk_recognition_utils/python/jsk_recognition_utils/visualize.py
* Merge pull request #1426 from wkentaro/merge-sklearn-to-jsk-perception
  Merge sklearn to jsk_perception
* [jsk_perception] Add basic_2d_features.launch to overview
  effective technique
  Added:
  - jsk_perception/launch/basic_2d_features.launch
* [jsk_perception] Run roslint for python code
* Merge pull request #1438 from wkentaro/image-to-label
  [jsk_perception] Add image_to_label.py
* [jsk_perception] Use StrictVersions instead of ROS_DISTRO
  Modified:
  - jsk_perception/node_scripts/tile_image.py
* [jsk_perception/label_image_decomposer.py] Fix typo
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception/label_image_decomposer.py] Can specify queue_size
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] Fix typo
  Modified:
  - jsk_perception/node_scripts/label_image_decomposer.py
* [jsk_perception] Fix tile_image.py for hydro.
  1. Disable approximate sync for hydro. it's not supported on hydro
  2. Use PIL.Image.frombytes instead of PIL.Image.fromstring
* [jsk_perception] Add image_to_label.py
  Added:
  - jsk_perception/node_scripts/image_to_label.py
* [jsk_perception] Fix typo in bof_histogram_extractor.py
  Modified:
  - jsk_perception/node_scripts/bof_histogram_extractor.py
* Merge sklearn to jsk_perception
  Modified:
  jsk_pcl_ros/CMakeLists.txt
  jsk_pcl_ros/package.xml
  jsk_perception/package.xml
  Added:
  jsk_perception/node_scripts/random_forest_server.py
  jsk_perception/sample/random_forest_client_sample.py
  jsk_perception/sample/random_forest_sample.launch
  jsk_perception/sample/random_forest_sample_data_x.txt
  jsk_perception/sample/random_forest_sample_data_y.txt
* added param for printing fps to frame
* nodelet for computing image space saliency map
* Contributors: Kamada Hitoshi, Kei Okada, Kentaro Wada, Ryohei Ueda, Krishneel Chaudhary

0.3.13 (2015-12-19)
-------------------

0.3.12 (2015-12-19)
-------------------
* Revert "[jsk_perception] slic as submodule"
* Contributors: Ryohei Ueda

0.3.11 (2015-12-18)
-------------------
* [jsk_perception] slic as submodule
* Contributors: Ryohei Ueda

0.3.10 (2015-12-17)
-------------------
* [jsk_perception] Add utils to save images by request or from bagfile
  I sent PR to upstream:
  - https://github.com/ros-perception/image_pipeline/pull/159
  - https://github.com/ros-perception/image_pipeline/pull/163
  - https://github.com/ros-perception/image_pipeline/pull/164
  Added:
  jsk_perception/node_scripts/extract_images_sync
  jsk_perception/node_scripts/image_saver_sync
  jsk_perception/node_scripts/publish_header
* [jsk_pcl_ros] Check header.frame_id before resolving 3-D spacially
  Modified:
  jsk_pcl_ros/src/multi_plane_extraction_nodelet.cpp
  jsk_perception/src/polygon_array_color_histogram.cpp
  jsk_recognition_utils/include/jsk_recognition_utils/pcl_ros_util.h
  jsk_recognition_utils/src/pcl_ros_util.cpp
* Contributors: Kentaro Wada, Ryohei Ueda

0.3.9 (2015-12-14)
------------------
* [jsk_perception] Test slop with test_topic_published.py
  Depends on https://github.com/jsk-ros-pkg/jsk_common/pull/1254
* [jsk_perception] Specific test name for each test files
* [jsk_perception] test_topic_published.py does not work on hydro travis/jenkins
  Modified:
  jsk_perception/CMakeLists.txt
* [jsk_perception] Warn about segfault with large size image in SlicSuperpixel
  Modified:
  jsk_perception/src/slic_superpixels.cpp
* [jsk_perception] Test slic_super_pixels
* merge origin/master
* use shared_ptr for self_mask instance.
* Merge remote-tracking branch 'origin/master' into add-robot-mask
* [jsk_perception] Clean up duplicated packages in package.xml
* [jsk_perception] Compute polygon likelihood based on color histogram.
* [jsk_perception] Add PolygonArrayColorHistogram
* add sample launch file.
* add robot_to_mask source files.
* Contributors: Kentaro Wada, Masaki Murooka, Ryohei Ueda

0.3.8 (2015-12-08)
------------------
* [jsk_perception] Add CATKIN_ENABLE_TESTING if block
* Use ccache if installed to make it fast to generate object file
* [jsk_perception] Refactor publish_fixed_images.launch and fix test
* [jsk_perception] Test split_fore_background.py
* [jsk_perception] Fix header of split_fore_background
* [jsk_perception] Refactor publish_fixed_images.launch and fix test
* [jsk_perception] Specify encoding by rosparam in image_publisher.py
* [jsk_perception] Refactor image_publisher.py
* [jsk_perception] Fix supported encodings of split_fore_background.py
  It supports both 16UC1 and 32FC1.
* [jsk_perception] Fix supported encodings of split_fore_background.py
  It supports both 16UC1 and 32FC1.
* [jsk_perception] Add warnNoRemap in ``subscribe()``
* [split fore background] add conversion for depth image format 32FC1
* [jsk_perception] Set frame_id by rosparam
* [jsk_perception] Publish mask also in SplitForeBackground
* add applying blur to output image on edge detector
* [jsk_perception] Split FG/BG with local depth max
* Contributors: Kei Okada, Kentaro Wada, Shingo Kitagawa, Yohei Kakiuchi

0.3.7 (2015-11-19)
------------------
* Use gcc -z defs to check undefined symbols in shared
  objects (jsk_recognitoin_utils, jsk_pcl_ros, jsk_perception).
  build_check.cpp cannot run on the environment using  multiple processes
  because of invoking libjsk_pcl_ros.so link.
* Merge pull request `#1320 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1320>`_ from wkentaro/colorize_labels-with-146-colors
  [jsk_perception] ColorizeLabels support 20->146 labels
* [jsk_perception] ColorizeLabels support 20->146 labels
* [jsk_perception] Call onInitPostProcess() in last of onInit()
* [jsk_perception] Warn no remapping for input topics
* [jsk_perception] Test whether get topic msg
* [jsk_perception] FastRCNN: (new node)
* [jsk_perception] Test label image decomposer async
* [jsk_perception] Rename SimpleClassifier -> ScikitLearnClassifier
* [jsk_perception] Download trained_data for apc recognition sample
* [jsk_perception] Sort build_depend & run_depend
* [jsk_perception] Publish VectorArray in simple_classifier
* [jsk_perception] Publish VectorArray in bof_histogram_extractor
* [jsk_perception] Convert mask to label image
* [jsk_perception] Convert mask to label image
* [jsk_perception] Make connection based and use ClassificationResult.msg
* [jsk_perception] Care about data size when creating bof data
* [jsk_perception] Specify data size when creating bof data
* [jsk_perception] Update BoF object recognition sample
* [jsk_perception] Extract bof histogram with ConnectionBasedTransport
* [jsk_perception] Create bof & bof_hist dataset
* [jsk_perception] Creating sift dataset script
* [jsk_perception] Move ros node scripts/ -> node_scripts/
  Closes `#1239 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1239>`_
* Merge pull request `#1236 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1236>`_ from wkentaro/slop-param
  [jsk_perception] slop as param for label_image_decomposer
* Merge pull request `#1235 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1235>`_ from wkentaro/skip-0-label-image-decomposer
  [jsk_perception] Skip 0 label in label_image_decomposer
* [jsk_perception] slop as param for label_image_decomposer
* [jsk_perception] Skip 0 label in label_image_decomposer
* [jsk_perception] Debug output about params
* [jsk_perception] Add LabelImageDecomposer
* [jsk_perception] Rename tile_images -> tile_image
* [jsk_perception] Use ConnectionBasedTransport and get_tile_image()
* [jsk_perception/point_pose_extractor] Remove pragma message in compiling
  and fix format warning
* add oriented_gradient and oriented_gradient_node to install target and export libraries
* [jsk_perception] Add tile_images.py
* Contributors: Hiroaki Yaguchi, Kei Okada, Kentaro Wada, Ryohei Ueda

0.3.6 (2015-09-11)
------------------

0.3.5 (2015-09-09)
------------------

0.3.4 (2015-09-07)
------------------
* Swap doc soft links (to make 'Edit on GitHub' work)
* ColorizeFloatImage correct image link
  Closes https://github.com/jsk-ros-pkg/jsk_recognition/issues/1165
* Contributors: Kentaro Wada

0.3.3 (2015-09-06)
------------------
* [jsk_perception] README.md -> readthedocs.org
* Revert "[jsk_perception] use sphinx for rosdoc"
  This reverts commit 9e4ba233599b21c6422ec9a45f395b460c53264d.
* [jsk_perception/TabletopColorDifferenceLikelihood] Use geo/polygon.h
  instead of geo_util.h
* Contributors: Kentaro Wada, Ryohei Ueda

0.3.2 (2015-09-05)
------------------
* [jsk_perception] Ignore autogenerated files
* [jsk_perception] Use histograms to compute distance in TabletopColorDifferenceLikelihood
* Contributors: Ryohei Ueda

0.3.1 (2015-09-04)
------------------
* [jsk_pcl_ros, jsk_perception] Fix dependency of jsk_recognition_utils for child packages
  like jsk_rviz_plugins
* Contributors: Ryohei Ueda

0.3.0 (2015-09-04)
------------------
* [jsk_perception/CMakeLists.txt] set ROS_PACKAGE_PATH before run roseus using package://
* [jsk_recognition_utils] Introduce new package jsk_recognition_utils in order to use utility libraries defined in jsk_pcl_ros in jsk_perception
* Contributors: Kei Okada, Ryohei Ueda

0.2.18 (2015-09-04)
-------------------
* [jsk_perception] Do not specify sexp from cmake, just write in file
* [jsk_perception] Add .gitignore about auto-generated files
* [jsk_perception] Add template directory to run eusmodel_template_gen.l correctly
* [jsk_perception] Add PolygonArrayToLabelImage nodelet
* [jsk_perception] Move matchtemplate.py from src to scripts
* [jsk_perception] Move eusmodel_template_gen.l location from src to euslisp
* [jsk_perception] Do not download trained data in compilation time and
  add script to donload them
* [jsk_perception] use sphinx for rosdoc
* Revert "[jsk_perception] Add rosdoc.yaml to overwrite default file_patterns"
* [package.xml] Updatae Author
* [jsk_perception] use README.md as mainpage.doc
* [jsk_perception] Add rosdoc.yaml to overwrite default file_patterns
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda

0.2.17 (2015-08-21)
-------------------

0.2.16 (2015-08-19)
-------------------
* [CMakeLists.txt] we can not use rospack within cmake process
* Contributors: Kei Okada

0.2.15 (2015-08-18)
-------------------
* Merge pull request `#1058 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/1058>`_ from garaemon/uncomment-generate-template
  Uncomment generate template
* [jsk_perception] Add executable flag to eusmodel_template_gen.l
* [jsk_perception] uncomment generate template
* Contributors: JSK-PR2, Ryohei Ueda

0.2.14 (2015-08-13)
-------------------
* [jsk_perception] pub posewithcovariancestamped
* [jsk_perception] Add nodelet ColorizeFloatImage to colorize generic float image
* sliding_window_object_detector : opencv3 has different API for cv::ml::SVM
* src/virtual_camera_mono: use cv.hpp and opencv2 code for cv::getPerspectiveTransform
* src/snake_segmentation: snake (legacy.hpp) is disabled on opencv3
* src/point_pose_extractor: use cv.hpp
* linemode is moved to opencv_contrib, disabled for now (only for opencv3)
* src/calc_flow.cpp: use cv.hpp instead of cv.h
* background_substraction: cv::BackgroundSubtractorMOG2 is abstract type for opencv3
* CMakeLists.txt: depends on cv_bridge, not opencv (jsk_perception)
* [jsk_perception] Update readme
* [jsk_perception] Add simple_classifier*
* [jsk_perception] Scripts for bof and its hist extractor
* do not convert image encode in kmeans and gaussian_blur
* Contributors: Kei Okada, Kentaro Wada, Ryohei Ueda, Hitoshi Kamada, Masaki Murooka

0.2.13 (2015-06-11)
-------------------
* [jsk_perception] Use dynamic_reconfigure in ImageTimeDiff
* [jsk_perception] Update image_time_diff to use hue/saturation
* [jsk_perception] Add Kmeans section to README
* [jek_perception] Add kmeans
* [jsk_perception] Add GaussignBlur section to README
* [jsk_perception] Add gaussian_blur
* [jsk_perception] Update README.md for squashing dilate/erode
* [jsk_perception] Squash erode/dilate to morphological_operator
* [jsk_perception] Update README.md for morphological operators
* [jsk_perception] Add advanced morphological transformations
* [jsk_perception] Use isBGR/isRGB/isBGRA/isRGBA in ApplyMaskImage
* [jsk_perception] Add isBGR/isRGB/isBGRA/isRGBA
* [jsk_perception] Use header to synchronize in ImageTimeDiff
* [jsk_perception] Update image_time_diff.py to use ImageDifferenceValue.msg
* [jsk_perception] Update docs of image_time_diff for output
* [jsk_perception] Publish with stamp in image_time_diff
* [jsk_perception/image_publisher] Do not exit program even though no file is found
* uncomment camera_info_cb
* add subscription of image_raw
* Updated Sliding window detector.
  - Removed the trainer
  - Added Bootstraper
* [jsk_perception] Update README for #927
* [jsk_perception] Enable apply_mask convert mask black to transparent
* Changed from reading saved image from directory to RosBag files
* [jsk_perception] Use jsk_topic_tools/log_utils.h for JSK_ROS_INFO,
  JSK_NODELET_INFO and so on
* [jsk_perception] add diff per pixel to ImageTimeDiff
* [jsk_perception] Fix bug in apply_mask in converting BGRA/RGBA input image
* [jsk_perception] remove no need get_param in image_publisher
* [jsk_perception] Enable HSVDecomposer to handle BGRA/RGBA image
* [jsk_perception] Enable ApplyMask handle BGRA/RGBA image
* [jsk_perception] ApplyMask Mono8 encoding to publish mask
* [jsk_perception] Add publish_info param to image_publisher
* [jsk_perception] Add dynamic_reconfigure feature to ImagePublisher
* [jsk_perception] Publish the difference between start and current image
* [jsk_perception][ApplyMaskImage] mask image should be mono8
* Node to for training the classifier for Sliding Window Object Detector
* [jsk_perception] Ignore trained_data directory from git filesystem
* Contributors: Kentaro Wada, Ryohei Ueda, Eisoku Kuroiwa, Krishneel Chaudhary

0.2.12 (2015-05-04)
-------------------
* Revert "[jsk_perception/point_pose_extractor] Use OpenCV's matcher class to estimate mathcing"
* [jsk_perception/point_pose_extractor] Use OpenCV's matcher class to
  estimate mathcing
* [jsk_perception/point_pose_extractor] Add license header
* [jsk_perception] Untabify point_pose_extractor.cpp
* [jsk_perception/point_pose_extractor] Publish PoseStamped from
  point_pose_extractor result
* add ROS_INFO
* [jsk_perception] check if pcam.intrinsicMatrix is valid
* [jsk_perception] Download drill trained data in compiling time
* Removed opencv non-free header directive
  Corrected the nodelet name in CMakeLists.txt
* Corrected the nodelet name in CMakeLists.txt
* Removed opencv non-free header directive
* Nodelet for Edge, Contour Thinning and Nodelet for Sliding window object detector
* [jsk_perception] add Fisheye Rotate parameter
* add upside down option to cfg
* add Fisheye Ray Publisher
* [jsk_perception] Add ProjectImagePoint nodelet to project image local
  coordinates into 3-D point
* [jsk_perception] Update README for fisheye
* [jsk_perception] update Fisheye To Panoarama
* [jsk_perception] Modify typo
* [jsk_perception] Add MaskImageGenerator
* add scale command to shrink the output and make faster
* add cfg
* [jsk_perception] Add fisheye rectify
* [jsk_perception] Add attributeError message to image_publisher.py
* [jsk_perception] Fix README.md about erode/dilate nodelets
* Merge pull request #834 from wkentaro/update-readme-for-pr-811
  [jsk_perception] Update README for histogram max_value of SingleChannelHistogram
* [jsk_perception] Update README for histogram max_value of SingleChannelHistogram
* [jsk_perception] Update README for iterations param of Dilate/ErodeMaskImage
* [jsk_perception] Add iteration param to DilateMaskImage & ErodeMaskImage
* Contributors: Kamada Hitoshi, Kentaro Wada, Ryohei Ueda, Yuto Inagaki, iKrishneel

0.2.11 (2015-04-13)
-------------------
* add encoded points rate
* Contributors: Kamada Hitoshi

0.2.10 (2015-04-09)
-------------------
* [jsk_perception] add Simple Fisheye to Panorama
* [jsk_perception] changed order of dynamic reconfigure
* [jsk_perception] default max value of histogram should be 256 to include 255 pixel
* [jsk_perception] print number of point when encoding sparse image
* [jsk_perception] Publish empty camera info from image_publisher.py
* [jsk_perception] Add sample for ColorHistogramLabelMatch
* [jsk_perception] Add documentation about ColorHistogramLabelMatch
* Contributors: Yuki Furuta, Ryohei Ueda, Yuto Inagaki, Kamada Hitoshi, Kentaro Wada

0.2.9 (2015-03-29)
------------------
* 0.2.8
* Update Changelog
* Contributors: Ryohei Ueda

0.2.8 (2015-03-29)
------------------

0.2.7 (2015-03-26)
------------------

0.2.6 (2015-03-25)
------------------

0.2.5 (2015-03-17)
------------------
* check target cloud data ifnot invalid
* Validate image message without image array (width == 0 and height == 0)
* Enhance: more specific error exception
* Change to avoid SyntaxWarning about not assigning rospy.Publisher argument queue_size
* Change import libs with reasonable order (thirdparty -> ros)
* Contributors: Kentaro Wada, Yu Ohara

0.2.4 (2015-03-08)
------------------
* [jsk_perception] Add simple script to publish image file into ros image
* Fix license: WillowGarage -> JSK Lab
* Contributors: Ryohei Ueda

0.2.3 (2015-02-02)
------------------
* [jsk_pcl_ros, jsk_perception] Move mask image operation to jsk_perception
* Remove rosbuild files
* [jsk_perception] Add ErodeMaskImage nodelet
* [jsk_perception] Add DilateMaskImage
* Contributors: Ryohei Ueda

0.2.2 (2015-01-30)
------------------
* [jsk_perception] add posedetection_msgs
* add image_view2 to depends
* Contributors: Kei Okada

0.2.1 (2015-01-30)
------------------
* add image_view2 to depends

0.2.0 (2015-01-29)
------------------

0.1.34 (2015-01-29)
-------------------
* [jsk_perception, checkerboard_detector] Remove dependency to jsk_pcl_ros
* [jsk_pcl_ros, jsk_perception] Move find_object_on_plane from
  jsk_perception to jsk_pcl_ros to make these packages independent
* [jsk_pcl_ros, jsk_perception] Use jsk_recognition_msgs
* [jsk_pcl_ros, jsk_perception, resized_image_transport] Do not include
  jsk_topic_tools/nodelet.cmake because it is exported by CFG_EXTRAS
* [imagesift] Better support of masking image:
  1) Use jsk_perception::boundingRectOfMaskImage to compute ROI
  2) support mask image in imagesift.cpp to make better performance
* [jsk_perception] Export library
* [jsk_perception] Do not use cv::boundingRect to compute bounding box of
  mask image
* [jsk_perception] install include directory of jsk_perception
* Contributors: Ryohei Ueda

0.1.33 (2015-01-24)
-------------------
* [jsk_perception] FindObjectOnPlane: Find object on plane from 2d binary
  image and 3-d polygon coefficients
* [jsk_perception] Publish convex hull image of mask from ContourFinder
* [jsk_perception] Fix min_area parameter to work in BlobDetector
* [jsk_pcl_ros, jsk_perception] Fix CmakeList for catkin build. Check jsk_topic_tools_SOURCE_PREFIX
* [jsk_perception] Add MultiplyMaskImage
* [jsk_perception] Add ~approximate_sync parameter to toggle
  exact/approximate synchronization
* [jsk_perception] Add UnapplyMaskImage
* [jsk_perception] Add blob image to document
* [jsk_perception] Add BlobDetector
* [jsk_perception] Colorize label 0 as black because label-0 indicates
  masked region
* [jsk_perception] AddMaskImage to add two mask images into one image
* [jsk_perception] Increase label index of SLICSuperPixels to avoid 0. 0
  is planned to be used as 'masked'
* [jsk_perception] Publish result binary image as mono image from ColorHistogramMatch
* [jsk_perception] Extract mask image from coefficients of histogram
  matching in ColorHistogramLabelMatch
* [jsk_perception] Publish result of coefficient calculation as float image
* [jsk_perception] Support mask image in ColorHistogramLabelMatch
* [jsk_perception] Use OpenCV's function to normalize histogram and add
  min and max value of histogram in ColorHistogramLabelMatch
* [jsk_perception] Add ~min_value and ~max_value to SingleChannelHistogram
* [jsk_perception] SingleChannelHistogram to compute histogram of single
  channel image
* [jsk_perception] Add YCrCb decomposer
* [jsk_perception] Add LabDecomposer to decompose BGR/RGB image into Lab
  color space
* [jsk_perception] Use cv::split to split bgr and hsv image into each channel
* [jsk_perception] Fix metrics of ColorHistogramLabelMatch:
  1) correlation
  original value is [-1:1] and 1 is perfect. we apply (1 - x) / 2
  2) chi-squared
  original value is [0:+inf] and 0 is perfect. we apply 1 / (1 + x^2)
  3) intersect
  original value is [0:1] and 1 is perfect. we apply x
  4) bhattacharyya
  original value is [0:1] and 0 is perfect. we apply 1 - x
  5, 6) EMD
  original value is [0:+inf] and 0 is perfect. we apply 1 / (1 + x^2)
* [jsk_perception] Publish more useful debug image from SLICSuperPixels
  and add documentation.
* [jsk_perception] Publish image of interest from ColorHistogram
* [jsk_perception] Implement 6 different method to compute coefficients
  between two histograms
* [jsk_perception] Increase the maximum number of super pixels
* [jsk_perception] Fix ColorHistogram minor bags:
  1. Support rect message out side of image
  2. Use mask image in HSV histogram calculation
* [jsk_perception] Fix HSVDecomposer color space conversion: support RGB8
* [jsk_perception] color matching based on histogram and label information
* [jsk_perception] Add utlity to visualize mask image: ApplyMaskImage
* [jsk_perception] Add GridLabel
* [jsk_perception] Publish hisotgram messages under private namespace
* [jsk_perception] Add simple launch file as sample of superpixels
* [jsk_perception] Utility to colorize labels of segmentation
* [jsk_perception] Fix SLICSuperPixels:
  1) if input image if BGR8
  2) transpose the result of clustering
* [jsk_perception] Publish segmentation result as cv::Mat<int> and use
  patched version of SLIC-SuperPixels to get better performance
* [jsk_perception] Support RGB8 and gray scale color in SLICSuperPixels
* [jsk_perception] Add dynamic_reconfigure interface to SLICSuperPixels
* [jsk_perception] Separate SLICSuperPixels into header and cpp files
* [jsk_perception] Publish result of segmentation of slic superpixels as image
* [jsk_perception] Add snake segmentation
* [jsk_perception] ContourFinder
* [jsk_perception] Support one-channel image in GrabCut
* [jsk_perception] HSVDecomposer to decompose RGB into HSV separate images
* [jsk_perception] Add RGBDecomposer to decompose RGB channels into
  separate images
* Contributors: Ryohei Ueda

0.1.32 (2015-01-12)
-------------------

0.1.31 (2015-01-08)
-------------------
* [jsk_perception] Add parameter to select seed policy (definitely
  back/foreground or probably back/foreground) to GrabCut
* adapt attention-clipper for fridge demo
* [jsk_perception] Publish mask image of grabcut result
* [jsk_perception] add GrabCut nodelet
* Remove roseus from build dependency of jsk_perception
* added debug pub

0.1.30 (2014-12-24)
-------------------

0.1.29 (2014-12-24)
-------------------
* added some more parameters for detection
* Contributors: Yu Ohara

0.1.28 (2014-12-17)
-------------------
* added param to set threshold of best_Windoq
* Add dynamic reconfigure to background substraction
* Clean up background substraction codes
* Add background substraction
* Support image mask in ColorHistogram
* Separate header and cpp file of color_hisotgram
* Use jsk_topic_tools::DiagnosticNodelet for color histogram
* Fix coding style of color_histogram
* Fix indent of linemod.cpp
* Add linemod sample
* changed color_histogram_matcher to pub box_array defined in jsk_pcl_ros

0.1.27 (2014-12-09)
-------------------
* added some algolism to get best window
* changed codes to pub center of object
* matchedPointPub by 2dResult of colorhistogram matching
* changed color_histogram_sliding_matcher and added launch to show result
* Contributors: Yu Ohara

0.1.26 (2014-11-23)
-------------------

0.1.25 (2014-11-21)
-------------------
* kalmanfilter
* changed name
* added codes in catkin.cmake
* added cfg
* added color_histogram_mathcer_node

0.1.24 (2014-11-15)
-------------------
* servicecall
* Use intrinsicMatrix instead of projectionMatrix to specify 3x3 matrix(K)
  instead of 4x3 matrix(P)
* remove eigen and add cmake_modules to find_package for indigo
* fix: use projectionMatrix() for indigo
* Add script to setup training assistant for opencv-like dataset
* Add script to check opencv cascade file
* Script to reject positive data for OpenCV training
* renamed only-perception.launch
* calc existance probability
* removed kalmanlib.l from jsk_perception
* add kalman-filter library
* Contributors: Ryohei Ueda, Hitoshi Kamada, Kei Okada, Kamada Hitoshi

0.1.23 (2014-10-09)
-------------------
* Install nodelet executables
* mend spell-miss in launch
* modified program to select which camera_info to sub
* renamed camera_node to uvc_camera_node, and added some options
* modified detection-interface.l
* Contributors: Ryohei Ueda, Kamada, Yu Ohara

0.1.22 (2014-09-24)
-------------------
* Disable ssl when calling git
* Contributors: Ryohei Ueda

0.1.21 (2014-09-20)
-------------------
* Add more diagnostics to OrganizedMultiPlaneSegmentation and fix global
  hook for ConvexHull
* Contributors: Ryohei Ueda

0.1.20 (2014-09-17)
-------------------

0.1.19 (2014-09-15)
-------------------

0.1.18 (2014-09-13)
-------------------
* add git to build_depend of jsk_libfreenect2
* Contributors: Ryohei Ueda

0.1.17 (2014-09-07)
-------------------
* add mk/git to build_depend
* Contributors: Kei Okada

0.1.16 (2014-09-04)
-------------------
* do not use rosrun in the script of jsk_perception/src/eusmodel_template_gen.sh
* Contributors: Ryohei Ueda

0.1.14 (2014-08-01)
-------------------

0.1.13 (2014-07-29)
-------------------

0.1.12 (2014-07-24)
-------------------
* fix to use catkin to link rospack
* Contributors: Kei Okada, Dave Coleman

0.1.11 (2014-07-08)
-------------------
* jsk_perception does not depends on pcl, but depends on eigen and tf
* Contributors: Ryohei Ueda

0.1.10 (2014-07-07)
-------------------
* adding oriented_gradient_node
* add calc_flow program to calc optical flow
* Contributors: Ryohei Ueda, Hiroaki Yaguchi

0.1.9 (2014-07-01)
------------------

0.1.8 (2014-06-29)
------------------
* initialize _img_ptr at first
* convert color image to GRAY
* add nodelet to detect circles based on hough transformation
* add program to compute color histogram (rgb and hsv color space)
* maked configure_file to create imagesurf, imagestar and imagebrisk automatically
* added the programs to use cv_detection
* Contributors: Ryohei Ueda, Yusuke Furuta, Yu Ohara

0.1.7 (2014-05-31)
------------------

0.1.6 (2014-05-30)
------------------

0.1.5 (2014-05-29)
------------------
* add service interface with sensor_msgs/SetCameraInfo to camshiftdemo, not only mouse selection.
* Contributors: Ryohei Ueda

0.1.4 (2014-04-25)
------------------

* add sparse_image program to jsk_percepton
* make edge_detector nodelet class
* Contributors: Ryohei Ueda, Yuki Furuta
* Merge pull request `#47 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/47>`_ from k-okada/add_rosbuild
* Contributors: Kei Okada

0.1.3 (2014-04-12)
------------------

0.1.2 (2014-04-11)
------------------

0.1.1 (2014-04-10)
------------------
* catkinize jsk_perception
* check initialization in check_subscribers function
* change callback function names for avoiding the same name functions
* add edge_detector.launch
* change debug message
* rename type -> atype
* fix minor bug
* change for treating multiple objects in one ObjectDetection.msg
* add test programs
* add rosbuild_link_boost for compile on fuerte/12.04 , see Issue `#224 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/224>`_, thanks tnakaoka
* add rectangle_detector, based on http://opencv-code.com/tutorials/automatic-perspective-correction-for-quadrilateral-objects/
* update hoguh_lines
* use blur before canny
* add image_proc modules from opencv samples
* change error_threshold max 200 -> 2000
* add :detection-topic keyword to (check-detection)
* replace sleep to :ros-wait for making interruptible
* add scripts for speaking english
* speak before sleep
* add to spek we're looking for...
* print out debug info
* turtlebot/ros pdf
* add ros/turtlebot-logo images `#173 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/173>`_
* update japanese speaking
* modify parameter definition. parameter should not be overwritten.
* add option publish-objectdetection-marker
* add slot :diff-rotation in detection_interface.l
* do not create ros::roseus object by load detection_interface.l
* publish tf from sensor frame to detected object pose
* update objectdetection-marker program for new detection_interface
* publish tf and markers, add messages
* print out error value
* fix segfault
* suppor rpy style in relative_pose, status:closed `#139 <https://github.com/jsk-ros-pkg/jsk_recognition/issues/139>`_
* add :target-object keyword to check-detection
* fix : project3dToPixel was removed in groovy
* update to use cv_bridge
* fix for groovy, use cv_bridge not CvBridge
* fix: speak content
* fix: speak-jp
* fix template location
* add microwave detection sample
* add speak-name for speaking japanease object name
* add speak words
* update detction_interface.l for single detection and speak flag
* add solve-tf parameter for not using tf
* add frame_id for coordinates
* add detection_interface.l for using point_pose_extractor
* remove euclidean_cluster,plane_detector and color_extractor from jsk_perception, they are supported in tabletop and pcl apps should go into jsk_pcl_ros
* add max_output
* add opencv2 to rosdep.yaml for compatibility
* update to fit opencv2 electric/fuerte convention
* fix for fuerte see https://code.ros.org/trac/ros/ticket/3955
* add size check
* fix btVector3 -> tf::Vector3
* fix remove define KdTreePtr
* fix style: support ROSPACK_API_V2 (fuerte)
* support ROSPACK_API_V2 (fuerte)
* fix for pcl > 1.3.0, pcl::KdTree -> pcl::search::KdTree, pcl::KdTreeFLANN -> pcl::search::KdTree
* remove explicit dependency to eigen from jsk_perception
* add whilte_balance_param.yaml
* add publish_array for publishing pointsarray
* move posedetectiondb/SetTemplate -> jsk_perception/SetTemplate
* add color_extractor, plane_detector, euclidean_clustering for jsk_perception
* fixed the package name of WhiteBalance.srv
* add eigen to dependency
* add white_balance_converter to jsk_perception
* change msg from face_detector_mono/Rect -> jsk_perception/Rect. I couldn't find set_serch_rect string under jsk-ros-pkg
* node moved from virtual_camera
* check if the matched region does not too big or too small
* add dynamic reconfigure for point_pose_extractor
* split launch for elevator_navigation, to test modules
* fix for oneiric
* fix for users who does not have roseus in their PATH
* ns can't be empty string in launch xml syntax
* commit updates for demo
* added tv-controller with ut logo
* added tv-controller with ut logo
* fixed the size of wrap image, which is calcurated from input (width/height)
* add to write wrapped image
* add error handling and output template file
* add opencv-logo2.png
* add lipton milktea model, auto generated file prefix .launch -> .xml to avoid listed by auto complete
* add sharp rimokon with ist logo
* changed variable name client -> clients
* add sharp tv controller to sample
* add sample for detection launcher generator
* use try to catch assertions
* set Zero as distortionMatrix, because ImageFeature0D.image is rectified
* fixed the box pose in debug image
* changed code for generate SIFT template info
* use projectionMatrix instead of intrinsicMatrix in solvePnP, remove CvBridge -> cv_bridge
* fix to work without roseus path in PATH
* fix relative pose, object coords to texture coords
* update generation script of SIFT pose estimation launcher, relative pose is not correct
* update eusmodel->sift_perception script
* change detection launch generation script to use jsk_perception/point_pose_extractor
* add std namespace appropriately
* update initialize template method
* publish the debug_image of point_pose_extractor
* chnage the output frame id when using only one template
* change threashold for detectiong object
* use /ObjectDetection_agg instead of /ObjectDetection
* add _agg output topic for debug and logging
* add debug message, set lifetime to 1 sec
* add objectdetection-marker.l
* add relative pose parameter to point_pose_extractor.cpp
* change the PutText region
* update sample launch file, point pose extractor do not subscribe input topics when output is not subscribed
* add viewer_window option to disable the OpenCV window
* empty window name to disable window, point_pose_extractor
* move posedetectiondb to jsk_visioncommon
* moved jsk_vision to jsk_visioncommon
* Contributors: Haseru Chen, Kazuto Murai, Youhei Kakiuchi, Yuki Furuta, Kei Okada, Yuto Inagaki, Manabu Saito, Rosen Dinakov, HiroyukiMikita
