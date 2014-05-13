^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package jsk_perception
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2014-04-25)
------------------

* add sparse_image program to jsk_percepton
* make edge_detector nodelet class
* Contributors: Ryohei Ueda, furushchev
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
* Contributors: HiroyukiMikita, Kei Okada, Haseru Chen, Yuki Furuta, Yuto Inagaki, Kazuto Murai, Manabu Saito, Rosen Dinakov, Youhei Kakiuchi
