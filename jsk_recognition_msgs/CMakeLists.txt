# $ENV{ROS_VERSION} is set by ros_environment, which is missing on
# pre-ros_environment distros (e.g. indigo) and on ROS 2 builds that do not
# depend on it (e.g. the buildfarm's binary jobs). Guess it then: ament sets
# AMENT_PREFIX_PATH (bloom's debian/rules passes it to cmake, setup.sh and
# colcon set the environment variable), catkin never does.
if("$ENV{ROS_VERSION}" STREQUAL "")
  if(DEFINED AMENT_PREFIX_PATH OR DEFINED ENV{AMENT_PREFIX_PATH})
    set(ENV{ROS_VERSION} 2)
  else()
    set(ENV{ROS_VERSION} 1)
  endif()
  message(WARNING "ROS_VERSION is not set, assuming ROS_VERSION=$ENV{ROS_VERSION}")
endif()

if("$ENV{ROS_VERSION}" EQUAL 2)
  cmake_minimum_required(VERSION 3.8)
  project(jsk_recognition_msgs)

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  set(_deps builtin_interfaces std_msgs sensor_msgs geometry_msgs pcl_msgs jsk_footstep_msgs)
  foreach(_dep ${_deps})
    find_package(${_dep} REQUIRED)
  endforeach()

  file(GLOB _msgs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS msg/*.msg)
  file(GLOB _srvs RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS srv/*.srv)
  file(GLOB _actions RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS action/*.action)

  # ExifTags.msg has a field named 'ycb_cr_Coefficients'; rosidl rejects
  # field names containing uppercase letters and renaming it would change the
  # ROS1 md5sum, so it is not available on ROS2.
  list(REMOVE_ITEM _msgs msg/ExifTags.msg)

  # 'time'/'duration' are ROS1 builtins; ROS2 uses builtin_interfaces/Time and
  # builtin_interfaces/Duration. Render ROS2 copies of such files in the build
  # directory so that the ROS1 definitions stay untouched.
  set(_ros2_interface_dir "${CMAKE_CURRENT_BINARY_DIR}/ros2_interfaces")
  set(_ros2_interfaces)
  foreach(_file msg/TimeRange.msg)
    list(REMOVE_ITEM _msgs ${_file})
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/${_file}" _content)
    string(REGEX REPLACE "(^|\n)([ \t]*)time([ \t\\[])" "\\1\\2builtin_interfaces/Time\\3" _content "${_content}")
    string(REGEX REPLACE "(^|\n)([ \t]*)duration([ \t\\[])" "\\1\\2builtin_interfaces/Duration\\3" _content "${_content}")
    file(WRITE "${_ros2_interface_dir}/${_file}" "${_content}")
    set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${_file}")
    list(APPEND _ros2_interfaces "${_ros2_interface_dir}:${_file}")
  endforeach()

  rosidl_generate_interfaces(${PROJECT_NAME}
    ${_msgs} ${_ros2_interfaces} ${_srvs} ${_actions}
    DEPENDENCIES ${_deps}
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()
else()
cmake_minimum_required(VERSION 2.8.3)
project(jsk_recognition_msgs)
find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs jsk_footstep_msgs)
add_message_files(
  FILES
  Accuracy.msg
  BoundingBoxArray.msg
  BoundingBoxMovement.msg
  BoundingBox.msg
  BoundingBoxArrayWithCameraInfo.msg
  Circle2DArray.msg
  Circle2D.msg
  ClassificationResult.msg
  ClipResult.msg
  ClusterPointIndices.msg
  ColorHistogramArray.msg
  ColorHistogram.msg
  DepthCalibrationParameter.msg
  DepthErrorResult.msg
  ExifTags.msg
  ExifGPSInfo.msg
  HeightmapConfig.msg
  Histogram.msg
  HistogramWithRangeBin.msg
  HistogramWithRange.msg
  HistogramWithRangeArray.msg
  HumanSkeleton.msg
  HumanSkeletonArray.msg
  ICPResult.msg
  ImageDifferenceValue.msg
  Int32Stamped.msg
  Label.msg
  LabelArray.msg
  LineArray.msg
  Line.msg
  ModelCoefficientsArray.msg
  Object.msg
  ObjectArray.msg
  PanoramaInfo.msg  #  added on jsk_recognition_msgs until https://github.com/ros/common_msgs/pull/171 merged
  ParallelEdgeArray.msg
  ParallelEdge.msg
  PlotData.msg
  PointsArray.msg
  PolygonArray.msg
  PosedCameraInfo.msg
  QueryAndProbability.msg
  QuestionAndAnswerText.msg
  RectArray.msg
  Rect.msg
  RotatedRect.msg
  RotatedRectStamped.msg
  SimpleHandle.msg
  SimpleOccupancyGridArray.msg
  SimpleOccupancyGrid.msg
  SlicedPointCloud.msg
  SnapItRequest.msg
  SparseImage.msg
  SparseOccupancyGridArray.msg
  SparseOccupancyGridCell.msg
  SparseOccupancyGridColumn.msg
  SparseOccupancyGrid.msg
  Spectrum.msg
  TimeRange.msg
  TorusArray.msg
  Torus.msg
  TrackerStatus.msg
  TrackingStatus.msg
  BoolStamped.msg
  VectorArray.msg
  VQAResult.msg
  WeightedPoseArray.msg
  ContactSensor.msg
  ContactSensorArray.msg
  PlotDataArray.msg
  Segment.msg
  SegmentStamped.msg
  SegmentArray.msg
  PeoplePose.msg
  PeoplePoseArray.msg
  HandPose.msg
  HandPoseArray.msg
  )

# TODO(wkentaro): Most of srv files are duplicated in jsk_pcl_ros,
#   and jsk_perception, so those in the packages should be removed.
add_service_files(FILES
  CallPolygon.srv
  CallSnapIt.srv
  CheckCircle.srv
  CheckCollision.srv
  EnvironmentLock.srv
  EuclideanSegment.srv
  ICPAlign.srv
  ICPAlignWithBox.srv
  NonMaximumSuppression.srv
  PolygonOnEnvironment.srv
  RobotPickupReleasePoint.srv
  SaveMesh.srv
  SetDepthCalibrationParameter.srv
  SetLabels.srv
  SetPointCloud2.srv
  SetTemplate.srv
  SnapFootstep.srv
  SwitchTopic.srv
  TowerPickUp.srv
  TowerRobotMoveCommand.srv
  TransformScreenpoint.srv
  UpdateOffset.srv
  WhiteBalancePoints.srv
  WhiteBalance.srv
  )

add_action_files(FILES
  VQATask.action
  ClassificationTask.action
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs pcl_msgs jsk_footstep_msgs actionlib_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs pcl_msgs jsk_footstep_msgs
)

# From https://github.com/jsk-ros-pkg/jsk_recognition/pull/2345
# Install header files directly into ${CATKIN_PACKAGE_INCLUDE_DESTINATION}.
# If the package has setup.py and modules under src/${PROJECT_NAME}/,
# install python executables directly into ${CATKIN_PACKAGE_BIN_DESTINATION}.
# However, if it doesn't have setup.py, directories including python executables
# should be installed recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Also, other directories like 'launch' or 'sample' must be installed
# recursively into ${CATKIN_PACKAGE_SHARE_DESTINATION}.
# Be careful that 'launch' and 'launch/' are different: the former is directory
# and the latter is each content.
install(DIRECTORY sample
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if($ENV{ROS_DISTRO} STREQUAL "indigo") # on noetic it needs catkin_install_python to support Python3 and it does not work on indigo for some reason...
  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    )
else()
  install(DIRECTORY scripts
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    USE_SOURCE_PERMISSIONS
    PATTERN "*"
    PATTERN "*/*.py" EXCLUDE
    )

  file(GLOB SCRIPT_PROGRAMS scripts/*.py)
  catkin_install_python(
  PROGRAMS ${SCRIPT_PROGRAMS}
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
  )
endif()

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/object_array_publisher.test)
  add_rostest(test/people_pose_array_to_pose_array.test)
  add_rostest(test/plot_data_to_csv.test)
  add_rostest(test/save_mesh_server.test)
endif()
endif()
