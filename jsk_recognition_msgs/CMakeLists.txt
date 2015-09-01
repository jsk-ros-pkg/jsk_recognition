cmake_minimum_required(VERSION 2.8.3)
project(jsk_recognition_msgs)
find_package(catkin REQUIRED
  std_msgs sensor_msgs geometry_msgs message_generation pcl_msgs)
add_message_files(
  FILES
  PlotData.msg
  HeightmapConfig.msg
  SimpleOccupancyGrid.msg
  SimpleOccupancyGridArray.msg
  PolygonArray.msg
  ClusterPointIndices.msg
  ParallelEdgeArray.msg
  ParallelEdge.msg
  TimeRange.msg
  Rect.msg
  RectArray.msg
  SparseOccupancyGridCell.msg
  Circle2DArray.msg
  BoundingBox.msg
  SparseOccupancyGridColumn.msg
  SparseOccupancyGrid.msg
  Histogram.msg
  HistogramWithRange.msg
  HistogramWithRangeBin.msg
  ColorHistogram.msg
  DepthErrorResult.msg
  BoundingBoxArray.msg
  PointsArray.msg
  Torus.msg
  SlicedPointCloud.msg
  LineArray.msg
  PosedCameraInfo.msg
  TorusArray.msg
  RotatedRectStamped.msg
  ModelCoefficientsArray.msg
  RotatedRect.msg
  DepthCalibrationParameter.msg
  Int32Stamped.msg
  BoundingBoxMovement.msg
  SnapItRequest.msg
  Circle2D.msg
  SparseOccupancyGridArray.msg
  SparseImage.msg
  ColorHistogramArray.msg
  Line.msg
  ICPResult.msg
  ImageDifferenceValue.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs sensor_msgs geometry_msgs pcl_msgs
)
catkin_package(
  CATKIN_DEPENDS std_msgs sensor_msgs geometry_msgs pcl_msgs
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})