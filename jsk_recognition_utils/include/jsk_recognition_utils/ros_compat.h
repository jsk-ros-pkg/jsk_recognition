// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// The sources of jsk_recognition_utils are shared by ROS 1 and ROS 2 and
// tell them apart with `#if ROS_VERSION_MAJOR != 1`. ROS_VERSION_MAJOR is
// defined by ros/common.h on ROS 1 and undefined (so 0) on ROS 2.
//
// On ROS 2 this header provides what the ROS 1 headers did for the part of
// the library that is built there:
// - the message headers (geometry_msgs/msg/polygon.hpp, ...) and aliases of
//   the ROS 1 type names (geometry_msgs::Polygon, ...) to the ROS 2 ones,
// - ROS_DEBUG/ROS_INFO/ROS_WARN/ROS_ERROR on the "jsk_recognition_utils"
//   rclcpp logger, unless they are defined already.

#ifndef JSK_RECOGNITION_UTILS_ROS_COMPAT_H_
#define JSK_RECOGNITION_UTILS_ROS_COMPAT_H_

#if defined(__has_include)
#if __has_include(<ros/common.h>)
#include <ros/common.h>
#endif
#else
#include <ros/common.h>
#endif

#if ROS_VERSION_MAJOR != 1

#include <rclcpp/logging.hpp>

// pulled in by ros/ros.h on ROS 1
#include <boost/array.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <jsk_recognition_msgs/msg/bounding_box.hpp>
#include <jsk_recognition_msgs/msg/bounding_box_array.hpp>
#include <jsk_recognition_msgs/msg/histogram_with_range_bin.hpp>
#include <jsk_recognition_msgs/msg/polygon_array.hpp>
#include <jsk_recognition_msgs/msg/simple_occupancy_grid.hpp>
#include <jsk_recognition_msgs/msg/sparse_occupancy_grid.hpp>
#include <jsk_recognition_msgs/msg/sparse_occupancy_grid_cell.hpp>
#include <jsk_recognition_msgs/msg/sparse_occupancy_grid_column.hpp>
#include <pcl_msgs/msg/model_coefficients.hpp>
#include <pcl_msgs/msg/point_indices.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/header.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define JSK_RECOGNITION_UTILS_MSG_ALIAS(pkg, type) \
  namespace pkg { using type = ::pkg::msg::type; }
JSK_RECOGNITION_UTILS_MSG_ALIAS(geometry_msgs, Point)
JSK_RECOGNITION_UTILS_MSG_ALIAS(geometry_msgs, Point32)
JSK_RECOGNITION_UTILS_MSG_ALIAS(geometry_msgs, Polygon)
JSK_RECOGNITION_UTILS_MSG_ALIAS(geometry_msgs, Pose)
JSK_RECOGNITION_UTILS_MSG_ALIAS(geometry_msgs, Transform)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, BoundingBox)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, BoundingBoxArray)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, HistogramWithRangeBin)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, PolygonArray)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, SimpleOccupancyGrid)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, SparseOccupancyGrid)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, SparseOccupancyGridCell)
JSK_RECOGNITION_UTILS_MSG_ALIAS(jsk_recognition_msgs, SparseOccupancyGridColumn)
JSK_RECOGNITION_UTILS_MSG_ALIAS(pcl_msgs, ModelCoefficients)
JSK_RECOGNITION_UTILS_MSG_ALIAS(pcl_msgs, PointIndices)
JSK_RECOGNITION_UTILS_MSG_ALIAS(sensor_msgs, CameraInfo)
JSK_RECOGNITION_UTILS_MSG_ALIAS(sensor_msgs, PointCloud2)
JSK_RECOGNITION_UTILS_MSG_ALIAS(sensor_msgs, PointField)
JSK_RECOGNITION_UTILS_MSG_ALIAS(std_msgs, ColorRGBA)
JSK_RECOGNITION_UTILS_MSG_ALIAS(std_msgs, Header)
JSK_RECOGNITION_UTILS_MSG_ALIAS(visualization_msgs, Marker)
#undef JSK_RECOGNITION_UTILS_MSG_ALIAS

#define JSK_RECOGNITION_UTILS_LOGGER rclcpp::get_logger("jsk_recognition_utils")
#ifndef ROS_DEBUG
#define ROS_DEBUG(...) RCLCPP_DEBUG(JSK_RECOGNITION_UTILS_LOGGER, __VA_ARGS__)
#endif
#ifndef ROS_INFO
#define ROS_INFO(...) RCLCPP_INFO(JSK_RECOGNITION_UTILS_LOGGER, __VA_ARGS__)
#endif
#ifndef ROS_WARN
#define ROS_WARN(...) RCLCPP_WARN(JSK_RECOGNITION_UTILS_LOGGER, __VA_ARGS__)
#endif
#ifndef ROS_ERROR
#define ROS_ERROR(...) RCLCPP_ERROR(JSK_RECOGNITION_UTILS_LOGGER, __VA_ARGS__)
#endif

#endif  // ROS_VERSION_MAJOR != 1

#endif  // JSK_RECOGNITION_UTILS_ROS_COMPAT_H_
