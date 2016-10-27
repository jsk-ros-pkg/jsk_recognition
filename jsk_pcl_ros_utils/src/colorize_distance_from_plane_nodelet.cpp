// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *     disclaimer in the documentation and/o2r other materials provided
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

#include "jsk_pcl_ros_utils/colorize_distance_from_plane.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros_utils
{
  void ColorizeDistanceFromPlane::onInit()
  {
    ConnectionBasedNodelet::onInit();

    ////////////////////////////////////////////////////////
    // publisher
    ////////////////////////////////////////////////////////
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    
    ////////////////////////////////////////////////////////
    // dynamic reconfigure
    ////////////////////////////////////////////////////////

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ColorizeDistanceFromPlane::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void ColorizeDistanceFromPlane::subscribe()
  {
    ////////////////////////////////////////////////////////
    // subscribe
    ////////////////////////////////////////////////////////
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_coefficients_, sub_polygons_);
    sync_->registerCallback(boost::bind(
                              &ColorizeDistanceFromPlane::colorize,
                              this, _1, _2, _3));
  }

  void ColorizeDistanceFromPlane::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_coefficients_.unsubscribe();
    sub_polygons_.unsubscribe();
  }
  
  double ColorizeDistanceFromPlane::distanceToConvexes(
    const PointT& p,
    const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
  {
    Eigen::Vector3f v = p.getVector3fMap();
    double min_distance = DBL_MAX;
    for (size_t i = 0; i < convexes.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr convex = convexes[i];
      if (!only_projectable_ || convex->isProjectableInside(v)) {
        double d = convex->distanceToPoint(v);
        if (d < min_distance) {
          min_distance = d;
        }
      }
    }
    return min_distance;
  }

  uint32_t ColorizeDistanceFromPlane::colorForDistance(const double d)
  {
    double ratio = 0.0;
    if (d > max_distance_) {
      ratio = 1.0;
    }
    else if (d < min_distance_) {
      ratio = 0.0;
    }
    else {
      ratio = fabs(min_distance_ - d) / (max_distance_ - min_distance_);
    }
    double r = ratio;
    double g = 0.0;
    double b = 1 - ratio;
    uint8_t ru, gu, bu;
    ru = (uint8_t)(r * 255);
    gu = (uint8_t)(g * 255);
    bu = (uint8_t)(b * 255);
    return ((uint32_t)ru<<16 | (uint32_t)gu<<8 | (uint32_t)bu);
  }
  
  void ColorizeDistanceFromPlane::colorize(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (coefficients_msg->coefficients.size() == 0) {
      return;
    }
    // convert all the data into pcl format
    pcl::PointCloud<PointT>::Ptr cloud
      (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    std::vector<pcl::ModelCoefficients::Ptr> coefficients
      = pcl_conversions::convertToPCLModelCoefficients(
        coefficients_msg->coefficients);
    
    // first, build jsk_recognition_utils::ConvexPolygon::Ptr
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes;
    for (size_t i = 0; i < polygons->polygons.size(); i++) {
      if (polygons->polygons[i].polygon.points.size() > 0) {
        jsk_recognition_utils::ConvexPolygon convex =
          jsk_recognition_utils::ConvexPolygon::fromROSMsg(polygons->polygons[i].polygon);
        jsk_recognition_utils::ConvexPolygon::Ptr convex_ptr
          = boost::make_shared<jsk_recognition_utils::ConvexPolygon>(convex);
        convexes.push_back(convex_ptr);
      }
      else {
        NODELET_ERROR_STREAM(__PRETTY_FUNCTION__ << ":: there is no points in the polygon");
      }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud
      (new pcl::PointCloud<pcl::PointXYZRGB>);
    
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      pcl::PointXYZRGB p_output;
      jsk_recognition_utils::pointFromXYZToXYZ<PointT, pcl::PointXYZRGB>(p, p_output);
      double d = distanceToConvexes(p, convexes);
      if (d != DBL_MAX) {
        uint32_t color = colorForDistance(d);
        p_output.rgb = *reinterpret_cast<float*>(&color);
        output_cloud->points.push_back(p_output);
      }
    }
    
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(*output_cloud, ros_output);
    ros_output.header = cloud_msg->header;
    pub_.publish(ros_output);
  }
  
  void ColorizeDistanceFromPlane::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (config.max_distance < config.min_distance) {
      if (max_distance_ != config.max_distance) {
        config.min_distance = config.max_distance;
      }
      else if (min_distance_ != config.min_distance) {
        config.max_distance = config.min_distance;
      }
    }
    else {
      max_distance_ = config.max_distance;
      min_distance_ = config.min_distance;
    }
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::ColorizeDistanceFromPlane,
                        nodelet::Nodelet);
