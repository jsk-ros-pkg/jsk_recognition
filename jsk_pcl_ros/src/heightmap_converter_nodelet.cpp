// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/heightmap_converter.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_topic_tools/color_utils.h>

namespace jsk_pcl_ros
{
  void HeightmapConverter::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_config_ = pnh_->advertise<jsk_recognition_msgs::HeightmapConfig>(
      "output/config", 1, true);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&HeightmapConverter::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pnh_->param("max_queue_size", max_queue_size_, 10);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }
  
  void HeightmapConverter::subscribe()
  {
    sub_ = pnh_->subscribe("input", max_queue_size_, &HeightmapConverter::convert, this);
  }

  void HeightmapConverter::unsubscribe()
  {
    sub_.shutdown();
  }

  void HeightmapConverter::convert(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    /* float image */
    cv::Mat height_map = cv::Mat(resolution_y_, resolution_x_, CV_32FC1);
    height_map = cv::Scalar::all(- FLT_MAX);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    float max_height = - FLT_MAX;
    float min_height = FLT_MAX;
    for (size_t i = 0; i < cloud.points.size(); i++) {
      pcl::PointXYZ p = cloud.points[i];
      if (isnan(p.x) || isnan(p.y) || isnan(p.z)) {
        continue;
      }
      cv::Point index = toIndex(p);
      if (index.x >= 0 && index.x < resolution_x_ && 
          index.y >= 0 && index.y < resolution_y_) {
        /* Store min/max value for colorization */
        max_height = std::max(max_height, p.z);
        min_height = std::min(min_height, p.z);
        if (height_map.at<float>(index.y, index.x) < p.z) {
          height_map.at<float>(index.y, index.x) = p.z;
        }
      }
    }
    // Convert to sensor_msgs/Image
    cv_bridge::CvImage height_map_image(msg->header,
                                        sensor_msgs::image_encodings::TYPE_32FC1,
                                        height_map);
    pub_.publish(height_map_image.toImageMsg());
  }

  void HeightmapConverter::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_x_ = config.min_x;
    max_x_ = config.max_x;
    min_y_ = config.min_y;
    max_y_ = config.max_y;
    resolution_x_ = config.resolution_x;
    resolution_y_ = config.resolution_y;
    jsk_recognition_msgs::HeightmapConfig heightmap_config;
    heightmap_config.min_x = min_x_;
    heightmap_config.min_y = min_y_;
    heightmap_config.max_x = max_x_;
    heightmap_config.max_y = max_y_;
    pub_config_.publish(heightmap_config);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HeightmapConverter,
                        nodelet::Nodelet);
