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

#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_pcl_ros/heightmap_to_pointcloud.h"
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace jsk_pcl_ros
{
  void HeightmapToPointCloud::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_config_ = pnh_->advertise<jsk_recognition_msgs::HeightmapConfig>(
      "output/config", 1);
    sub_config_ = pnh_->subscribe(
      getHeightmapConfigTopic(pnh_->resolveName("input")), 1,
      &HeightmapToPointCloud::configCallback, this);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void HeightmapToPointCloud::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &HeightmapToPointCloud::convert, this);
  }

  void HeightmapToPointCloud::unsubscribe()
  {
    sub_.shutdown();
  }

  void HeightmapToPointCloud::configCallback(
    const jsk_recognition_msgs::HeightmapConfig::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_msg_ = msg;
    min_x_ = msg->min_x;
    max_x_ = msg->max_x;
    min_y_ = msg->min_y;
    max_y_ = msg->max_y;
    pub_config_.publish(msg);
  }

  void HeightmapToPointCloud::convert(const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!config_msg_) {
      NODELET_ERROR("no ~input/config is yet available");
      return;
    }

    cv::Mat float_image = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_32FC2)->image;
    pcl::PointCloud<HeightMapPointType > cloud;

    bool keep_organized;
    pnh_->param("keep_organized", keep_organized, false);
    if (keep_organized) {
      convertHeightMapToPCLOrganize(float_image, cloud, max_x_, min_x_, max_y_, min_y_);
    }
    else {
      convertHeightMapToPCL(float_image, cloud, max_x_, min_x_, max_y_, min_y_);
    }

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(cloud, ros_cloud);
    ros_cloud.header = msg->header;
    pub_.publish(ros_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HeightmapToPointCloud, nodelet::Nodelet);
