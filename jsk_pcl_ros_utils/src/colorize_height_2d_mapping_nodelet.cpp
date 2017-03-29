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

#include "jsk_pcl_ros_utils/colorize_height_2d_mapping.h"

#include <pcl_conversions/pcl_conversions.h>

namespace jsk_pcl_ros_utils
{
  void ColorizeHeight2DMapping::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ColorizeHeight2DMapping::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ColorizeHeight2DMapping::colorize, this);
  }

  void ColorizeHeight2DMapping::unsubscribe()
  {
    sub_.shutdown();
  }

  void ColorizeHeight2DMapping::colorize(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);

    pcl::PointCloud<pcl::PointXYZI> colorized_cloud;
    for (size_t i = 0; i < cloud.points.size(); i++) {
      pcl::PointXYZ in = cloud.points[i];
      if (isnan(in.x) || isnan(in.y) || isnan(in.z)) {
        continue;
      }
      pcl::PointXYZI out;
      out.x = in.x;
      out.y = in.y;
      out.z = 0.0;
      out.intensity = in.z;
      colorized_cloud.points.push_back(out);
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(colorized_cloud, ros_cloud);
    ros_cloud.header = msg->header;
    pub_.publish(ros_cloud);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::ColorizeHeight2DMapping, nodelet::Nodelet);
