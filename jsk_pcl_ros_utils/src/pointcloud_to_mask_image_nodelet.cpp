// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
#include "jsk_pcl_ros_utils/pointcloud_to_mask_image.h"
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros_utils
{
  void PointCloudToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PointCloudToMaskImage::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &PointCloudToMaskImage::convert, this);
  }

  void PointCloudToMaskImage::unsubscribe()
  {
    sub_.shutdown();
  }

  void PointCloudToMaskImage::updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive())
    {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "PointCloudToMaskImage running");
    }
    else
    {
      jsk_topic_tools::addDiagnosticErrorSummary("PointCloudToMaskImage", vital_checker_, stat);
    }
  }

  void PointCloudToMaskImage::convert(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pc);

    if (!pc->isOrganized())
    {
      JSK_NODELET_FATAL("Input point cloud is not organized.");
      return;
    }

    cv::Mat mask_image = cv::Mat::zeros(cloud_msg->height, cloud_msg->width, CV_8UC1);
    for (size_t index = 0; index < pc->points.size(); index++)
    {
      if (isnan(pc->points[index].x) || isnan(pc->points[index].y) || isnan(pc->points[index].z))
      {
        continue;
      }
      int width_index = index % cloud_msg->width;
      int height_index = index / cloud_msg->width;
      mask_image.at<uchar>(height_index, width_index) = 255;
    }
    cv_bridge::CvImage mask_bridge(cloud_msg->header,
                                   sensor_msgs::image_encodings::MONO8,
                                   mask_image);
    pub_.publish(mask_bridge.toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::PointCloudToMaskImage, nodelet::Nodelet);
