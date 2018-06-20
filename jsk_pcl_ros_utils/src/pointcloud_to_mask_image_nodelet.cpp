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

namespace enc = sensor_msgs::image_encodings;

namespace jsk_pcl_ros_utils
{
  void PointCloudToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind(&PointCloudToMaskImage::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void PointCloudToMaskImage::subscribe()
  {
    sub_cloud_ = pnh_->subscribe<sensor_msgs::PointCloud2>(
        "input", 1, &PointCloudToMaskImage::convert, this);
    sub_image_ = pnh_->subscribe<sensor_msgs::Image>(
        "input/depth", 1, &PointCloudToMaskImage::convert, this);
  }

  void PointCloudToMaskImage::unsubscribe()
  {
    sub_cloud_.shutdown();
    sub_image_.shutdown();
  }

  void PointCloudToMaskImage::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    z_near_ = std::min(config.z_near, config.z_far);
    z_far_ = std::max(config.z_near, config.z_far);
    config.z_near = z_near_;
    config.z_far = z_far_;
  }

  void PointCloudToMaskImage::convert(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *pc);

    if (!pc->isOrganized())
    {
      NODELET_FATAL("Input point cloud is not organized.");
      return;
    }

    cv::Mat mask_image = cv::Mat::zeros(cloud_msg->height, cloud_msg->width, CV_8UC1);
    for (size_t index = 0; index < pc->points.size(); index++)
    {
      if (isnan(pc->points[index].x) || isnan(pc->points[index].y) || isnan(pc->points[index].z))
      {
        continue;
      }
      if (pc->points[index].z < z_near_ || pc->points[index].z > z_far_)
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

  void PointCloudToMaskImage::convert(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();

    cv_bridge::CvImage::Ptr depth_img;
    try {
      depth_img = cv_bridge::toCvCopy(image_msg);
    } catch (cv_bridge::Exception &e) {
      NODELET_ERROR_STREAM("Failed to convert image: " << e.what());
      return;
    }

    cv_bridge::CvImage::Ptr mask_img(new cv_bridge::CvImage);
    mask_img->header = depth_img->header;
    mask_img->encoding = "mono8";
    mask_img->image = cv::Mat(depth_img->image.rows, depth_img->image.cols, CV_8UC1);

    cv::MatIterator_<uint8_t>
      mask_it  = mask_img->image.begin<uint8_t>(),
      mask_end = mask_img->image.end<uint8_t>();

    if (depth_img->encoding == enc::TYPE_32FC1) {
      cv::MatConstIterator_<float>
        depth_it = depth_img->image.begin<float>(),
        depth_end = depth_img->image.end<float>();

      for (; (depth_it != depth_end) && (mask_it != mask_end); ++depth_it, ++mask_it)
      {
        if (z_near_ < *depth_it && *depth_it < z_far_) *mask_it = -1;
        else *mask_it = 0;
      }
    } else if (depth_img->encoding == enc::TYPE_16UC1) {
      uint16_t z_near16 = (uint16_t) (z_near_ * 1000.0), z_far16 = (uint16_t) (z_far_ * 1000.0); // mm
      cv::MatConstIterator_<uint16_t>
        depth_it = depth_img->image.begin<uint16_t>(),
        depth_end = depth_img->image.end<uint16_t>();

      for (; (depth_it != depth_end) && (mask_it != mask_end); ++depth_it, ++mask_it)
      {
        if (z_near16 < *depth_it && *depth_it < z_far16) *mask_it = -1;
        else *mask_it = 0;
      }

    } else {
      NODELET_ERROR_STREAM("Invalid encoding:" << depth_img->encoding);
      return;
    }

    pub_.publish(mask_img->toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::PointCloudToMaskImage, nodelet::Nodelet);
