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
#include "jsk_pcl_ros_utils/mask_image_to_point_indices.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros_utils
{
  void MaskImageToPointIndices::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("use_multi_channels", use_multi_channels_, false);
    pnh_->param("target_channel", target_channel_, -1);

    if (use_multi_channels_ && target_channel_ < 0) {
      pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output/all_indices", 1);
    } else {
      pub_ = advertise<PCLIndicesMsg>(*pnh_, "output", 1);
    }
    onInitPostProcess();
  }

  void MaskImageToPointIndices::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1,
                           &MaskImageToPointIndices::indices,
                           this);
  }

  void MaskImageToPointIndices::unsubscribe()
  {
    sub_.shutdown();
  }

  void MaskImageToPointIndices::indices(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    if (use_multi_channels_) {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(image_msg);
      cv::Mat image = cv_ptr->image;
      if (target_channel_ < 0) {
        jsk_recognition_msgs::ClusterPointIndices cluster_msg;
        cluster_msg.header = image_msg->header;
        cluster_msg.cluster_indices.resize(image.channels());
        for (size_t c = 0; c < image.channels(); ++c) {
          PCLIndicesMsg &indices_msg = cluster_msg.cluster_indices[c];
          indices_msg.header = image_msg->header;
          for (size_t j = 0; j < image.rows; j++) {
            for (size_t i = 0; i < image.cols; i++) {
              if (image.data[j * image.step + i * image.elemSize() + c] > 127) {
                indices_msg.indices.push_back(j * image.cols + i);
              }
            }
          }
        }
        pub_.publish(cluster_msg);
      } else {
        if (target_channel_ > image.channels() - 1) {
          NODELET_ERROR("target_channel_ is %d, but image has %d channels",
                        target_channel_, image.channels());
          return;
        }
        PCLIndicesMsg indices_msg;
        indices_msg.header = image_msg->header;
        for (size_t j = 0; j < image.rows; j++) {
          for (size_t i = 0; i < image.cols; i++) {
            if (image.data[j * image.step + i * image.elemSize() + target_channel_] > 127) {
              indices_msg.indices.push_back(j * image.cols + i);
            }
          }
        }
        pub_.publish(indices_msg);
      }
    } else {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
        image_msg, sensor_msgs::image_encodings::TYPE_8UC1);
      cv::Mat image = cv_ptr->image;
      PCLIndicesMsg indices_msg;
      indices_msg.header = image_msg->header;
      for (size_t j = 0; j < image.rows; j++) {
        for (size_t i = 0; i < image.cols; i++) {
          if (image.at<uchar>(j, i) > 127) {
            indices_msg.indices.push_back(j * image.cols + i);
          }
        }
      }
      pub_.publish(indices_msg);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::MaskImageToPointIndices, nodelet::Nodelet);
