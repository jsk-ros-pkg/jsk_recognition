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
    pub_ = advertise<PCLIndicesMsg>(*pnh_, "output", 1);
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

  void MaskImageToPointIndices::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "MaskImageToPointIndices running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "MaskImageToPointIndices", vital_checker_, stat);
    }
  }
  
  void MaskImageToPointIndices::indices(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    vital_checker_->poke();
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      image_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat image = cv_ptr->image;
    PCLIndicesMsg indices_msg;
    indices_msg.header = image_msg->header;
    for (size_t j = 0; j < image.rows; j++) {
      for (size_t i = 0; i < image.cols; i++) {
        if (image.at<uchar>(j, i) == 255) {
          indices_msg.indices.push_back(j * image.cols + i);
        }
      }
    }
    pub_.publish(indices_msg);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::MaskImageToPointIndices, nodelet::Nodelet);
