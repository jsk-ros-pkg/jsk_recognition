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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include "jsk_pcl_ros/roi_clipper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

namespace jsk_pcl_ros
{
  void ROIClipper::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void ROIClipper::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input/image", 1);
    sub_info_.subscribe(*pnh_, "input/camera_info", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_image_, sub_info_);
    sync_->registerCallback(boost::bind(&ROIClipper::clip, this, _1, _2));
  }
  
  void ROIClipper::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_info_.unsubscribe();
  }
  void ROIClipper::clip(const sensor_msgs::Image::ConstPtr& image_msg,
                        const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
  {
    vital_checker_->poke();
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
      cv::Mat image = cv_ptr->image;
      cv::Rect roi(camera_info_msg->roi.x_offset, camera_info_msg->roi.y_offset,
                   camera_info_msg->roi.width, camera_info_msg->roi.height);
      //NODELET_INFO("roi::(%d, %d, %d, %d)", roi.x, roi.y, roi.width, roi.height);
      cv::Mat image_roi = image(roi);
      // cv::imshow("roi", image_roi);
      // cv::waitKey(3);
      cv_bridge::CvImage roi_bridge(image_msg->header,
                                    sensor_msgs::image_encodings::RGB8,
                                    image_roi);
      pub_image_.publish(roi_bridge.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void ROIClipper::updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "ROIClipper running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "ROIClipper", vital_checker_, stat);
    }

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ROIClipper, nodelet::Nodelet);
