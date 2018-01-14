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

#include "jsk_perception/polygon_to_mask_image.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void PolygonToMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void PolygonToMaskImage::subscribe()
  {
    sub_info_ = pnh_->subscribe("input/camera_info", 1,
                                &PolygonToMaskImage::infoCallback, this);
    sub_ = pnh_->subscribe("input", 1,
                           &PolygonToMaskImage::convert, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/camera_info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void PolygonToMaskImage::unsubscribe()
  {
    sub_info_.shutdown();
    sub_.shutdown();
  }

  void PolygonToMaskImage::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_ = info_msg;
  }

  void PolygonToMaskImage::convert(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    if (camera_info_) {
      if (polygon_msg->header.frame_id != camera_info_->header.frame_id) {
        NODELET_ERROR("frame_id of polygon (%s) and camera (%s) are not same.",
                      polygon_msg->header.frame_id.c_str(), camera_info_->header.frame_id.c_str());
      }

      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(camera_info_);
      cv::Mat mask_image = cv::Mat::zeros(camera_info_->height,
                                          camera_info_->width,
                                          CV_8UC1);
      std::vector<cv::Point> points;
      // we expect same tf frame
      if (polygon_msg->polygon.points.size() >= 3) {
        for (size_t i = 0; i < polygon_msg->polygon.points.size(); i++) {
          geometry_msgs::Point32 p = polygon_msg->polygon.points[i];
          cv::Point uv = model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
          points.push_back(uv);
        }
        cv::fillConvexPoly(mask_image, &(points[0]), points.size(), cv::Scalar(255));
      }
      pub_.publish(cv_bridge::CvImage(polygon_msg->header,
                                      sensor_msgs::image_encodings::MONO8,
                                      mask_image).toImageMsg());
    }
    else {
      NODELET_WARN("no camera info is available");
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::PolygonToMaskImage, nodelet::Nodelet);
