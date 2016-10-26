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

#include "jsk_perception/polygon_array_to_label_image.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_perception
{
  void PolygonArrayToLabelImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void PolygonArrayToLabelImage::subscribe()
  {
    sub_info_ = pnh_->subscribe("input/camera_info", 1,
                                &PolygonArrayToLabelImage::infoCallback, this);
    sub_ = pnh_->subscribe("input", 1,
                           &PolygonArrayToLabelImage::convert, this);
    ros::V_string names = boost::assign::list_of("~input")("~input/camera_info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void PolygonArrayToLabelImage::unsubscribe()
  {
    sub_info_.shutdown();
    sub_.shutdown();
  }

  void PolygonArrayToLabelImage::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_ = info_msg;
  }

  void PolygonArrayToLabelImage::convert(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    if (camera_info_) {
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(camera_info_);
      cv::Mat mask_image = cv::Mat::zeros(camera_info_->height,
                                          camera_info_->width,
                                          CV_32SC1);
      
      int label_counter = 1;
      // we expect same tf frame
      for (size_t i_polygon = 0; i_polygon < polygon_msg->polygons.size(); ++i_polygon) {
        geometry_msgs::Polygon polygon = polygon_msg->polygons[i_polygon].polygon;
        std::vector<cv::Point> points;
        if (polygon.points.size() >= 3) {
          bool all_outside = true;
          for (size_t i = 0; i < polygon.points.size(); i++) {
            geometry_msgs::Point32 p = polygon.points[i];
            cv::Point uv = model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
            if ((uv.x > 0 && uv.x < camera_info_->width) &&
                (uv.y > 0 && uv.y < camera_info_->height)) {
              all_outside = false;
            }
            points.push_back(uv);
          }
          if (!all_outside) {
            cv::fillConvexPoly(mask_image, &(points[0]), points.size(), label_counter++);
          }
        }
      }
      pub_.publish(cv_bridge::CvImage(polygon_msg->header,
                                      sensor_msgs::image_encodings::TYPE_32SC1,
                                      mask_image).toImageMsg());
    }
    else {
      NODELET_WARN("no camera info is available");
    }
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::PolygonArrayToLabelImage, nodelet::Nodelet);
