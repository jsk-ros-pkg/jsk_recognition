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

#include "jsk_perception/rect_array_actual_size_filter.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>

namespace jsk_perception
{
  void RectArrayActualSizeFilter::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &RectArrayActualSizeFilter::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<jsk_recognition_msgs::RectArray>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void RectArrayActualSizeFilter::subscribe()
  {
    sub_rect_array_.subscribe(*pnh_, "input", 1);
    sub_image_.subscribe(*pnh_, "input/depth_image", 1);
    sub_info_.subscribe(*pnh_, "input/info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(100);
      async_->connectInput(sub_rect_array_, sub_image_, sub_info_);
      async_->registerCallback(boost::bind(&RectArrayActualSizeFilter::filter, this, _1, _2, _3));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_rect_array_, sub_image_, sub_info_);
      sync_->registerCallback(boost::bind(&RectArrayActualSizeFilter::filter, this, _1, _2, _3));
    }
  }

  void RectArrayActualSizeFilter::unsubscribe()
  {
    sub_rect_array_.unsubscribe();
    sub_image_.unsubscribe();
    sub_info_.unsubscribe();
  }

  void RectArrayActualSizeFilter::configCallback(Config& config,
                                                 uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    kernel_size_ = config.kernel_size;
    min_x_ = config.min_x;
    min_y_ = config.min_y;
    max_x_ = config.max_x;
    max_y_ = config.max_y;
  }

  double RectArrayActualSizeFilter::averageDistance(
    const int center_x, const int center_y, const cv::Mat& img) const
  {
    double d = 0.0;
    int valid = 0;
    for (int j = -kernel_size_; j <= kernel_size_; j++) {
      for (int i = -kernel_size_; i <= kernel_size_; i++) {
        const int x = center_x + i;
        const int y = center_y + j;
        if (0 <= x && x <= img.cols &&
            0 <= y && y <= img.rows) {
          d += img.at<float>(y, x);
          ++valid;
        }
      }
    }
    return d / valid;
  }
  
  void RectArrayActualSizeFilter::filter
  (const jsk_recognition_msgs::RectArray::ConstPtr& rect_array_msg,
   const sensor_msgs::Image::ConstPtr& depth_image_msg,
   const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // 1. compute average distance from kernel_size_
    // 2. compute x and y actual size at the average distance
    // 3. filter them
    jsk_recognition_msgs::RectArray result_msg;
    result_msg.header = rect_array_msg->header;
    cv_bridge::CvImagePtr cv_depth = cv_bridge::toCvCopy(depth_image_msg, sensor_msgs::image_encodings::TYPE_32FC1);
    cv::Mat depth = cv_depth->image;
    cv::Mat average_depth;
    // 1
    jsk_recognition_utils::CameraDepthSensor model;
    model.setCameraInfo(*info_msg);
    image_geometry::PinholeCameraModel camera_model = model.getPinholeCameraModel();
    for (size_t i = 0; i< rect_array_msg->rects.size(); i++) {
      jsk_recognition_msgs::Rect rect = rect_array_msg->rects[i];
      // rect has x, y, width and height
      const int center_x = rect.x + rect.width / 2;
      const int center_y = rect.y + rect.height / 2;
      const cv::Point A(rect.x, rect.y);
      const cv::Point C(rect.x + rect.width, rect.y + rect.height);
      //const double distance = average_depth.at<double>(center_y, center_x);
      const double distance = averageDistance(center_x, center_y, depth);  // z [m]
      cv::Point3d a_ray = camera_model.projectPixelTo3dRay(A);  // (x, y, z) [depth_value]
      cv::Point3d c_ray = camera_model.projectPixelTo3dRay(C);  // (x, y, z) [depth_value]
      if (a_ray.z != 0.0 && c_ray.z != 0.0) {
        cv::Point3d a_3d = a_ray * (distance / a_ray.z);  // m = depth_value * (m / depth_value)
        cv::Point3d c_3d = c_ray * (distance / c_ray.z);  // m = depth_value * (m / depth_value)
        const double width = std::abs(a_3d.x - c_3d.x);
        const double height = std::abs(a_3d.y - c_3d.y);
        if (min_x_ <= width && width <= max_x_ &&
            min_y_ <= height && height <= max_y_) {
          result_msg.rects.push_back(rect);
        }
      }
      else {
        NODELET_ERROR("rect has z=0 ray");
        return;
      }
    }
    
    pub_.publish(result_msg);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::RectArrayActualSizeFilter, nodelet::Nodelet);
