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
#include <Eigen/Geometry>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include "jsk_perception/tabletop_color_difference_likelihood.h"
#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_utils/sensor_model_utils.h>

namespace jsk_perception
{
  void TabletopColorDifferenceLikelihood::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("tf_queue_size", tf_queue_size_, 10);
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
  }

  void TabletopColorDifferenceLikelihood::subscribe()
  {
    sub_info_ = pnh_->subscribe("input/camera_info", 1, 
                                &TabletopColorDifferenceLikelihood::infoCallback, this);
    sub_polygons_ = pnh_->subscribe("input/polygons", 1, 
                                &TabletopColorDifferenceLikelihood::polygonCallback, this);
    sub_image_.subscribe(*pnh_, "input", 1);
  }

  void TabletopColorDifferenceLikelihood::unsubscribe()
  {
    sub_info_.shutdown();
    sub_polygons_.shutdown();
    sub_image_.unsubscribe();
  }

  void TabletopColorDifferenceLikelihood::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_info_msg_ = msg;
  }

  void TabletopColorDifferenceLikelihood::polygonCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_polygon_msg_ = msg;
    if (!tf_filter_) {
      // If tf_filter_ is not initialized yet,
      // we initialize tf_filter_ with frame_id
      // of the message.
      tf_filter_.reset(new tf::MessageFilter<sensor_msgs::Image>(sub_image_,
                                                                 *tf_listener_,
                                                                 msg->header.frame_id,
                                                                 tf_queue_size_));
      tf_filter_->registerCallback(boost::bind(&TabletopColorDifferenceLikelihood::imageCallback, this, _1));
    }
  }

  void TabletopColorDifferenceLikelihood::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // check camera info and polygon is available
    if (!latest_info_msg_) {
      JSK_ROS_WARN("no camera info is available yet");
      return;
    }
    if (!latest_polygon_msg_) {
      JSK_ROS_WARN("no polygon is available yet");
      return;
    }

    // try-catch for tf exception
    try 
    {
      
      cv_bridge::CvImagePtr input_cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat input_image = input_cv_image->image;
      // Transform PolygonArray to the frame of image
      tf::StampedTransform transform;
      tf_listener_->lookupTransform(msg->header.frame_id,
                                    latest_polygon_msg_->header.frame_id,
                                    msg->header.stamp,
                                    transform);
      Eigen::Affine3f polygon_transform;
      tf::transformTFToEigen(transform, polygon_transform);
      
      // convert to polygon
      std::vector<jsk_recognition_utils::Polygon::Ptr> polygons 
        = jsk_recognition_utils::Polygon::fromROSMsg(*latest_polygon_msg_, polygon_transform);
      jsk_recognition_utils::CameraDepthSensor model;
      model.setCameraInfo(*latest_info_msg_);
      cv::Mat image = model.image(CV_8UC1);
      model.setCameraInfo(*latest_info_msg_);
      
      for (size_t i = 0; i < polygons.size(); i++) {
        jsk_recognition_utils::Polygon::Ptr polygon = polygons[i];
        cv::Mat mask_image;
        polygon->maskImage(model, mask_image);
          cv::Scalar center_color = cv::mean(input_image, mask_image);
          for (size_t j = 0; j < model.height(); j++) {
            for (size_t i = 0; i < model.width(); i++) {
              if (mask_image.at<unsigned char>(j, i) != 0) {
                unsigned char current_value = image.at<unsigned char>(j, i);
                unsigned char new_value = (unsigned char)std::abs((int)input_image.at<unsigned char>(j, i) - (int)center_color[0]);
                if (current_value > new_value || current_value == 0) {
                  image.at<unsigned char>(j, i) = new_value;
                }
              }
          }
        }
      }
      pub_.publish(cv_bridge::CvImage(msg->header,
                                      sensor_msgs::image_encodings::MONO8,
                                      image).toImageMsg());

    }
    catch (...)
    {
      JSK_NODELET_ERROR("Failed to resolve tf");
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::TabletopColorDifferenceLikelihood, nodelet::Nodelet);
