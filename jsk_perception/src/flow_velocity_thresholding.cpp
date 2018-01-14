/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#include <boost/assign.hpp>
#include <boost/tuple/tuple.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_utils/cv_utils.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Dense>
#include "jsk_perception/flow_velocity_thresholding.h"

namespace jsk_perception
{
  void FlowVelocityThresholding::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FlowVelocityThresholding::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);
    pnh_->param("use_camera_info", use_camera_info_, true);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void FlowVelocityThresholding::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    threshold_ = config.threshold;
    window_size_ = config.window_size;
  }

  void FlowVelocityThresholding::subscribe()
  {
    sub_flow_.subscribe(*pnh_, "input/flows", 1);
    ros::V_string names = boost::assign::list_of("~input/flows");
    if (use_camera_info_)
    {
      sub_info_.subscribe(*pnh_, "input/camera_info", 1);
      if (approximate_sync_) {
        async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
        async_->connectInput(sub_flow_, sub_info_);
        async_->registerCallback(boost::bind(&FlowVelocityThresholding::callback, this, _1, _2));
      }
      else {
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
        sync_->connectInput(sub_flow_, sub_info_);
        sync_->registerCallback(boost::bind(&FlowVelocityThresholding::callback, this, _1, _2));
      }
      names.push_back("~input/camera_info");
    }
    else
    {
      if (pnh_->hasParam("image_height") && pnh_->hasParam("image_width"))
      {
        pnh_->getParam("image_height", image_height_);
        pnh_->getParam("image_width", image_width_);
        sub_flow_.registerCallback(&FlowVelocityThresholding::callback, this);
      }
      else
      {
        ROS_FATAL("Rosparam ~image_height and ~image_width must be set if ~use_camera_info=false");
      }
    }
    jsk_topic_tools::warnNoRemap(names);
  }

  void FlowVelocityThresholding::unsubscribe()
  {
    sub_flow_.unsubscribe();
    sub_info_.unsubscribe();
  }

  void FlowVelocityThresholding::callback(
    const opencv_apps::FlowArrayStamped::ConstPtr& flow_msg)
  {
    process(flow_msg, image_height_, image_width_);
  }

  void FlowVelocityThresholding::callback(
    const opencv_apps::FlowArrayStamped::ConstPtr& flow_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    process(flow_msg, info_msg->height, info_msg->width);
  }

  void FlowVelocityThresholding::process(
    const opencv_apps::FlowArrayStamped::ConstPtr& flow_msg,
    const int image_height,
    const int image_width)
  {
    vital_checker_->poke();

    cv::Mat mask = cv::Mat::zeros(image_height, image_width, CV_8UC1);

    for (size_t i=0; i<flow_msg->flow.size(); i++)
    {
      Eigen::Vector2d vel;
      vel(0) = flow_msg->flow[i].velocity.x;
      vel(1) = flow_msg->flow[i].velocity.y;
      if (vel.norm() > threshold_) {
        cv::Point center = cv::Point(flow_msg->flow[i].point.x, flow_msg->flow[i].point.y);
        cv::rectangle(
          mask,
          center - cv::Point(window_size_ / 2, window_size_ / 2),
          center + cv::Point(window_size_ / 2, window_size_ / 2),
          cv::Scalar(255),
          CV_FILLED);
      }
    }

    pub_.publish(cv_bridge::CvImage(
                    flow_msg->header,
                    sensor_msgs::image_encodings::MONO8,
                    mask).toImageMsg());
  }
}  // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::FlowVelocityThresholding, nodelet::Nodelet);
