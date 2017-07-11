/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Kentaro Wada and JSK Lab
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
 *   * Neither the name of Kentaro Wada and JSK Lab nor the names of its
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

#include <limits>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include "jsk_pcl_ros/fuse_images.h"


namespace jsk_pcl_ros
{
  void FuseImages::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_out_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void FuseImages::subscribe()
  {
    pnh_->param("approximate_sync", approximate_sync_, true);
    pnh_->param("queue_size", queue_size_, 10);
    pnh_->param("averaging", averaging_, true);  // TODO: -> dynparam

    XmlRpc::XmlRpcValue input_topics;
    if (!pnh_->getParam("input_topics", input_topics))
    {
      NODELET_ERROR("Rosparam '~input_topics' is required.");
      return;
    }
    if (input_topics.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      NODELET_ERROR("Invalid type of '~input_topics' is specified. String array is expected.");
      return;
    }

    // Subscribe to the filters
    filters_.resize(input_topics.size());

    // 8 topics
    if (approximate_sync_)
    {
      async_.reset(new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> >(queue_size_));
    }
    else
    {
      sync_.reset(new message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
          sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> > (queue_size_));
    }

    // First input_topics.size() filters are valid
    ROS_INFO("Subscribing to %d topics user given topics as inputs:", input_topics.size());
    for (size_t i = 0; i < input_topics.size (); i++)
    {
      ROS_INFO_STREAM(" - " << (std::string)(input_topics[i]));
      filters_[i].reset(new message_filters::Subscriber<sensor_msgs::Image>());
      filters_[i]->subscribe(*pnh_, (std::string)(input_topics[i]), queue_size_);
    }

    // Bogus null filter
    filters_[0]->registerCallback(bind(&FuseDepthImages::input_callback, this, _1));

    if (input_topics.size() == 2)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], nf_, nf_, nf_, nf_, nf_, nf_);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], nf_, nf_, nf_, nf_, nf_, nf_);
      }
    }
    else if (input_topics.size() == 3)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], *filters_[2], nf_, nf_, nf_, nf_, nf_);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], *filters_[2], nf_, nf_, nf_, nf_, nf_);
      }
    }
    else if (input_topics.size() == 4)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], nf_, nf_, nf_, nf_);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], nf_, nf_, nf_, nf_);
      }
    }
    else if (input_topics.size() == 5)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4], nf_, nf_, nf_);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4], nf_, nf_, nf_);
      }
    }
    else if (input_topics.size() == 6)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4],
                             *filters_[5], nf_, nf_);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4],
                            *filters_[5], nf_, nf_);
      }
    }
    else if (input_topics.size() == 7)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4],
                             *filters_[5], *filters_[6], nf_);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4],
                            *filters_[5], *filters_[6], nf_);
      }
    }
    else if (input_topics.size() == 8)
    {
      if (approximate_sync_)
      {
        async_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4],
                             *filters_[5], *filters_[6], *filters_[7]);
      }
      else
      {
        sync_->connectInput(*filters_[0], *filters_[1], *filters_[2], *filters_[3], *filters_[4],
                            *filters_[5], *filters_[6], *filters_[7]);
      }
    }
    else
    {
      NODELET_ERROR("Invalid number of topics is specified: %d. It must be > 1 and <= 8.", input_topics.size());
      return;
    }

    if (approximate_sync_)
    {
      async_->registerCallback(boost::bind(&FuseDepthImages::inputCb, this, _1, _2, _3, _4, _5, _6, _7, _8));
    }
    else
    {
      sync_->registerCallback(boost::bind(&FuseDepthImages::inputCb, this, _1, _2, _3, _4, _5, _6, _7, _8));
    }
  }

  void FuseImages::unsubscribe()
  {
    for (size_t i = 0; i < filters_.size(); i++) {
      filters_[i]->unsubscribe();
    }
  }

  bool FuseImages::validateInput(
      const sensor_msgs::Image::ConstPtr& in,
      const int height_expected,
      const int width_expected,
      std::vector<cv::Mat>& inputs)
  {
    if (in->height == 0 && in->width == 0)
    {
      // No subscription.
      return false;
    }
    if (in->height != height_expected || in->width != width_expected)
    {
      NODELET_ERROR_THROTTLE(10, "Input images must have same size: height=%d, width=%d.",
                             height_expected, width_expected);
      return false;
    }
    if (in->encoding != encoding_)
    {
      NODELET_ERROR_THROTTLE(10, "Input images must have same encoding. Expected: %s, Actual: %s",
                             encoding_.c_str(), in->encoding.c_str());
      return false;
    }
    inputs.push_back(cv_bridge::toCvShare(in, encoding_)->image);
    return true;
  }

  void FuseImages::inputCb(
    const sensor_msgs::Image::ConstPtr& in1, const sensor_msgs::Image::ConstPtr& in2,
    const sensor_msgs::Image::ConstPtr& in3, const sensor_msgs::Image::ConstPtr& in4,
    const sensor_msgs::Image::ConstPtr& in5, const sensor_msgs::Image::ConstPtr& in6,
    const sensor_msgs::Image::ConstPtr& in7, const sensor_msgs::Image::ConstPtr& in8)
  {
    int height = in1->height;
    int width = in1->width;
    std::vector<cv::Mat> inputs;
    validateInput(in1, height, width, inputs);
    validateInput(in2, height, width, inputs);
    validateInput(in3, height, width, inputs);
    validateInput(in4, height, width, inputs);
    validateInput(in5, height, width, inputs);
    validateInput(in6, height, width, inputs);
    validateInput(in7, height, width, inputs);
    validateInput(in8, height, width, inputs);

    cv::Mat out = fuseInputs(inputs);
    pub_out_.publish(cv_bridge::CvImage(in1->header, encoding_, out).toImageMsg());
  }

  cv::Mat FuseDepthImages::fuseInputs(const std::vector<cv::Mat> inputs)
  {
    cv::Mat out(inputs[0].rows, inputs[0].cols, cv_bridge::getCvType(encoding_));
    out.setTo(std::numeric_limits<float>::quiet_NaN());
    for (size_t j = 0; j < inputs[0].rows; j++)
    {
      for (size_t i = 0; i < inputs[0].cols; i++)
      {
        int n_fused = 0;
        float value_fused = 0;
        for (size_t k = 0; k < inputs.size(); k++)
        {
          float value = inputs[k].at<float>(j, i);
          if (!std::isnan(value))
          {
            value_fused += value;
            n_fused += 1;
          }
        }
        if (n_fused > 0)
        {
          out.at<float>(j, i) = value_fused / n_fused;
        }
      }
    }
    return out;
  }

  cv::Mat FuseRGBImages::fuseInputs(const std::vector<cv::Mat> inputs)
  {
    cv::Mat out(inputs[0].rows, inputs[0].cols, cv_bridge::getCvType(encoding_));
    out.setTo(cv::Scalar(0, 0, 0));
    for (size_t j = 0; j < inputs[0].rows; j++)
    {
      for (size_t i = 0; i < inputs[0].cols; i++)
      {
        int n_fused = 0;
        cv::Vec3b value_fused(0, 0, 0);
        for (size_t k = 0; k < inputs.size(); k++)
        {
          cv::Vec3b value = inputs[k].at<cv::Vec3b>(j, i);
          if (value[0] != 0 || value[1] != 0 || value[2] != 0)
          {
            if (!averaging_ && n_fused > 0)
            {
              continue;
            }
            value_fused += value;
            n_fused += 1;
          }
        }
        if (n_fused > 0)
        {
          out.at<cv::Vec3b>(j, i) = value_fused / n_fused;
        }
      }
    }
    return out;
  }

} // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::FuseDepthImages, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::FuseRGBImages, nodelet::Nodelet);
