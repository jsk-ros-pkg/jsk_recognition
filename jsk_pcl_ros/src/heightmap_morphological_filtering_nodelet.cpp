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
#include "jsk_pcl_ros/heightmap_morphological_filtering.h"
#include "jsk_pcl_ros/heightmap_utils.h"
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  void HeightmapMorphologicalFiltering::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_config_ = pnh_->advertise<jsk_recognition_msgs::HeightmapConfig>("output/config",
                                                                         1, true);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&HeightmapMorphologicalFiltering::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pnh_->param("max_queue_size", max_queue_size_, 10);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    sub_config_ = pnh_->subscribe(getHeightmapConfigTopic(pnh_->resolveName("input")), 1,
                                  &HeightmapMorphologicalFiltering::configTopicCallback,
                                  this);
  }

  void HeightmapMorphologicalFiltering::subscribe()
  {
    sub_ = pnh_->subscribe(
      "input", max_queue_size_,
      &HeightmapMorphologicalFiltering::filter, this);
  }

  void HeightmapMorphologicalFiltering::unsubscribe()
  {
    sub_.shutdown();
  }

  void HeightmapMorphologicalFiltering::configTopicCallback(const jsk_recognition_msgs::HeightmapConfig::ConstPtr& msg)
  {
    pub_config_.publish(msg);
  }

  void HeightmapMorphologicalFiltering::configCallback(
    Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    mask_size_ = config.mask_size;
    max_variance_ = config.max_variance;
  }

  void HeightmapMorphologicalFiltering::filter(
    const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    cv::Mat input = cv_bridge::toCvShare(
      msg, sensor_msgs::image_encodings::TYPE_32FC1)->image;
    cv::Mat filtered_image = input.clone();

    for (size_t j = 0; j < input.rows; j++) {
      for (size_t i = 0; i < input.cols; i++) {
        float v = input.at<float>(j, i);
        if (isnan(v) || v == -FLT_MAX) { // Need to filter
          Accumulator acc;
          for (int jj = - mask_size_; jj <= mask_size_; jj++) {
            int target_j = j + jj;
            if (target_j >= 0 && target_j < input.rows) {
              for (int ii = - mask_size_; ii <= mask_size_; ii++) {
                int target_i = i + ii;
                if (target_i >= 0 && target_i < input.cols) {
                  if (std::abs(jj) + std::abs(ii) <= mask_size_) {
                    float vv = input.at<float>(target_j, target_i);
                    if (!isnan(vv) && vv != -FLT_MAX) {
                      acc(vv);
                    }
                  }
                }
              }
            }
          }
          if (boost::accumulators::count(acc) != 0) {
            float newv = boost::accumulators::mean(acc);
            float variance = boost::accumulators::variance(acc);
            if (variance < max_variance_) {
              filtered_image.at<float>(j, i) = newv;
            }
          }
        }
      }
    }
    
    pub_.publish(cv_bridge::CvImage(
                   msg->header,
                   sensor_msgs::image_encodings::TYPE_32FC1,
                   filtered_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HeightmapMorphologicalFiltering, nodelet::Nodelet);
