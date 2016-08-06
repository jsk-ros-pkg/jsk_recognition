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
 *     disclaimer in the documentation and/or other materials provided
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

#ifndef JSK_PERCEPTION_COLOR_FILTER_H_
#define JSK_PERCEPTION_COLOR_FILTER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_perception/RGBColorFilterConfig.h"
#include "jsk_perception/HSIColorFilterConfig.h"
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception {
  class RGBColorFilter;
  class HSIColorFilter;

  template <typename Config>
  class ColorFilter : public jsk_topic_tools::DiagnosticNodelet {
    friend class RGBColorFilter;
    friend class HSIColorFilter;

   public:
    ColorFilter() : DiagnosticNodelet("ColorFilter") {}

   protected:
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config& config, uint32_t level) = 0;
    virtual void filter(const sensor_msgs::Image::ConstPtr& input_image_msg);
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header) = 0;
    virtual void updateCondition() = 0;

    cv::Scalar lower_color_range_;
    cv::Scalar upper_color_range_;

    ros::Publisher pub_;
    ros::Subscriber sub_;

    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;

    boost::mutex mutex_;

   private:
    virtual void onInit();
  };

  class RGBColorFilter
    : public ColorFilter<jsk_perception::RGBColorFilterConfig> {
   public:
   protected:
    int r_min_;
    int r_max_;
    int b_min_;
    int b_max_;
    int g_min_;
    int g_max_;
    virtual void configCallback(jsk_perception::RGBColorFilterConfig& config,
                                uint32_t level);
    virtual void updateCondition();
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header);

   private:
    virtual void onInit();
  };

  class HSIColorFilter
    : public ColorFilter<jsk_perception::HSIColorFilterConfig> {
   public:
   protected:
    int h_min_;
    int h_max_;
    int s_min_;
    int s_max_;
    int i_min_;
    int i_max_;
    virtual void configCallback(jsk_perception::HSIColorFilterConfig& config,
                                uint32_t level);
    virtual void updateCondition();
    virtual void process(const cv::Mat& input_image,
                         const std_msgs::Header& header);

   private:
    virtual void onInit();
  };
}

#endif
