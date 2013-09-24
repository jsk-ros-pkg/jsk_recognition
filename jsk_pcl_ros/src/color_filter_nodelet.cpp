/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/color_filter_nodelet.h"

namespace jsk_pcl_ros
{
  void RGBColorFilter::config_callback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);

    if (impl_.getRedMax() != config.r_limit_max)
      impl_.setRedMax(config.r_limit_max);

    if (impl_.getGreenMax() != config.g_limit_max)
      impl_.setGreenMax(config.g_limit_max);

    if (impl_.getBlueMax() != config.b_limit_max)
      impl_.setBlueMax(config.b_limit_max);

    if (impl_.getRedMin() != config.r_limit_min)
      impl_.setRedMin(config.r_limit_min);

    if (impl_.getGreenMin() != config.g_limit_min)
      impl_.setGreenMin(config.g_limit_min);

    if (impl_.getBlueMin() != config.b_limit_min)
      impl_.setBlueMin(config.b_limit_min);

    impl_.updateCondition();
  }

  void HSVColorFilter::config_callback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);

    if (impl_.getHueMax() != config.h_limit_max)
      impl_.setHueMax(config.h_limit_max);

    if (impl_.getSaturationMax() != config.s_limit_max)
      impl_.setSaturationMax(config.s_limit_max);

    if (impl_.getValueMax() != config.v_limit_max)
      impl_.setValueMax(config.v_limit_max);

    if (impl_.getHueMin() != config.h_limit_min)
      impl_.setHueMin(config.h_limit_min);

    if (impl_.getSaturationMin() != config.s_limit_min)
      impl_.setSaturationMin(config.s_limit_min);

    if (impl_.getValueMin() != config.v_limit_min)
      impl_.setValueMin(config.v_limit_min);

    if (impl_.getUseHue() != config.use_h)
      impl_.setUseHue(config.use_h);

    impl_.updateCondition();
  }
}

typedef jsk_pcl_ros::RGBColorFilter RGBColorFilter;
typedef jsk_pcl_ros::HSVColorFilter HSVColorFilter;

PLUGINLIB_DECLARE_CLASS (jsk_pcl, RGBColorFilter, RGBColorFilter, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (jsk_pcl, HSVColorFilter, HSVColorFilter, nodelet::Nodelet);
