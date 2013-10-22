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

// -*- mode: c++ -*-
#ifndef COLOR_FILTER_NODELET_H_
#define COLOR_FILTER_NODELET_H_

// PCL includes
#include "jsk_pcl_ros/color_filter.h"

#include "jsk_pcl_ros/filter.h"

#include "jsk_pcl_ros/RGBColorFilterConfig.h"
#include "jsk_pcl_ros/HSVColorFilterConfig.h"

namespace jsk_pcl_ros
{
  class RGBColorFilter : public Filter
  {
  protected:
    typedef jsk_pcl_ros::RGBColorFilterConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;

    inline void
    filter (const PointCloud2::ConstPtr &input,
            const IndicesPtr &indices, PointCloud2 &output)
    {
      boost::mutex::scoped_lock lock (mutex_);
      impl_.setInputCloud (input);
      impl_.setIndices (indices);
      impl_.filter (output);
    }

    bool child_init (ros::NodeHandle &nh, bool &has_service)
    {
      has_service = true;
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&RGBColorFilter::config_callback, this, _1, _2);
      srv_->setCallback (f);

      return true;
    }

    void config_callback (Config &config, uint32_t level);

    pcl::RGBColorFilter<POINT_CLOUD2_TYPE> impl_;
  public:
    void onInit ()
    {
      Filter::onInit ();
      double r_max, r_min, g_max, g_min, b_max, b_min;

      pnh_->param<double>("r_limit_max", r_max, 255);
      pnh_->param<double>("g_limit_max", g_max, 255);
      pnh_->param<double>("b_limit_max", b_max, 255);
      pnh_->param<double>("r_limit_min", r_min, 0);
      pnh_->param<double>("g_limit_min", g_min, 0);
      pnh_->param<double>("b_limit_min", b_min, 0);

      impl_.setRedMax(r_max);
      impl_.setGreenMax(g_max);
      impl_.setBlueMax(b_max);
      impl_.setRedMin(r_min);
      impl_.setGreenMin(g_min);
      impl_.setBlueMin(b_min);

      impl_.updateCondition();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  class HSVColorFilter : public Filter
  {
  protected:
    typedef jsk_pcl_ros::HSVColorFilterConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;

    inline void
    filter (const PointCloud2::ConstPtr &input,
            const IndicesPtr &indices, PointCloud2 &output)
    {
      boost::mutex::scoped_lock lock (mutex_);
      impl_.setInputCloud (input);
      impl_.setIndices (indices);
      impl_.filter (output);
    }

    bool child_init (ros::NodeHandle &nh, bool &has_service)
    {
      has_service = true;
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&HSVColorFilter::config_callback, this, _1, _2);
      srv_->setCallback (f);

      return true;
    }

    void config_callback (Config &config, uint32_t level);

    pcl::HSVColorFilter<POINT_CLOUD2_TYPE> impl_;
  public:
    void onInit ()
    {
      Filter::onInit ();
      double h_max, h_min, s_max, s_min, v_max, v_min;
      bool use_h;
      pnh_->param<double>("h_limit_max", h_max, 127);
      pnh_->param<double>("s_limit_max", s_max, 255);
      pnh_->param<double>("v_limit_max", v_max, 255);
      pnh_->param<double>("h_limit_min", h_min, -128);
      pnh_->param<double>("s_limit_min", s_min, 0);
      pnh_->param<double>("v_limit_min", v_min, 0);
      pnh_->param<bool>("use_h", use_h, true);

      impl_.setHueMax(h_max);
      impl_.setSaturationMax(s_max);
      impl_.setValueMax(v_max);
      impl_.setHueMin(h_min);
      impl_.setSaturationMin(s_min);
      impl_.setValueMin(v_min);

      impl_.updateCondition();
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif
