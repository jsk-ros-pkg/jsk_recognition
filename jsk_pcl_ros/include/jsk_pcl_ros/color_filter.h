// -*- mode: C++ -*-
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


#ifndef JSK_PCL_ROS_COLOR_FILTER_H_
#define JSK_PCL_ROS_COLOR_FILTER_H_

#include <nodelet/nodelet.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/filters/conditional_removal.h>
#include "jsk_pcl_ros/RGBColorFilterConfig.h"
#include "jsk_pcl_ros/HSIColorFilterConfig.h"
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_topic_tools/connection_based_nodelet.h"

namespace jsk_pcl_ros
{
  class RGBColorFilter;
  class HSIColorFilter;

  template <class PackedComparison, typename Config>
  class ColorFilter: public jsk_topic_tools::ConnectionBasedNodelet
  {
    friend class RGBColorFilter;
    friend class HSIColorFilter;
  public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                      PCLIndicesMsg> SyncPolicy;
    typedef typename pcl::ConditionBase<pcl::PointXYZRGB>::Ptr ConditionPtr;
    typedef typename pcl::ComparisonBase<pcl::PointXYZRGB>::Ptr ComparisonPtr;
    typedef PackedComparison Comparison;

  protected:
    boost::mutex mutex_;
    pcl::ConditionalRemoval<pcl::PointXYZRGB> filter_instance_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<PCLIndicesMsg> sub_indices_;
    ros::Publisher pub_;
    sensor_msgs::PointCloud2 color_space_msg_;
    ros::Publisher color_space_pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    virtual void configCallback(Config &config, uint32_t level) = 0;
    virtual void updateCondition() = 0;
    virtual void convertToColorSpace(float &x, float &y, float &z,
                                     unsigned char r, unsigned char g, unsigned char b) = 0;
    virtual void filter(const sensor_msgs::PointCloud2ConstPtr &input);
    virtual void filter(const sensor_msgs::PointCloud2ConstPtr &input,
                        const PCLIndicesMsg::ConstPtr& indices);
    virtual void subscribe();
    virtual void unsubscribe();
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;

    bool use_indices_;
  private:
    virtual void onInit();
  };

  class RGBColorFilter: public ColorFilter<pcl::PackedRGBComparison<pcl::PointXYZRGB>, jsk_pcl_ros::RGBColorFilterConfig>
  {
  public:
  protected:
    int r_min_, r_max_, b_min_, b_max_, g_min_, g_max_;
    virtual void configCallback(jsk_pcl_ros::RGBColorFilterConfig &config, uint32_t level);
    virtual void updateCondition();
    virtual void convertToColorSpace(float &x, float &y, float &z,
                                     unsigned char r, unsigned char g, unsigned char b);
  private:
    virtual void onInit();
  };

  class HSIColorFilter: public ColorFilter<pcl::PackedHSIComparison<pcl::PointXYZRGB>, jsk_pcl_ros::HSIColorFilterConfig>
  {
  public:
  protected:
    int h_min_, h_max_, s_min_, s_max_, i_min_, i_max_;
    virtual void configCallback(jsk_pcl_ros::HSIColorFilterConfig &config, uint32_t level);
    virtual void updateCondition();
    virtual void convertToColorSpace(float &x, float &y, float &z,
                                     unsigned char r, unsigned char g, unsigned char b);
  private:
    virtual void onInit();
  };
}


#endif
