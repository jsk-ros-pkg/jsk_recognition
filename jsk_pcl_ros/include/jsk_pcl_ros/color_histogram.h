// -*- mode: C++ -*-
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
/*
 * color_histogram.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef JSK_PCL_ROS_COLOR_HISTOGRAM_H__
#define JSK_PCL_ROS_COLOR_HISTOGRAM_H__

#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/ColorHistogram.h>
#include <jsk_recognition_msgs/ColorHistogramArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_utils/pcl/color_histogram.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud2.h>

#include <jsk_pcl_ros/ColorHistogramConfig.h>


namespace jsk_pcl_ros
{
  class ColorHistogram : public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                      jsk_recognition_msgs::ClusterPointIndices> SyncPolicy;
    typedef ColorHistogramConfig Config;

    ColorHistogram() : DiagnosticNodelet("ColorHistogram") {}
  protected:
    virtual void onInit();
    virtual void configCallback(Config& config, uint32_t level);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void feature(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices);

    // properties
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    ros::Publisher pub_histogram_;

    // parameters
    int queue_size_;
    int bin_size_;
    double white_threshold_, black_threshold_;
    jsk_recognition_utils::HistogramPolicy histogram_policy_;
  };
}

#endif // JSK_PCL_ROS_COLOR_HISTOGRAM_H__
