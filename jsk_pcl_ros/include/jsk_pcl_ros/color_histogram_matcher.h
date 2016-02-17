// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_COLOR_HISTOGRAM_H_
#define JSK_PCL_ROS_COLOR_HISTOGRAM_H_

#include <pcl_ros/pcl_nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include <jsk_recognition_msgs/ColorHistogram.h>
#include <jsk_recognition_msgs/ColorHistogramArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/ColorHistogramMatcherConfig.h>

#include "jsk_topic_tools/connection_based_nodelet.h"

namespace jsk_pcl_ros
{
  class ColorHistogramMatcher : public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime< sensor_msgs::PointCloud2,
                                                       jsk_recognition_msgs::ClusterPointIndices > SyncPolicy;
    typedef ColorHistogramMatcherConfig Config;
    enum ComparePolicy {
      USE_HUE,
      USE_SATURATION,
      USE_VALUE,
      USE_HUE_AND_SATURATION
    };
  protected:
    virtual void onInit();
    virtual void feature(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& input_indices);
    virtual void reference(
      const sensor_msgs::PointCloud2::ConstPtr& input_cloud);
    virtual void referenceHistogram(
      const jsk_recognition_msgs::ColorHistogram::ConstPtr& input_histogram);
    virtual void computeHistogram(const pcl::PointCloud<pcl::PointXYZHSV>& cloud,
                                  std::vector<float>& output,
                                  const ComparePolicy policy);
    virtual double bhattacharyyaCoefficient(const std::vector<float>& a,
                                            const std::vector<float>& b);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void subscribe();
    virtual void unsubscribe();
    boost::mutex mutex_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Subscriber reference_sub_;
    ros::Subscriber reference_histogram_sub_;
    ros::Publisher result_pub_;
    ros::Publisher all_histogram_pub_;
    ros::Publisher reference_histogram_pub_;
    ros::Publisher best_pub_;
    ros::Publisher coefficient_points_pub_;
    std::vector<float> reference_histogram_;
    bool reference_set_;
    double coefficient_thr_;
    int bin_size_;
    bool publish_colored_cloud_;
    int power_;
    double color_min_coefficient_;
    double color_max_coefficient_;
    int show_method_;
    // must be exclusive
    ComparePolicy policy_;
  private:
    
  };

}

#endif
