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


#ifndef JSK_PCL_ROS_GEOMETRIC_CONSISTENCY_GROUPING_H_
#define JSK_PCL_ROS_GEOMETRIC_CONSISTENCY_GROUPING_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <jsk_pcl_ros/GeometricConsistencyGroupingConfig.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

namespace jsk_pcl_ros
{
  /**
   * @brief
   * Nodelet implementation of jsk_pcl/GeometricConsistencyGrouping 
   */
  class GeometricConsistencyGrouping: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef GeometricConsistencyGroupingConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2> SyncPolicy;
    GeometricConsistencyGrouping():
      DiagnosticNodelet("GeometricConsistencyGrouping") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    virtual void recognize(
      const sensor_msgs::PointCloud2::ConstPtr& scene_cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& scene_feature_msg);

    virtual void referenceCallback(
      const sensor_msgs::PointCloud2::ConstPtr& model_cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& model_feature_msg);

    /**
     * @brief
     * callback for dynamic_reconfigure
     */
    virtual void configCallback(Config& config, uint32_t level);
    
    boost::mutex mutex_;
    ros::Publisher pub_output_;
    ros::Publisher pub_output_cloud_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_feature_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_reference_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_reference_feature_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > reference_sync_;
    
    pcl::PointCloud<pcl::SHOT352>::Ptr reference_feature_;
    pcl::PointCloud<pcl::PointNormal>::Ptr reference_cloud_;

    double gc_size_;
    double gc_thresh_;
  private:
  };
}

#endif
