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


#ifndef JSK_PCL_ROS_FEATURE_REGISTRATION_H_
#define JSK_PCL_ROS_FEATURE_REGISTRATION_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/FeatureRegistrationConfig.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

namespace jsk_pcl_ros
{
  /**
   * @brief
   * Nodelet implementation of jsk_pcl/FeatureRegistration
   */
  class FeatureRegistration: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef FeatureRegistrationConfig Config;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2> SyncPolicy;
    FeatureRegistration(): DiagnosticNodelet("FeatureRegistration") {};
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    virtual void configCallback(Config &config, uint32_t level);
    
    virtual void referenceCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& feature_msg);

    virtual void estimate(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& feature_msg);

    boost::mutex mutex_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_cloud_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_feature_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_reference_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_reference_feature_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > reference_sync_;

    // parameters to store reference data
    pcl::PointCloud<pcl::PointNormal>::Ptr reference_cloud_;
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr reference_feature_;
    
    int max_iterations_;
    int correspondence_randomness_;
    double similarity_threshold_;
    double max_correspondence_distance_;
    double inlier_fraction_;
    double transformation_epsilon_;
  private:
    
  };
}

#endif
