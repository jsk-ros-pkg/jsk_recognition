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


#ifndef JSK_PCL_ROS_ICP_REGISTRATION_H_
#define JSK_PCL_ROS_ICP_REGISTRATION_H_

#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/ICPRegistrationConfig.h>
#include <jsk_pcl_ros/BoundingBox.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_listener.h>
#include "jsk_pcl_ros/connection_based_nodelet.h"
#include <jsk_pcl_ros/PointsArray.h>

namespace jsk_pcl_ros
{
  class ICPRegistration: public ConnectionBasedNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef jsk_pcl_ros::ICPRegistrationConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      BoundingBox > SyncPolicy;
  protected:
    ////////////////////////////////////////////////////////
    // methosd
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void align(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void alignWithBox(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const BoundingBox::ConstPtr& box_msg);
    virtual void referenceCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void referenceArrayCallback(
      const PointsArray::ConstPtr& msg);
    virtual void configCallback (Config &config, uint32_t level);
    virtual void publishDebugCloud(
      ros::Publisher& pub,
      const pcl::PointCloud<PointT>& cloud);
    virtual double alignPointcloud(
      pcl::PointCloud<PointT>::Ptr& cloud,
      pcl::PointCloud<PointT>::Ptr& reference,
      const Eigen::Affine3f& offset,
      pcl::PointCloud<PointT>::Ptr& output_cloud,
      Eigen::Affine3d& output_transform);
    virtual double alignPointcloudWithReferences(
      pcl::PointCloud<PointT>::Ptr& cloud,
      const Eigen::Affine3f& offset,
      const std_msgs::Header& header);
    virtual double scorePointcloudAlignment(
      pcl::PointCloud<PointT>::Ptr& cloud,
      pcl::PointCloud<PointT>::Ptr& reference,
      const Eigen::Affine3f& offset,
      Eigen::Affine3f& offset_result,
      pcl::PointCloud<PointT>::Ptr transformed_cloud,
      Eigen::Affine3d& transform_result);

    virtual void subscribe();
    virtual void unsubscribe();
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Subscriber sub_reference_;
    ros::Subscriber sub_reference_array_;
    ros::Publisher pub_result_pose_;
    ros::Publisher pub_result_cloud_;
    ros::Publisher pub_debug_source_cloud_,
      pub_debug_target_cloud_,
      pub_debug_result_cloud_,
      pub_debug_flipped_cloud_;
    bool align_box_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<BoundingBox> sub_box_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;

    ////////////////////////////////////////////////////////
    // parameters for ICP
    ////////////////////////////////////////////////////////
    bool use_flipped_initial_pose_;
    int algorithm_;
    std::vector<pcl::PointCloud<PointT>::Ptr> reference_cloud_list_;
    int max_iteration_;
    double correspondence_distance_;
    double transform_epsilon_;
    double euclidean_fittness_epsilon_;
    ////////////////////////////////////////////////////////
    // parameters for GICP
    ////////////////////////////////////////////////////////
    double rotation_epsilon_;
    int correspondence_randomness_;
    int maximum_optimizer_iterations_;
  private:
    
  };
}

#endif
