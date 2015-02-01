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


#ifndef JSK_PCL_ROS_INCREMENTAL_MODEL_REGISTRATION_H_
#define JSK_PCL_ROS_INCREMENTAL_MODEL_REGISTRATION_H_

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <std_srvs/Empty.h>

namespace jsk_pcl_ros
{
  class CapturedSamplePointCloud
  {
  public:
    typedef boost::shared_ptr<CapturedSamplePointCloud> Ptr;
    CapturedSamplePointCloud();
    CapturedSamplePointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
               const Eigen::Affine3f& original_pose);
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getOriginalPointCloud();
    virtual Eigen::Affine3f getOriginalPose();
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr getRefinedPointCloud();
    virtual Eigen::Affine3f getRefinedPose();
    virtual void setRefinedPointCloud(
      pcl::PointCloud<pcl::PointXYZRGB> cloud);
    virtual void setRefinedPose(Eigen::Affine3f pose);
    
  protected:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr original_cloud_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr refined_cloud_;
    Eigen::Affine3f original_pose_;
    Eigen::Affine3f refined_pose_;
  private:
    
  };
  
  class IncrementalModelRegistration: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    IncrementalModelRegistration(): DiagnosticNodelet("IncrementalModelRegistration") {}
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      pcl_msgs::PointIndices,
      geometry_msgs::PoseStamped > SyncPolicy;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe() {}
    virtual void unsubscribe() {}
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat) {}
    virtual void newsampleCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const pcl_msgs::PointIndices::ConstPtr& indices_msg,
      const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    virtual void transformPointCloudRepsectedToPose(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr output,
      const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    virtual bool startRegistration(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual void callICP(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr reference,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr target,
      Eigen::Affine3f& output_transform);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<pcl_msgs::PointIndices> sub_indices_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_pose_;
    boost::mutex mutex_;
    ros::ServiceServer start_registration_srv_;
    ros::Publisher pub_cloud_non_registered_;
    ros::Publisher pub_registered_;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_;
    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    std::vector<CapturedSamplePointCloud::Ptr> samples_;
    Eigen::Affine3f origin_;
    pcl::PointCloud<pcl::PointXYZRGB> all_cloud_;
    std::string frame_id_;
  private:
    
  };
}

#endif
