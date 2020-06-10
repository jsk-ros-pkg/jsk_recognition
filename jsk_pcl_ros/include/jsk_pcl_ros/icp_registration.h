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


#ifndef JSK_PCL_ROS_ICP_REGISTRATION_H_
#define JSK_PCL_ROS_ICP_REGISTRATION_H_

#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/ICPRegistrationConfig.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/ICPAlignWithBox.h>
#include <jsk_recognition_msgs/ICPAlign.h>
#include <jsk_recognition_msgs/ICPResult.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_msgs/PointsArray.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_utils/time_util.h>
#include <std_msgs/Float32.h>

namespace jsk_pcl_ros
{
  class ICPRegistration: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef pcl::PointXYZRGBNormal PointT;
    typedef jsk_pcl_ros::ICPRegistrationConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::BoundingBox > SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      geometry_msgs::PoseStamped > OffsetSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2
      > ReferenceSyncPolicy;
    ICPRegistration(): timer_(10), done_init_(false) { }
  protected:
    ////////////////////////////////////////////////////////
    // methosd
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void align(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void align(const sensor_msgs::PointCloud2::ConstPtr& msg,
                       const sensor_msgs::PointCloud2::ConstPtr& reference_msg);
    virtual void alignWithBox(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg);
    virtual void alignWithOffset(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    virtual bool alignWithBoxService(
      jsk_recognition_msgs::ICPAlignWithBox::Request& req, 
      jsk_recognition_msgs::ICPAlignWithBox::Response& res);
    virtual bool alignService(
      jsk_recognition_msgs::ICPAlign::Request& req, 
      jsk_recognition_msgs::ICPAlign::Response& res);
    virtual void referenceCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void referenceArrayCallback(
      const jsk_recognition_msgs::PointsArray::ConstPtr& msg);
    virtual void referenceAddCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void configCallback (Config &config, uint32_t level);
    virtual void publishDebugCloud(
      ros::Publisher& pub,
      const pcl::PointCloud<PointT>& cloud,
      const std_msgs::Header& header);
    virtual double alignPointcloud(
      pcl::PointCloud<PointT>::Ptr& cloud,
      pcl::PointCloud<PointT>::Ptr& reference,
      const Eigen::Affine3f& offset,
      pcl::PointCloud<PointT>::Ptr& output_cloud,
      Eigen::Affine3d& output_transform);
    virtual double alignPointcloudWithICP(
      pcl::PointCloud<PointT>::Ptr& cloud,
      pcl::PointCloud<PointT>::Ptr& reference,
      const Eigen::Affine3f& offset,
      pcl::PointCloud<PointT>::Ptr& output_cloud,
      Eigen::Affine3d& output_transform);
    virtual double alignPointcloudWithNDT(
      pcl::PointCloud<PointT>::Ptr& cloud,
      pcl::PointCloud<PointT>::Ptr& reference,
      const Eigen::Affine3f& offset,
      pcl::PointCloud<PointT>::Ptr& output_cloud,
      Eigen::Affine3d& output_transform);
    virtual jsk_recognition_msgs::ICPResult alignPointcloudWithReferences(
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
    virtual void cameraInfoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& msg);
    virtual void subscribe();
    virtual void unsubscribe();
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_camera_info_;
    ros::Subscriber sub_;
    ros::Subscriber sub_reference_;
    ros::Subscriber sub_reference_add;
    ros::Subscriber sub_reference_array_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_sync_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_sync_reference_;
    ros::Publisher pub_result_pose_;
    ros::Publisher pub_result_cloud_;
    ros::Publisher pub_latest_time_;
    ros::Publisher pub_average_time_;
    ros::Publisher pub_debug_source_cloud_,
      pub_debug_target_cloud_,
      pub_debug_result_cloud_,
      pub_debug_flipped_cloud_;
    ros::Publisher pub_icp_result;
    jsk_recognition_utils::WallDurationTimer timer_;
    ros::ServiceServer srv_icp_align_with_box_;
    ros::ServiceServer srv_icp_align_;
    bool align_box_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> sub_box_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_offset_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<OffsetSyncPolicy> >sync_offset_;
    boost::shared_ptr<message_filters::Synchronizer<ReferenceSyncPolicy> > sync_reference_;
    tf::TransformListener* tf_listener_;

    /**
     * @brief
     * set via ~transform_3dof parameter. default is false.
     */
    bool transform_3dof_;
    /**
     * @brief
     * set via ~use_offset_pose parameter. default is false.
     */
    bool use_offset_pose_;

    /** @brief
     * Store value of ~use_normal.
     * If this parameter is true, ICPRegistration nodelet expects reference and input
     * pointcloud have normal_x, normal_y and normal_z fields. 
     */
    bool use_normal_;
    
    ////////////////////////////////////////////////////////
    // parameters for ICP
    ////////////////////////////////////////////////////////
    bool synchronize_reference_;
    bool use_flipped_initial_pose_;
    int algorithm_;
    int correspondence_algorithm_;
    std::vector<pcl::PointCloud<PointT>::Ptr> reference_cloud_list_;
    int max_iteration_;
    double correspondence_distance_;
    double transform_epsilon_;
    double euclidean_fittness_epsilon_;
    double ransac_iterations_;
    double ransac_outlier_threshold_;
    sensor_msgs::CameraInfo::ConstPtr camera_info_msg_;
    
    ////////////////////////////////////////////////////////
    // parameters for GICP
    ////////////////////////////////////////////////////////
    double rotation_epsilon_;
    int correspondence_randomness_;
    int maximum_optimizer_iterations_;

    ////////////////////////////////////////////////////////
    // parameters for NDT
    ////////////////////////////////////////////////////////
    double ndt_resolution_;
    double ndt_step_size_;
    double ndt_outlier_ratio_;

    bool done_init_;
  private:
    
  };
}

#endif
