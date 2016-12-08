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


#ifndef JSK_PCL_ROS_POINTCLOUD_LOCALIZATION
#define JSK_PCL_ROS_POINTCLOUD_LOCALIZATION
#include <tf_conversions/tf_eigen.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <tf/transform_broadcaster.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <jsk_recognition_msgs/UpdateOffset.h>

namespace jsk_pcl_ros
{
  class PointCloudLocalization: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    PointCloudLocalization():
      first_time_(true), localize_requested_(false),
      DiagnosticNodelet("PointCloudLocalization") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    /**
     * @brief
     * callback function of ~input topic.
     */
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /**
     * @brief
     * callback function of ~localize service.
     */
    virtual bool localizationRequest(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    
    /**
     * @brief
     * cloud periodic timer callback
     */
    virtual void cloudTimerCallback(
      const ros::TimerEvent& event);

    /**
     * @brief
     * tf periodic timer callback
     */
    virtual void tfTimerCallback(
      const ros::TimerEvent& event);
    
    /**
     * @brief
     * return true if it is the first time to localize
     */
    virtual bool isFirstTime();

    /**
     * @brief
     * callback function for ~update_offset service
     */
    virtual bool updateOffsetCallback(
      jsk_recognition_msgs::UpdateOffset::Request& req,
      jsk_recognition_msgs::UpdateOffset::Response& res);
    
    virtual void applyDownsampling(
      pcl::PointCloud<pcl::PointNormal>::Ptr in_cloud,
      pcl::PointCloud<pcl::PointNormal>& out_cloud);

    boost::mutex mutex_;
    boost::mutex tf_mutex_;
    ros::Subscriber sub_;
    ros::Publisher pub_cloud_;
    tf::TransformListener* tf_listener_;
    ros::ServiceServer localization_srv_;
    ros::ServiceServer update_offset_srv_;
    ros::Timer cloud_timer_;
    ros::Timer tf_timer_;
    pcl::PointCloud<pcl::PointNormal>::Ptr all_cloud_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud_;
    tf::TransformBroadcaster tf_broadcast_;
    bool localize_requested_;
    std::string sensor_frame_;
    bool clip_unseen_pointcloud_;
    bool initialize_from_tf_;
    std::string initialize_tf_;
    /**
     * @brief
     * Publishes tf transformation of global_frame_ -> odom_frame_
     */
    std::string global_frame_;
    std::string odom_frame_;

    /**
     * @brief
     * Resolution of voxel grid
     */
    bool use_normal_;
    double leaf_size_;
    tf::Transform localize_transform_;
    bool first_time_;
  private:
    
  };
}

#endif
