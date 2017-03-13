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

#ifndef _JSK_PCL_ROS_ELEMENT_MATCHING_POSE_ESTIMATION_H_
#define _JSK_PCL_ROS_ELEMENT_MATCHING_POSE_ESTIMATION_H_

#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_recognition_msgs/EdgeArray.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <jsk_recognition_msgs/PoseLabeledArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"


#include "jsk_pcl_ros/pcl/transformation_estimation_point_to_element.h"


namespace jsk_pcl_ros
{
  class ElementMatchingPoseEstimation: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef std::vector<Eigen::Vector3f,
                        Eigen::aligned_allocator<Eigen::Vector3f> > Vertices;

    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::EdgeArray,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::PoseLabeledArray > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::EdgeArray,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::PoseLabeledArray > ApproximateSyncPolicy;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void estimate(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& edge_indices_msg,
      const jsk_recognition_msgs::EdgeArray::ConstPtr& edges_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& poly_indices_msg,
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polys_msg,
      const jsk_recognition_msgs::PoseLabeledArray::ConstPtr& poses_msg);
    virtual bool reset_estimated_pose_cb(
      jsk_recognition_msgs::ResetPose::Request &req,
      jsk_recognition_msgs::ResetPose::Response &res);
    virtual void tfTimerCallback(
      const ros::TimerEvent& event);
    virtual geometry_msgs::Point toPoint(geometry_msgs::Point32 pt);

    std::string frame_id_;
    int max_queue_size_;
    bool approximate_sync_;
    bool debug_viewer_;
    bool publish_tf_;
    std::string output_frame_id_;
    bool publish_model_element_;
    int max_edge_indices_;
    int max_polygon_indices_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >async_;

    ros::Publisher pub_;
    ros::Publisher pub_edges_;
    ros::Publisher pub_polys_;
    ros::Publisher pub_poses_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_edge_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::EdgeArray> sub_edges_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_poly_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polys_;
    message_filters::Subscriber<jsk_recognition_msgs::PoseLabeledArray> sub_poses_;
    ros::ServiceClient cli_element_corresp_;
    ros::ServiceServer srv_reset_pose_;
    tf::TransformListener* tf_listener_;
    ros::Timer tf_timer_;

    pcl::registration::TransformationEstimationPointToElement<pcl::PointXYZ, pcl::PointElement> trans_est_;
    geometry_msgs::PoseStamped estimated_pose_;
    tf::Transform estimated_tf_;

    boost::mutex tf_mutex_;

  private:
  };

}

#endif
