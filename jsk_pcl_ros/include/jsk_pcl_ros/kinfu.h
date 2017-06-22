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

#ifndef JSK_PCL_ROS_KINFU_H_
#define JSK_PCL_ROS_KINFU_H_

#include <pcl/console/parse.h>
#include <pcl/gpu/kinfu_large_scale/kinfu.h>
#include <pcl/gpu/kinfu_large_scale/marching_cubes.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_pcl_ros/tf_listener_singleton.h>
#include <tf/transform_broadcaster.h>


namespace jsk_pcl_ros
{
  class Kinfu: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::CameraInfo,
      sensor_msgs::Image > SyncPolicy;

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    void initKinfu(const int height, const int width);
    void update(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg, const sensor_msgs::Image::ConstPtr& depth_msg);
    bool resetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    boost::shared_ptr<pcl::PolygonMesh> convertToMesh(const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles);
    bool saveMeshCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    pcl::gpu::kinfuLS::KinfuTracker* kinfu_;
    pcl::gpu::kinfuLS::MarchingCubes::Ptr marching_cubes_;

    int device_;
    int queue_size_;
    bool auto_reset_;
    bool is_kinfu_initialized_;

    boost::mutex mutex_;

    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

    ros::Publisher pub_rendered_image_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_camera_pose_;
    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_save_mesh_;

  private:
  };

}  // namespace jsk_pcl_ros

#endif  // JSK_PCL_ROS_KINFU_H_
