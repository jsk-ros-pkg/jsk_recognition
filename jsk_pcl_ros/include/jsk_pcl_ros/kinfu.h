/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Kentaro Wada and JSK Lab
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
 *   * Neither the name of Kentaro Wada and JSK Lab nor the names of its
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
#include <pcl/gpu/kinfu_large_scale/raycaster.h>
#include <pcl/gpu/containers/initialization.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/angles.h>

#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <pcl_conversions/pcl_conversions.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <jsk_pcl_ros/tf_listener_singleton.h>
#include <tf/transform_broadcaster.h>

#include "jsk_pcl_ros/KinfuConfig.h"

// defined in pcl/gpu/kinfu_large_scale/src/kinfu.cpp
namespace pcl
{
  namespace gpu
  {
    namespace kinfuLS
    {
      void paint3DView (const KinfuTracker::View& rgb24, KinfuTracker::View& view, float colors_weight = 0.5f);
    }
  }
}

namespace jsk_pcl_ros
{
  class Kinfu: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::CameraInfo, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::Image> SyncPolicyWithColor;
    typedef jsk_pcl_ros::KinfuConfig Config;

    Kinfu(): ConnectionBasedNodelet(), frame_idx_(0) {}
    ~Kinfu() {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();

    void initKinfu(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg);
    void update(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg,
                const sensor_msgs::Image::ConstPtr& depth_msg);
    void update(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg,
                const sensor_msgs::Image::ConstPtr& depth_msg,
                const sensor_msgs::Image::ConstPtr& rgb_msg);
    bool resetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    virtual void configCallback(Config &config, uint32_t level);

    pcl::PolygonMesh createPolygonMesh();
    pcl::PolygonMesh createPolygonMesh(const jsk_recognition_msgs::BoundingBox& box_msg,
                                       const std::string& ground_frame_id);
    pcl::TextureMesh convertToTextureMesh(const pcl::PolygonMesh& triangles,
                                          const std::vector<cv::Mat> textures,
                                          pcl::texture_mapping::CameraVector cameras);
    bool saveMeshCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    bool saveMeshWithContextCallback(
      jsk_recognition_msgs::SaveMesh::Request& req, jsk_recognition_msgs::SaveMesh::Response& res);

    boost::shared_ptr<pcl::gpu::kinfuLS::KinfuTracker> kinfu_;
    pcl::gpu::kinfuLS::MarchingCubes::Ptr marching_cubes_;
    pcl::gpu::kinfuLS::KinfuTracker::View colors_device_;
    pcl::gpu::kinfuLS::RayCaster::Ptr raycaster_;
    std::vector<cv::Mat> textures_;
    pcl::texture_mapping::CameraVector cameras_;

    int device_;
    bool auto_reset_;
    bool integrate_color_;
    bool slam_;
    std::string fixed_frame_id_;
    int n_textures_;
    float volume_size_;

    int frame_idx_;
    std::string save_dir_;

    boost::mutex mutex_;

    boost::shared_ptr<tf::TransformListener> tf_listener_;
    Eigen::Affine3f odom_init_to_kinfu_origin_;

    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_camera_info_;
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_;
    message_filters::Subscriber<sensor_msgs::Image> sub_color_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicyWithColor> > sync_with_color_;

    ros::Publisher pub_camera_pose_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_rendered_image_;
    ros::Publisher pub_status_;

    tf::TransformBroadcaster tf_broadcaster_;

    ros::ServiceServer srv_reset_;
    ros::ServiceServer srv_save_mesh_;
    ros::ServiceServer srv_save_mesh_with_context_;

  private:
  };

}  // namespace jsk_pcl_ros

#endif  // JSK_PCL_ROS_KINFU_H_
