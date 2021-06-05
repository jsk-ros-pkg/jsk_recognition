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

#ifndef JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_
#define JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"
#include "jsk_recognition_utils/tf_listener_singleton.h"

#include "sensor_msgs/PointCloud2.h"
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "jsk_recognition_utils/pcl_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_topic_tools/diagnostic_nodelet.h"
#include "jsk_pcl_ros/ClusterPointIndicesDecomposerConfig.h"

namespace jsk_pcl_ros
{
  class ClusterPointIndicesDecomposer: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ClusterPointIndicesDecomposer(): DiagnosticNodelet("ClusterPointIndicesDecomposer") { }
    typedef jsk_pcl_ros::ClusterPointIndicesDecomposerConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices > ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::ModelCoefficientsArray> SyncAlignPolicy;
    virtual void onInit();
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &point,
                         const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices,
                         const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
                         const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients);
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &point,
                         const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices);
    virtual void sortIndicesOrder(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                  const std::vector<pcl::IndicesPtr> indices_array,
                                  std::vector<size_t>* argsort);
    void sortIndicesOrderByIndices(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                   const std::vector<pcl::IndicesPtr> indices_array,
                                   std::vector<size_t>* argsort);
    void sortIndicesOrderByZAxis(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                 const std::vector<pcl::IndicesPtr> indices_array,
                                 std::vector<size_t>* argsort);
    void sortIndicesOrderByCloudSize(const pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                     const std::vector<pcl::IndicesPtr> indices_array,
                                     std::vector<size_t>* argsort);
  protected:
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;

    void addToDebugPointCloud
    (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
     size_t i,
     pcl::PointCloud<pcl::PointXYZRGB>& debug_output);
    
    virtual bool computeCenterAndBoundingBox
    (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
     const std_msgs::Header header,
     const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
     const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
     geometry_msgs::Pose& center_pose_msg,
     jsk_recognition_msgs::BoundingBox& bounding_box);

    virtual bool transformPointCloudToAlignWithPlane(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud_transformed,
      const Eigen::Vector4f center,
      const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
      const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
      Eigen::Matrix4f& m4,
      Eigen::Quaternionf& q,
      int& nearest_plane_index);
    
    virtual int findNearestPlane(const Eigen::Vector4f& center,
                                 const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
                                 const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients);

    virtual void configCallback (Config &config, uint32_t level);
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void allocatePublishers(size_t num);
    virtual void publishNegativeIndices(
      const sensor_msgs::PointCloud2ConstPtr &input,
      const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices_input);
    virtual void subscribe();
    virtual void unsubscribe();
    
    static uint32_t colorRGBAToUInt32(std_msgs::ColorRGBA c)
    {
        uint8_t r, g, b;
        r = (uint8_t)(c.r * 255);
        g = (uint8_t)(c.g * 255);
        b = (uint8_t)(c.b * 255);
        return ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);
    }

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_target_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >async_;
    boost::shared_ptr<message_filters::Synchronizer<SyncAlignPolicy> >sync_align_;
    std::vector<ros::Publisher> publishers_;
    ros::Publisher pc_pub_, box_pub_, mask_pub_, label_pub_, centers_pub_, negative_indices_pub_, indices_pub_;
    boost::shared_ptr<tf::TransformBroadcaster> br_;
    std::string tf_prefix_;
    
    bool use_async_;
    int queue_size_;
    bool force_to_flip_z_axis_;
    bool publish_clouds_;
    bool publish_tf_;
    bool align_boxes_;
    bool align_boxes_with_plane_;
    std::string target_frame_id_;
    tf::TransformListener* tf_listener_;
    bool use_pca_;
    bool fill_boxes_label_with_nearest_plane_index_;
    int max_size_;
    int min_size_;
    std::string sort_by_;

    jsk_recognition_utils::Counter cluster_counter_;
    
  };

  class ClusterPointIndicesDecomposerZAxis: public ClusterPointIndicesDecomposer
  {
  public:
    virtual void onInit();
  };

}  // namespace jsk_pcl_ros

#endif  // JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_
