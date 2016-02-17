// -*- mode: c++ -*-
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

#ifndef JSK_PCL_ROS_HINTED_PLANE_DETECTOR_H_
#define JSK_PCL_ROS_HINTED_PLANE_DETECTOR_H_

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/HintedPlaneDetectorConfig.h>

namespace jsk_pcl_ros {
  
  class HintedPlaneDetector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef HintedPlaneDetectorConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2> SyncPolicy;
    HintedPlaneDetector(): DiagnosticNodelet("HintedPlaneDetector") {}
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void detect(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& hint_cloud_msg);
    virtual bool detectHintPlane(
      pcl::PointCloud<pcl::PointXYZ>::Ptr hint_cloud,
      jsk_recognition_utils::ConvexPolygon::Ptr& convex);
    virtual bool detectLargerPlane(
      pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,
      jsk_recognition_utils::ConvexPolygon::Ptr hint_convex);
    virtual pcl::PointIndices::Ptr getBestCluster(
      pcl::PointCloud<pcl::PointNormal>::Ptr input_cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      const jsk_recognition_utils::ConvexPolygon::Ptr hint_convex);
    virtual void publishPolygon(
      const jsk_recognition_utils::ConvexPolygon::Ptr convex,
      ros::Publisher& pub_polygon, ros::Publisher& pub_polygon_array,
      const pcl::PCLHeader& header);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void densityFilter(
      const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
      const pcl::PointIndices::Ptr indices,
      pcl::PointIndices& output);
    virtual void euclideanFilter(
      const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
      const pcl::PointIndices::Ptr indices,
      const jsk_recognition_utils::ConvexPolygon::Ptr hint_convex,
      pcl::PointIndices& output);
    virtual void planeFilter(
      const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
      const pcl::PointIndices::Ptr indices,
      const Eigen::Vector3f& normal,
      pcl::PointIndices& output,
      pcl::ModelCoefficients& coefficients);
    virtual void hintFilter(
      const pcl::PointCloud<pcl::PointNormal>::Ptr cloud,
      const jsk_recognition_utils::ConvexPolygon::Ptr hint_convex,
      pcl::PointIndices& output);

    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_hint_cloud_;
    ros::Publisher pub_hint_polygon_;
    ros::Publisher pub_hint_polygon_array_;
    ros::Publisher pub_hint_inliers_;
    ros::Publisher pub_hint_coefficients_;
    ros::Publisher pub_polygon_array_;
    ros::Publisher pub_polygon_;
    ros::Publisher pub_inliers_;
    ros::Publisher pub_coefficients_;
    ros::Publisher pub_hint_filtered_indices_;
    ros::Publisher pub_plane_filtered_indices_;
    ros::Publisher pub_density_filtered_indices_;
    ros::Publisher pub_euclidean_filtered_indices_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    double hint_outlier_threashold_;
    int hint_max_iteration_;
    int hint_min_size_;
    int max_iteration_;
    int min_size_;
    double outlier_threashold_;
    double eps_angle_;
    double normal_filter_eps_angle_;
    double euclidean_clustering_filter_tolerance_;
    int euclidean_clustering_filter_min_size_;
    bool enable_euclidean_filtering_;
    bool enable_normal_filtering_;
    bool enable_distance_filtering_;
    bool enable_density_filtering_;
    double density_radius_;
    int density_num_;
  };
}

#endif
