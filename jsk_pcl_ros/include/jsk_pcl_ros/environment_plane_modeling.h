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

#ifndef JSK_PCL_ROS_ENVIRONMENT_PLANE_MODELING_H_
#define JSK_PCL_ROS_ENVIRONMENT_PLANE_MODELING_H_

#include <pcl_ros/pcl_nodelet.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/EnvironmentLock.h>
#include <jsk_recognition_msgs/PolygonOnEnvironment.h>

#include <jsk_recognition_utils/pcl_conversion_util.h>
#include <jsk_pcl_ros/EnvironmentPlaneModelingConfig.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <std_srvs/Empty.h>

#include <jsk_topic_tools/time_accumulator.h>

#include "jsk_recognition_utils/pcl_util.h"
#include <jsk_recognition_msgs/SimpleOccupancyGridArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_pcl_ros/tf_listener_singleton.h"

namespace jsk_pcl_ros
{

  // Helper classes

  /**
   * @brief
   * Nodelet implementation of jsk_pcl/EnvironmentPlaneModeling
   */
  class EnvironmentPlaneModeling: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef EnvironmentPlaneModelingConfig Config;
    
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::ModelCoefficientsArray,
      jsk_recognition_msgs::ClusterPointIndices > SyncPolicy;
    EnvironmentPlaneModeling(): DiagnosticNodelet("EnvironmentPlaneModeling") {}
  protected:
    virtual void onInit();

    /**
     * @brief
     * subscription callback function of jsk_topic_tools::DiagnosticNodelet.
     * This method is empty method because EnvironmentPlaneModeling needs to always run
     */
    virtual void subscribe() {}

    /**
     * @brief
     * unsubscription callback function of jsk_topic_tools::DiagnosticNodelet.
     * This method is empty method because EnvironmentPlaneModeling needs to always run
     */
    virtual void unsubscribe() {}

    /**
     * @brief
     * main callback function
     */
    virtual void inputCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& full_cloud_msg,
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg);

    virtual void printInputData(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& full_cloud_msg,
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg);


    virtual bool isValidFrameIds(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& full_cloud_msg,
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg);

    virtual std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convertToConvexPolygons(
      const pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg);

    virtual void publishConvexPolygonsBoundaries(
      ros::Publisher& pub,
      const std_msgs::Header& header,
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes);
    
    /**
     * @brief
     * Callback method of dynamic reconfigure
     */
    virtual void configCallback(Config &config, uint32_t level);

    /**
     * @brief
     * Publish array of jsk_recognition_utils::ConvexPolygon::Ptr by using specified publisher
     */
    virtual void publishConvexPolygons(
      ros::Publisher& pub,
      const std_msgs::Header& header,
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes);
    
    /**
     * @brief
     * Publish array of GridPlane::Ptr by using specified publisher
     */
    virtual void publishGridMaps(
      ros::Publisher& pub,
      const std_msgs::Header& header,
      std::vector<GridPlane::Ptr>& grids);
    
    /**
     * @brief
     * Magnify jsk_recognition_utils::ConvexPolygons according to maginify_distance_ parameter.
     */
    virtual std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> magnifyConvexes(
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes);

    /**
     * @brief
     * make GridPlane from jsk_recognition_utils::ConvexPolygon and PointCloud
     */
    virtual std::vector<GridPlane::Ptr> buildGridPlanes(
      pcl::PointCloud<pcl::PointNormal>::Ptr& cloud,
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes,
      std::set<int>& non_plane_indices);

    virtual std::vector<GridPlane::Ptr> morphologicalFiltering(
      std::vector<GridPlane::Ptr>& raw_grid_maps);

    virtual void boundingBoxCallback(
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box_array);

    virtual std::vector<GridPlane::Ptr> completeFootprintRegion(
      const std_msgs::Header& header,
      std::vector<GridPlane::Ptr>& grid_maps);
    
    virtual std::vector<GridPlane::Ptr> erodeFiltering(
      std::vector<GridPlane::Ptr>& grid_maps);

    virtual int lookupGroundPlaneForFootprint(
      const std::string& footprint_frame_id, const std_msgs::Header& header,
      const std::vector<GridPlane::Ptr>& grid_maps);
    
    virtual int lookupGroundPlaneForFootprint(
      const Eigen::Affine3f& pose, const std::vector<GridPlane::Ptr>& grid_maps);

    virtual GridPlane::Ptr completeGridMapByBoundingBox(
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box,
      const std_msgs::Header& header,
      GridPlane::Ptr grid_map);

    virtual void moveBaseSimpleGoalCallback(
      const geometry_msgs::PoseStamped::ConstPtr& msg);
    
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_full_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    ros::Subscriber sub_leg_bbox_;
    ros::Subscriber sub_move_base_simple_goal_;
    ros::Publisher pub_debug_magnified_polygons_;
    ros::Publisher pub_debug_convex_point_cloud_;
    ros::Publisher pub_debug_raw_grid_map_;
    ros::Publisher pub_debug_noeroded_grid_map_;
    ros::Publisher pub_debug_plane_coords_;
    ros::Publisher pub_debug_magnified_plane_coords_;
    ros::Publisher pub_grid_map_;
    ros::Publisher pub_non_plane_indices_;
    ros::Publisher pub_snapped_move_base_simple_goal_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    tf::TransformListener* tf_listener_;
    jsk_recognition_msgs::BoundingBox::ConstPtr latest_leg_bounding_box_;
    std::vector<std::string> footprint_frames_;
    std::vector<GridPlane::Ptr> latest_grid_maps_;
    std_msgs::Header latest_global_header_;
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    double magnify_distance_;
    double distance_threshold_;
    double normal_threshold_;
    double resolution_;
    int morphological_filter_size_;
    bool complete_footprint_region_;
    int erode_filter_size_;
    double footprint_plane_distance_threshold_;
    double footprint_plane_angular_threshold_;
  private:
  };
}

#endif
