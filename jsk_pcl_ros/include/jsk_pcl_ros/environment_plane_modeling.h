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

#ifndef JSK_PCL_ROS_ENVIRONMENT_PLANE_MODELING_H_
#define JSK_PCL_ROS_ENVIRONMENT_PLANE_MODELING_H_

#include <pcl_ros/pcl_nodelet.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <dynamic_reconfigure/server.h>

#include <jsk_pcl_ros/PolygonArray.h>
#include <jsk_pcl_ros/ModelCoefficientsArray.h>
#include <jsk_pcl_ros/ClusterPointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/EnvironmentLock.h>
#include <jsk_pcl_ros/PolygonOnEnvironment.h>

#include <jsk_pcl_ros/pcl_conversion_util.h>
#include <jsk_pcl_ros/EnvironmentPlaneModelingConfig.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

#include <std_srvs/Empty.h>

#include <jsk_topic_tools/time_accumulator.h>

#include "jsk_pcl_ros/grid_map.h"
#include "jsk_pcl_ros/pcl_util.h"

namespace jsk_pcl_ros
{
  class EnvironmentPlaneModeling: public pcl_ros::PCLNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef EnvironmentPlaneModelingConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      ClusterPointIndices,
      PolygonArray,
      ModelCoefficientsArray,
      PolygonArray,
      ModelCoefficientsArray> SyncPolicy;
  protected:
    virtual void onInit();
    virtual void estimateOcclusion(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
      const ClusterPointIndices::ConstPtr& input_indices,
      const std::vector<pcl::PointCloud<PointT>::Ptr>& segmented_cloud,
      std::vector<GridMap::Ptr>& grid_maps,
      const PolygonArray::ConstPtr& polygons,
      const ModelCoefficientsArray::ConstPtr& coefficients,
      const PolygonArray::ConstPtr& static_polygons,
      const ModelCoefficientsArray::ConstPtr& static_coefficients,
      PolygonArray::Ptr result_polygons,
      ModelCoefficientsArray::Ptr result_coefficients,
      pcl::PointCloud<PointT>::Ptr result_pointcloud,
      ClusterPointIndices::Ptr result_indices);
    
    virtual void inputCallback(
      const sensor_msgs::PointCloud2::ConstPtr& input,
      const ClusterPointIndices::ConstPtr& input_indices,
      const PolygonArray::ConstPtr& polygons,
      const ModelCoefficientsArray::ConstPtr& coefficients,
      const PolygonArray::ConstPtr& static_polygons,
      const ModelCoefficientsArray::ConstPtr& static_coefficients);
    virtual void configCallback(Config &config, uint32_t level);
    virtual bool lockCallback();
    virtual bool dummyLockCallback(EnvironmentLock::Request& req,
                                   EnvironmentLock::Response& res);
    virtual bool polygonOnEnvironmentCallback(PolygonOnEnvironment::Request& req,
                                              PolygonOnEnvironment::Response& res);
    virtual bool primitiveLockCallback(std_srvs::Empty::Request& req,
                                       std_srvs::Empty::Response& res);
    virtual bool primitiveUnlockCallback(std_srvs::Empty::Request& req,
                                         std_srvs::Empty::Response& res);
    virtual bool polygonNearEnoughToPointCloud(
      const size_t plane_i,
      const pcl::PointCloud<PointT>::Ptr sampled_point_cloud);
    virtual void samplePolygonToPointCloud(
      const geometry_msgs::PolygonStamped sample_polygon,
      pcl::PointCloud<PointT>::Ptr output,
      double sampling_param);
    
    virtual void decomposePointCloud(
      const pcl::PointCloud<PointT>::Ptr& input,
      const ClusterPointIndices::ConstPtr& input_indices,
      std::vector<pcl::PointCloud<PointT>::Ptr>& output);
    virtual void internalPointDivide(const PointT& A, const PointT& B,
                                     const double ratio,
                                     PointT& output);

    virtual void extendConvexPolygon(
      const geometry_msgs::PolygonStamped& static_polygon,
      const PCLModelCoefficientMsg& coefficients,
      const geometry_msgs::PolygonStamped& nearest_polygon,
      geometry_msgs::PolygonStamped& output_polygon);
    virtual void updateAppendingInfo(const int env_plane_index,
                                     const size_t static_plane_index,
                                     std::map<int, std::set<size_t> >& result);
    virtual void buildGridMap(
      const std::vector<pcl::PointCloud<PointT>::Ptr>& segmented_clouds,
      const PolygonArray::ConstPtr& polygons,
      const ModelCoefficientsArray::ConstPtr& coefficients,
      std::vector<GridMap::Ptr>& grid_maps);
    virtual void publishGridMap(
      const std_msgs::Header& header,
      const std::vector<GridMap::Ptr> grid_maps);
    // find the nearest plane to static_polygon and static_coefficient
    // from polygons and coefficients
    virtual int findNearestPolygon(
      const PolygonArray::ConstPtr& polygons,
      const ModelCoefficientsArray::ConstPtr& coefficients,
      const geometry_msgs::PolygonStamped& static_polygon,
      const PCLModelCoefficientMsg& static_coefficient);

    virtual void fillEstimatedRegionByPointCloud
    (const std_msgs::Header& header,
     const pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
     const ClusterPointIndices::ConstPtr& indices,
     const PolygonArray::ConstPtr& polygons,
     const ModelCoefficientsArray::ConstPtr& coefficients,
     const PolygonArray::ConstPtr& static_polygons,
     const ModelCoefficientsArray::ConstPtr& static_coefficients,
     const PolygonArray& result_polygons,
     const std::map<int, std::set<size_t> >& estimation_summary,
     pcl::PointCloud<PointT>::Ptr all_cloud,
     ClusterPointIndices& all_indices,
     std::vector<GridMap::Ptr> grid_maps);
    
    virtual void copyClusterPointIndices
    (const ClusterPointIndices::ConstPtr& indices,
     ClusterPointIndices& output);
    virtual void computePolygonCentroid(
      const geometry_msgs::PolygonStamped msg,
      pcl::PointXYZRGB& output);
    virtual void addIndices(const size_t start, const size_t end,
                            PCLIndicesMsg& output);
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    // for historical_accumulation_
    virtual int findCorrespondGridMap(
      const std::vector<float>& coefficients,
      const geometry_msgs::Polygon& polygon);
    virtual void registerGridMap(const GridMap::Ptr new_grid_map);
    virtual void selectionGridMaps();
    
    boost::mutex mutex_;

    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    // synchronized subscription
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<PolygonArray> sub_polygons_;
    message_filters::Subscriber<ModelCoefficientsArray> sub_coefficients_;
    message_filters::Subscriber<PolygonArray> sub_static_polygons_;
    message_filters::Subscriber<ModelCoefficientsArray> sub_static_coefficients_;
    
    ros::ServiceServer lock_service_;
    ros::ServiceServer polygon_on_environment_service_;
    ros::ServiceServer primitive_lock_service_;
    ros::ServiceServer primitive_unlock_service_;
    ros::Publisher debug_polygon_pub_;
    ros::Publisher debug_env_polygon_pub_;
    ros::Publisher debug_pointcloud_pub_;
    ros::Publisher debug_env_pointcloud_pub_;
    ros::Publisher occlusion_result_polygons_pub_;
    ros::Publisher occlusion_result_coefficients_pub_;
    ros::Publisher occlusion_result_pointcloud_pub_;
    ros::Publisher occlusion_result_indices_pub_;
    ros::Publisher grid_map_array_pub_;
    // member variables to store the latest messages
    sensor_msgs::PointCloud2::ConstPtr latest_input_;
    ClusterPointIndices::ConstPtr latest_input_indices_;
    PolygonArray::ConstPtr latest_input_polygons_;
    ModelCoefficientsArray::ConstPtr latest_input_coefficients_;
    PolygonArray::ConstPtr latest_static_polygons_;
    ModelCoefficientsArray::ConstPtr latest_static_coefficients_;
    
    // member variables to keep the messasge which we process
    sensor_msgs::PointCloud2::ConstPtr processing_input_;
    ClusterPointIndices::ConstPtr processing_input_indices_;
    PolygonArray::ConstPtr processing_input_polygons_;
    ModelCoefficientsArray::ConstPtr processing_input_coefficients_;
    PolygonArray::ConstPtr processing_static_polygons_;
    ModelCoefficientsArray::ConstPtr processing_static_coefficients_;
    
    std::vector<pcl::KdTreeFLANN<PointT>::Ptr> kdtrees_;
    std::vector<pcl::PointCloud<PointT>::Ptr> separated_point_cloud_;
    uint32_t environment_id_;
    double distance_thr_;
    double sampling_d_;
    double resolution_size_;
    // parameters for occlusion
    double plane_distance_threshold_;
    double plane_angle_threshold_;
    // grid map
    double grid_map_distance_threshold_;
    double grid_map_angle_threshold_;
    bool continuous_estimation_;
    bool history_accumulation_;
    bool history_statical_rejection_;
    int static_generation_;
    int required_vote_;
    std::vector<GridMap::Ptr> grid_maps_;
    jsk_topic_tools::TimeAccumulator occlusion_estimate_time_acc_;
    jsk_topic_tools::TimeAccumulator grid_building_time_acc_;
    jsk_topic_tools::TimeAccumulator kdtree_building_time_acc_;
    jsk_topic_tools::TimeAccumulator polygon_collision_check_time_acc_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    int generation_;
  private:
  };
}

#endif
