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

#ifndef JSK_PCL_ROS_ORGANIZED_PLANE_SEGMENTATION_H_
#define JSK_PCL_ROS_ORGANIZED_PLANE_SEGMENTATION_H_

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/OrganizedMultiPlaneSegmentationConfig.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"
#include <jsk_topic_tools/time_accumulator.h>
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_topic_tools/diagnostic_utils.h>

namespace jsk_pcl_ros
{
  class OrganizedMultiPlaneSegmentation:
    public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef pcl::PointXYZRGBA PointT;
    typedef std::vector<pcl::PlanarRegion<PointT>,
                        Eigen::aligned_allocator<pcl::PlanarRegion<PointT> > >
    PlanarRegionVector;
    typedef jsk_pcl_ros::OrganizedMultiPlaneSegmentationConfig Config;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void estimateNormal(pcl::PointCloud<PointT>::Ptr input,
                                pcl::PointCloud<pcl::Normal>::Ptr output);
    virtual void configCallback (Config &config, uint32_t level);
    virtual void pointCloudToPolygon(const pcl::PointCloud<PointT>& input,
                                     geometry_msgs::Polygon& polygon);
    virtual void pclIndicesArrayToClusterPointIndices(const std::vector<pcl::PointIndices>& inlier_indices,
                                                      const std_msgs::Header& header,
                                                      jsk_recognition_msgs::ClusterPointIndices& output_indices);
    virtual void connectPlanesMap(const pcl::PointCloud<PointT>::Ptr& input,
                                  const std::vector<pcl::ModelCoefficients>& model_coefficients,
                                  const std::vector<pcl::PointIndices>& boundary_indices,
                                  jsk_recognition_utils::IntegerGraphMap& connection_map);
    virtual void buildConnectedPlanes(const pcl::PointCloud<PointT>::Ptr& input,
                                      const std_msgs::Header& header,
                                      const std::vector<pcl::PointIndices>& inlier_indices,
                                      const std::vector<pcl::PointIndices>& boundary_indices,
                                      const std::vector<pcl::ModelCoefficients>& model_coefficients,
                                      const jsk_recognition_utils::IntegerGraphMap& connection_map,
                                      std::vector<pcl::PointIndices>& output_indices,
                                      std::vector<pcl::ModelCoefficients>& output_coefficients,
                                      std::vector<pcl::PointCloud<PointT> >& output_boundary_clouds);
    virtual void forceToDirectOrigin(const std::vector<pcl::ModelCoefficients>& coefficients,
                                     std::vector<pcl::ModelCoefficients>& output_coefficients);
    virtual void publishMarkerOfConnection(
      jsk_recognition_utils::IntegerGraphMap connection_map,
      const pcl::PointCloud<PointT>::Ptr cloud,
      const std::vector<pcl::PointIndices>& inliers,
      const std_msgs::Header& header);

    virtual void segmentOrganizedMultiPlanes(
      pcl::PointCloud<PointT>::Ptr input,
      pcl::PointCloud<pcl::Normal>::Ptr normal,
      PlanarRegionVector& regions,
      std::vector<pcl::ModelCoefficients>& model_coefficients,
      std::vector<pcl::PointIndices>& inlier_indices,
      pcl::PointCloud<pcl::Label>::Ptr& labels,
      std::vector<pcl::PointIndices>& label_indices,
      std::vector<pcl::PointIndices>& boundary_indices);
    
    virtual void segmentFromNormals(pcl::PointCloud<PointT>::Ptr input,
                                    pcl::PointCloud<pcl::Normal>::Ptr normal,
                                    const std_msgs::Header& header);
    
    virtual void publishSegmentationInformation(
      const std_msgs::Header& header,
      const pcl::PointCloud<PointT>::Ptr input,
      ros::Publisher& indices_pub,
      ros::Publisher& polygon_pub,
      ros::Publisher& coefficients_pub,
      const std::vector<pcl::PointIndices>& inlier_indices,
      const std::vector<pcl::PointCloud<PointT> >& boundaries,
      const std::vector<pcl::ModelCoefficients>& model_coefficients);
    virtual void publishSegmentationInformation(
      const std_msgs::Header& header,
      const pcl::PointCloud<PointT>::Ptr input,
      ros::Publisher& indices_pub,
      ros::Publisher& polygon_pub,
      ros::Publisher& coefficients_pub,
      const std::vector<pcl::PointIndices>& inlier_indices,
      const std::vector<pcl::PointIndices>& boundary_indices,
      const std::vector<pcl::ModelCoefficients>& model_coefficients);
    
    virtual void refineBasedOnRANSAC(
      const pcl::PointCloud<PointT>::Ptr input,
      const std::vector<pcl::PointIndices>& input_indices,
      const std::vector<pcl::ModelCoefficients>& input_coefficients,
      std::vector<pcl::PointIndices>& output_indices,
      std::vector<pcl::ModelCoefficients>& output_coefficients,
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& output_boundaries);

    
    virtual void updateDiagnostics(const ros::TimerEvent& event);
    virtual void updateDiagnosticNormalEstimation(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void updateDiagnosticPlaneSegmentation(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void subscribe();
    virtual void unsubscribe();
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Publisher org_pub_, org_polygon_pub_, org_coefficients_pub_;
    ros::Publisher pub_, polygon_pub_, coefficients_pub_;
    ros::Publisher refined_pub_, refined_polygon_pub_, refined_coefficients_pub_;
    ros::Publisher normal_pub_;
    ros::Publisher pub_connection_marker_;
    ros::Subscriber sub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    jsk_topic_tools::TimeAccumulator plane_segmentation_time_acc_;
    jsk_topic_tools::TimeAccumulator normal_estimation_time_acc_;
    jsk_topic_tools::TimeAccumulator ransac_refinement_time_acc_;
    jsk_topic_tools::VitalChecker::Ptr normal_estimation_vital_checker_;
    jsk_topic_tools::VitalChecker::Ptr plane_segmentation_vital_checker_;
    ros::Timer diagnostics_timer_;
    
    int min_size_;
    double concave_alpha_;
    double angular_threshold_;
    double distance_threshold_;
    double max_curvature_;
    double connect_plane_angle_threshold_;
    double connect_distance_threshold_;
    double min_refined_area_threshold_;
    double max_refined_area_threshold_;
    int estimation_method_;
    bool depth_dependent_smoothing_;
    double max_depth_change_factor_;
    double normal_smoothing_size_;
    bool border_policy_ignore_;
    bool estimate_normal_;
    bool publish_normal_;

    ////////////////////////////////////////////////////////
    // parameters for RANSAC refinement
    ////////////////////////////////////////////////////////
    bool ransac_refine_coefficients_;
    double ransac_refine_outlier_distance_threshold_;
    
    jsk_recognition_utils::Counter original_plane_num_counter_;
    jsk_recognition_utils::Counter connected_plane_num_counter_;

    
  private:
    virtual void onInit();
  };
}

#endif
