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

#ifndef JSK_PCL_ROS_ORGANIZED_PLANE_SEGMENTATION_H_
#define JSK_PCL_ROS_ORGANIZED_PLANE_SEGMENTATION_H_

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/OrganizedMultiPlaneSegmentationConfig.h"
#include "jsk_pcl_ros/PolygonArray.h"
#include <jsk_topic_tools/time_accumulator.h>
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_pcl_ros/pcl_util.h"

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>

namespace jsk_pcl_ros
{
  class OrganizedMultiPlaneSegmentation: public pcl_ros::PCLNodelet
  {
  public:
    typedef pcl::PointXYZRGBA PointT;
  protected:
    ros::Publisher org_pub_, org_polygon_pub_, org_coefficients_pub_;
    ros::Publisher pub_, polygon_pub_, coefficients_pub_;
    ros::Publisher normal_pub_;
    ros::Subscriber sub_;
    int min_size_;
    double concave_alpha_;
    double angular_threshold_;
    double distance_threshold_;
    double max_curvature_;
    double connect_plane_angle_threshold_;
    double connect_plane_distance_threshold_;
    double connect_distance_threshold_;
    int estimation_method_;
    bool depth_dependent_smoothing_;
    double max_depth_change_factor_;
    double normal_smoothing_size_;
    bool border_policy_ignore_;
    bool estimate_normal_;
    bool publish_normal_;
    typedef jsk_pcl_ros::OrganizedMultiPlaneSegmentationConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    jsk_topic_tools::TimeAccumulator plane_segmentation_time_acc_;
    jsk_topic_tools::TimeAccumulator normal_estimation_time_acc_;
    jsk_topic_tools::VitalChecker::Ptr vital_checker_;
    ros::Timer diagnostics_timer_;
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void estimateNormal(pcl::PointCloud<PointT>::Ptr input,
                                pcl::PointCloud<pcl::Normal>::Ptr output);
    virtual void configCallback (Config &config, uint32_t level);
    virtual void pointCloudToPolygon(const pcl::PointCloud<PointT>& input,
                                     geometry_msgs::Polygon& polygon);
    virtual void pclIndicesArrayToClusterPointIndices(const std::vector<pcl::PointIndices>& inlier_indices,
                                                      const std_msgs::Header& header,
                                                      jsk_pcl_ros::ClusterPointIndices& output_indices);
    virtual void connectPlanesMap(const pcl::PointCloud<PointT>::Ptr& input,
                                  const std::vector<pcl::ModelCoefficients>& model_coefficients,
                                  const std::vector<pcl::PointIndices>& boundary_indices,
                                  std::vector<std::map<size_t, bool> >& connection_map);
    virtual void buildConnectedPlanes(const pcl::PointCloud<PointT>::Ptr& input,
                                      const std_msgs::Header& header,
                                      const std::vector<pcl::PointIndices>& inlier_indices,
                                      const std::vector<pcl::PointIndices>& boundary_indices,
                                      const std::vector<pcl::ModelCoefficients>& model_coefficients,
                                      std::vector<std::map<size_t, bool> > connection_map,
                                      std::vector<pcl::PointIndices>& output_indices,
                                      std::vector<pcl::ModelCoefficients>& output_coefficients,
                                      std::vector<pcl::PointCloud<PointT> >& output_boundary_clouds);
    
    virtual void segmentFromNormals(pcl::PointCloud<PointT>::Ptr input,
                                    pcl::PointCloud<pcl::Normal>::Ptr normal,
                                    const std_msgs::Header& header);
    virtual void updateDiagnostics(const ros::TimerEvent& event);
    virtual void updateDiagnosticNormalEstimation(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void updateDiagnosticPlaneSegmentation(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void updateDiagnosticConnectivity(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
  private:
    virtual void onInit();
  };
}

#endif
