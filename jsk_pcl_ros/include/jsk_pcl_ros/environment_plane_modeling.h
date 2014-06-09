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

#include <jsk_pcl_ros/PolygonArray.h>
#include <jsk_pcl_ros/ModelCoefficientsArray.h>
#include <jsk_pcl_ros/ClusterPointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/EnvironmentLock.h>
#include <jsk_pcl_ros/PolygonOnEnvironment.h>

namespace jsk_pcl_ros
{
  class EnvironmentPlaneModeling: public pcl_ros::PCLNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef message_filters::sync_policies::ExactTime< sensor_msgs::PointCloud2,
                                                       jsk_pcl_ros::ClusterPointIndices,
                                                       jsk_pcl_ros::PolygonArray,
                                                       jsk_pcl_ros::ModelCoefficientsArray> SyncPolicy;
  protected:
    virtual void onInit();
    virtual void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input,
                               const jsk_pcl_ros::ClusterPointIndices::ConstPtr& input_indices,
                               const jsk_pcl_ros::PolygonArray::ConstPtr& input_polygons,
                               const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& input_coefficients);
    virtual bool lockCallback(jsk_pcl_ros::EnvironmentLock::Request& req,
                              jsk_pcl_ros::EnvironmentLock::Response& res);
    virtual bool polygonOnEnvironmentCallback(jsk_pcl_ros::PolygonOnEnvironment::Request& req,
                                              jsk_pcl_ros::PolygonOnEnvironment::Response& res);
    virtual bool polygonNearEnoughToPointCloud(
      const size_t plane_i,
      const pcl::PointCloud<PointT>::Ptr sampled_point_cloud);
    virtual void samplePolygonToPointCloud(
      const geometry_msgs::PolygonStamped sample_polygon,
      pcl::PointCloud<PointT>::Ptr output,
      double sampling_param);
    virtual void internalPointDivide(const PointT& A, const PointT& B,
                                     const double ratio,
                                     PointT& output);
    void msgToPCL(const geometry_msgs::Point32 msg_point, PointT& pcl_point);
    
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_pcl_ros::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_pcl_ros::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_pcl_ros::ModelCoefficientsArray> sub_coefficients_;
    ros::ServiceServer lock_service_;
    ros::ServiceServer polygon_on_environment_service_;

    ros::Publisher debug_polygon_pub_;
    ros::Publisher debug_env_polygon_pub_;
    ros::Publisher debug_pointcloud_pub_;
    ros::Publisher debug_env_pointcloud_pub_;
    
    sensor_msgs::PointCloud2::ConstPtr latest_input_;
    jsk_pcl_ros::ClusterPointIndices::ConstPtr latest_input_indices_;
    jsk_pcl_ros::PolygonArray::ConstPtr latest_input_polygons_;
    jsk_pcl_ros::ModelCoefficientsArray::ConstPtr latest_input_coefficients_;

    sensor_msgs::PointCloud2::ConstPtr processing_input_;
    jsk_pcl_ros::ClusterPointIndices::ConstPtr processing_input_indices_;
    jsk_pcl_ros::PolygonArray::ConstPtr processing_input_polygons_;
    jsk_pcl_ros::ModelCoefficientsArray::ConstPtr processing_input_coefficients_;
    
    std::vector<pcl::KdTreeFLANN<PointT>::Ptr> kdtrees_;
    std::vector<pcl::PointCloud<PointT>::Ptr> separated_point_cloud_;
    uint32_t environment_id_;
    double distance_thr_;
    double sampling_d_;
    
  private:
  };
}

#endif
