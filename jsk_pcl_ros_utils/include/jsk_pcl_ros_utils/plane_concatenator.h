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


#ifndef JSK_PCL_ROS_UTILS_PLANE_CONCATENATOR_H_
#define JSK_PCL_ROS_UTILS_PLANE_CONCATENATOR_H_

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>

#include <jsk_pcl_ros_utils/PlaneConcatenatorConfig.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_recognition_utils/geo_util.h"

namespace jsk_pcl_ros_utils
{
  class PlaneConcatenator: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<PlaneConcatenator> Ptr;
    typedef PlaneConcatenatorConfig Config;
    typedef pcl::PointXYZRGB PointT;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::ModelCoefficientsArray
      > SyncPolicy;
    PlaneConcatenator(): DiagnosticNodelet("PlaneConcatenator") {}
    
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void concatenate(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_array_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_array_msg);
    virtual void configCallback(Config &config, uint32_t level);
    virtual bool isNearPointCloud(
      pcl::KdTreeFLANN<PointT>& kdtree,
      pcl::PointCloud<PointT>::Ptr cloud,
      jsk_recognition_utils::Plane::Ptr target_plane);
    virtual pcl::ModelCoefficients::Ptr refinement(
      pcl::PointCloud<PointT>::Ptr cloud,
      pcl::PointIndices::Ptr indices,
      pcl::ModelCoefficients::Ptr original_coefficients);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::mutex mutex_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_indices_;
    ros::Publisher pub_polygon_;
    ros::Publisher pub_coefficients_;
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    double connect_angular_threshold_;
    double connect_distance_threshold_;
    double connect_perpendicular_distance_threshold_;
    int ransac_refinement_max_iteration_;
    double ransac_refinement_outlier_threshold_;
    double ransac_refinement_eps_distance_;
    double ransac_refinement_eps_angle_;
    int min_size_;
    double min_area_;
    double max_area_;
  private:
    
  };
}
#endif
