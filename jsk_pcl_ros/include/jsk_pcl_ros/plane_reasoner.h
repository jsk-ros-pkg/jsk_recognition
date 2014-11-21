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


#ifndef JSK_PCL_ROS_PLANE_REASONER_H_
#define JSK_PCL_ROS_PLANE_REASONER_H_

#include "jsk_pcl_ros/diagnostic_nodelet.h"
#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "jsk_pcl_ros/PolygonArray.h"
#include "jsk_pcl_ros/ModelCoefficientsArray.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/PlaneReasonerConfig.h"

#include "jsk_pcl_ros/geo_util.h"

namespace jsk_pcl_ros
{
  typedef boost::tuple<pcl::PointIndices::Ptr,
                       pcl::ModelCoefficients::Ptr,
                       Plane::Ptr,
                       geometry_msgs::PolygonStamped>
  PlaneInfoContainer;
  
  class PlaneReasoner: public DiagnosticNodelet
  {
  public:
    ////////////////////////////////////////////////////////
    // typedefs
    ////////////////////////////////////////////////////////
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    ClusterPointIndices,
    ModelCoefficientsArray,
    PolygonArray> SyncPolicy;
    typedef jsk_pcl_ros::PlaneReasonerConfig Config;
    typedef pcl::PointXYZRGB PointT;
    
    
    PlaneReasoner(): DiagnosticNodelet("PlaneReasoner") { }

  protected:
    ////////////////////////////////////////////////////////
    // Methods
    ////////////////////////////////////////////////////////
    virtual void onInit();

    virtual void subscribe();

    virtual void unsubscribe();
    
    virtual void reason(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const ClusterPointIndices::ConstPtr& inliers_msg,
      const ModelCoefficientsArray::ConstPtr& coefficients_msg,
      const PolygonArray::ConstPtr& polygons_msg);

    virtual void configCallback (Config &config, uint32_t level);

    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    
    virtual std::vector<PlaneInfoContainer>
    packInfo(std::vector<pcl::PointIndices::Ptr>& inliers,
             std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
             std::vector<Plane::Ptr>& planes,
             std::vector<geometry_msgs::PolygonStamped>& polygons);

    virtual std::vector<PlaneInfoContainer>
    filterHorizontalPlanes(
      std::vector<PlaneInfoContainer>& infos);
    
    virtual std::vector<PlaneInfoContainer>
    filterVerticalPlanes(
      std::vector<PlaneInfoContainer>& infos);

    virtual std::vector<PlaneInfoContainer>
    filterPlanesAroundAngle(
      double reference_angle,
      double thrshold,
      std::vector<PlaneInfoContainer>& infos);

    virtual void publishPlaneInfo(
      std::vector<PlaneInfoContainer>& containers,
      const std_msgs::Header& header,
      pcl::PointCloud<PointT>::Ptr cloud,
      ros::Publisher& pub_inlier,
      ros::Publisher& pub_coefficients,
      ros::Publisher& pub_polygons);
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<ClusterPointIndices> sub_inliers_;
    message_filters::Subscriber<ModelCoefficientsArray> sub_coefficients_;
    message_filters::Subscriber<PolygonArray> sub_polygons_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    ros::Publisher pub_vertical_inliers_;
    ros::Publisher pub_vertical_coefficients_;
    ros::Publisher pub_vertical_polygons_;
    ros::Publisher pub_horizontal_inliers_;
    ros::Publisher pub_horizontal_coefficients_;
    ros::Publisher pub_horizontal_polygons_;
    
    boost::mutex mutex_;
    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    std::string global_frame_id_;
    double horizontal_angular_threshold_;
    double vertical_angular_threshold_;
  private:
    
  };
}

#endif
