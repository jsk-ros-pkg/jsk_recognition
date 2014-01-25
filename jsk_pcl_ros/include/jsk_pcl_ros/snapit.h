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

#ifndef JSK_PCL_ROS_SNAPIT_H_
#define JSK_PCL_ROS_SNAPIT_H_

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>

#include "jsk_pcl_ros/CallSnapIt.h"
#include <tf/transform_listener.h>

namespace jsk_pcl_ros
{
  class SnapIt: public pcl_ros::PCLNodelet
  {
  public:
    SnapIt();
    virtual ~SnapIt();
    virtual void onInit();
  protected:
    typedef std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > EigenVector3fVector;
    virtual void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    virtual bool snapitCallback(jsk_pcl_ros::CallSnapIt::Request& req,
                                jsk_pcl_ros::CallSnapIt::Response& res);
    virtual bool processModelPlane(jsk_pcl_ros::CallSnapIt::Request& req,
                                   jsk_pcl_ros::CallSnapIt::Response& res);
    virtual bool processModelCylinder(jsk_pcl_ros::CallSnapIt::Request& req,
                                      jsk_pcl_ros::CallSnapIt::Response& res);
    virtual bool extractPointsInsidePlanePole(geometry_msgs::PolygonStamped target_plane,
                                              pcl::PointIndices::Ptr inliers,
                                              EigenVector3fVector& points,
                                              Eigen::Vector3f &n,
                                              Eigen::Vector3f &p);
    virtual double distanceAlongWithLine(const Eigen::Vector4f& point, const Eigen::Vector4f& center, const Eigen::Vector4f direction);
    virtual void extractPlanePoints(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointIndices::Ptr out_inliers,
                                    pcl::ModelCoefficients::Ptr out_coefficients,
                                    Eigen::Vector3f normal,
                                    double eps_angle);
    virtual bool extractPointsInsideCylinder(const geometry_msgs::PointStamped& center,
                                             const geometry_msgs::Vector3Stamped direction,
                                             const double radius,
                                             const double height,
                                             pcl::PointIndices::Ptr inliers,
                                             Eigen::Vector3f &n,
                                             Eigen::Vector3f &C_orig,
                                             const double fat_factor);
    virtual bool checkPointInsidePlane(EigenVector3fVector &plane_points,
                                       Eigen::Vector3f normal,
                                       Eigen::Vector3f point);
    virtual void publishPointCloud(ros::Publisher pub, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    virtual void publishConvexHullMarker(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull);
    
    ros::Subscriber sub_input_;
    ros::Publisher debug_candidate_points_pub_;
    ros::Publisher debug_candidate_points_pub2_;
    ros::Publisher debug_candidate_points_pub3_;
    ros::Publisher debug_centroid_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher debug_centroid_after_trans_pub_;
    ros::ServiceServer call_snapit_srv_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
    boost::shared_ptr<tf::TransformListener> tf_listener_;
    std_msgs::Header input_header_;
    std::string input_frame_id_;
  };
}

#endif 
