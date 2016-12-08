// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, JSK Lab
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

#ifndef JSK_PCL_ROS_SNAPIT_H_
#define JSK_PCL_ROS_SNAPIT_H_

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include "jsk_recognition_msgs/CallSnapIt.h"
#include <tf/transform_listener.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_recognition_utils/geo_util.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <jsk_recognition_msgs/SnapFootstep.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
namespace jsk_pcl_ros
{
  class SnapIt: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::ModelCoefficientsArray> SyncPolygonPolicy;
    SnapIt(): DiagnosticNodelet("SnapIt") {}
    
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void polygonCallback(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg);
    virtual void polygonAlignCallback(
      const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    virtual void convexAlignCallback(
      const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    virtual void convexAlignPolygonCallback(
      const geometry_msgs::PolygonStamped::ConstPtr& poly_msg);
    virtual std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> createConvexes(
      const std::string& frame_id, const ros::Time& stamp,
      jsk_recognition_msgs::PolygonArray::ConstPtr polygons);
    virtual int findNearestConvex(
      const Eigen::Vector3f& pose_point, 
      const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes);
    virtual geometry_msgs::PoseStamped alignPose(
      Eigen::Affine3f& pose, jsk_recognition_utils::ConvexPolygon::Ptr convex);
    virtual bool footstepAlignServiceCallback(
      jsk_recognition_msgs::SnapFootstep::Request& req,
      jsk_recognition_msgs::SnapFootstep::Response& res);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    tf::TransformListener* tf_listener_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolygonPolicy> >sync_polygon_;
    ros::Publisher polygon_aligned_pub_;
    ros::Publisher convex_aligned_pub_;
    ros::Publisher convex_aligned_pose_array_pub_;
    ros::Publisher convex_aligned_pose_array_marker_pub_;
    ros::Subscriber polygon_align_sub_;
    ros::Subscriber convex_align_sub_;
    ros::Subscriber convex_align_polygon_sub_;
    bool use_service_;
    ros::ServiceServer align_footstep_srv_;
    jsk_recognition_msgs::PolygonArray::ConstPtr polygons_;
    boost::mutex mutex_;
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    
  };
}

#endif 
