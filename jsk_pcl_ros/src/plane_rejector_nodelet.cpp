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

#include "jsk_pcl_ros/plane_rejector.h"
#include <pluginlib/class_list_macros.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <eigen_conversions/eigen_msg.h>

#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
typedef pcl_msgs::ModelCoefficients PCLModelCoefficientMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
typedef pcl::ModelCoefficients PCLModelCoefficientMsg;
#endif


namespace jsk_pcl_ros
{
  void PlaneRejector::onInit()
  {
    PCLNodelet::onInit();
    if (!pnh_->getParam("processing_frame_id", processing_frame_id_)) {
      NODELET_FATAL("You need to specify ~processing_frame_id");
      return;
    }
    if (!readVectorParam("reference_axis")) {
      return;
    }
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PlaneRejector::configCallback, this, _1, _2);
    srv_->setCallback (f);

    listener_.reset(new tf::TransformListener());
    polygons_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_polygons", 1);
    coefficients_pub_ = pnh_->advertise<jsk_pcl_ros::ModelCoefficientsArray>("output_coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_polygons_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&PlaneRejector::reject, this, _1, _2));
  }

  double PlaneRejector::getXMLDoubleValue(XmlRpc::XmlRpcValue val) {
    switch(val.getType()) {
    case XmlRpc::XmlRpcValue::TypeInt:
      return (double)((int)val);
    case XmlRpc::XmlRpcValue::TypeDouble:
      return (double)val;
    default:
      return 0;
    }
  }
  
  bool PlaneRejector::readVectorParam(const std::string& param_name)
  {
    if (pnh_->hasParam(param_name)) {
      XmlRpc::XmlRpcValue v;
      pnh_->param(param_name, v, v);
      if (v.getType() == XmlRpc::XmlRpcValue::TypeArray &&
          v.size() == 3) {
        reference_axis_[0] = getXMLDoubleValue(v[0]);
        reference_axis_[1] = getXMLDoubleValue(v[1]);
        reference_axis_[2] = getXMLDoubleValue(v[2]);
        reference_axis_.normalize();
        return true;
      }
      else {
        NODELET_FATAL("%s is not 3 dimensional vector",
                      param_name.c_str());
        return false;
      }
    }
    else {
      NODELET_FATAL("%s is not available",
                    param_name.c_str());
      return false;
    }
  }

  
  void PlaneRejector::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    angle_thr_ = config.angle_thr;
  }
  
  void PlaneRejector::reject(const jsk_pcl_ros::PolygonArray::ConstPtr& polygons,
                             const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients)
  {
    boost::mutex::scoped_lock lock(mutex_);
    jsk_pcl_ros::PolygonArray result_polygons;
    jsk_pcl_ros::ModelCoefficientsArray result_coefficients;
    result_polygons.header = polygons->header;
    result_coefficients.header = coefficients->header;
    for (size_t i = 0; i < polygons->polygons.size(); i++) {
      geometry_msgs::PolygonStamped polygon = polygons->polygons[i];
      PCLModelCoefficientMsg coefficient = coefficients->coefficients[i];
      // transform the coefficients to processing_frame_id_
      if (listener_->canTransform(coefficient.header.frame_id,
                                  processing_frame_id_,
                                  coefficient.header.stamp)) {
        geometry_msgs::Vector3Stamped plane_axis;
        plane_axis.header = coefficient.header;
        plane_axis.vector.x = coefficient.values[0];
        plane_axis.vector.y = coefficient.values[1];
        plane_axis.vector.z = coefficient.values[2];
        geometry_msgs::Vector3Stamped transformed_plane_axis;
        listener_->transformVector(processing_frame_id_,
                                   plane_axis,
                                   transformed_plane_axis);
        Eigen::Vector3d eigen_transformed_plane_axis;
        tf::vectorMsgToEigen(transformed_plane_axis.vector,
                             eigen_transformed_plane_axis);
        double ang = acos(eigen_transformed_plane_axis.normalized().dot(reference_axis_));
        if (ang < angle_thr_) {
          result_polygons.polygons.push_back(polygons->polygons[i]);
          result_coefficients.coefficients.push_back(coefficient);
        }
      }
      else {
        ROS_FATAL("failed to transform %s to %s",
                  coefficient.header.frame_id.c_str(), processing_frame_id_.c_str());
     }
    }
    polygons_pub_.publish(result_polygons);
    coefficients_pub_.publish(result_coefficients);
  }
  
}

typedef jsk_pcl_ros::PlaneRejector PlaneRejector;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, PlaneRejector, PlaneRejector, nodelet::Nodelet);
