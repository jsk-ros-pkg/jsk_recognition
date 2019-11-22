// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros_utils/polygon_flipper.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <algorithm>
#include <iterator>

namespace jsk_pcl_ros_utils
{
  void PolygonFlipper::onInit()
  {
    DiagnosticNodelet::onInit();
    if (!pnh_->getParam("sensor_frame", sensor_frame_)) {
      NODELET_FATAL("no ~sensor_frame is specified");
      return;
    }

    pnh_->param<int>("queue_size", queue_size_, 100);
    pnh_->param<bool>("use_indices", use_indices_, true);

    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pub_polygons_ = advertise<jsk_recognition_msgs::PolygonArray>(
      *pnh_, "output/polygons", 1);
    pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output/coefficients", 1);
    if (use_indices_)
      pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
        *pnh_, "output/indices", 1);

    onInitPostProcess();
  }

  void PolygonFlipper::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
    sub_polygons_.subscribe(*pnh_, "input/polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 1);
    if (use_indices_) {
      sub_indices_.subscribe(*pnh_, "input/indices", 1);
      sync_->connectInput(sub_polygons_, sub_indices_, sub_coefficients_);
    } else {
      sub_polygons_.registerCallback(boost::bind(&PolygonFlipper::fillEmptyIndices, this, _1));
      sync_->connectInput(sub_polygons_, sub_indices_null_, sub_coefficients_);
    }
    sync_->registerCallback(boost::bind(&PolygonFlipper::flip, this, _1, _2, _3));
  }

  void PolygonFlipper::unsubscribe()
  {
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
    if (use_indices_)
      sub_indices_.unsubscribe();
  }

  void PolygonFlipper::fillEmptyIndices(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons_msg)
  {
    jsk_recognition_msgs::ClusterPointIndices indices;
    indices.header.stamp = polygons_msg->header.stamp;
    indices.cluster_indices.resize(polygons_msg->polygons.size());
    sub_indices_null_.add(
      boost::make_shared<jsk_recognition_msgs::ClusterPointIndices>(indices));
  }

  void PolygonFlipper::flip(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    vital_checker_->poke();
    if (polygons_msg->polygons.size() != coefficients_msg->coefficients.size()) {
      NODELET_ERROR("The size of polygons and coefficients are not same");
      return;
    }
    jsk_recognition_msgs::PolygonArray flipped_polygons = *polygons_msg;
    jsk_recognition_msgs::ModelCoefficientsArray flipped_coefficients;
    jsk_recognition_msgs::ClusterPointIndices flipped_indices;
    flipped_polygons.polygons.clear();
    flipped_coefficients.header = coefficients_msg->header;
    flipped_indices.header = indices_msg->header;
    try {
      for (size_t i = 0; i < polygons_msg->polygons.size(); i++) {
        geometry_msgs::PolygonStamped target_polygon = polygons_msg->polygons[i];
        PCLModelCoefficientMsg target_coefficients = coefficients_msg->coefficients[i];
        PCLIndicesMsg target_indices = indices_msg->cluster_indices[i];
        tf::StampedTransform tf_transform
          = jsk_recognition_utils::lookupTransformWithDuration(
            tf_listener_, target_coefficients.header.frame_id,
            sensor_frame_, target_coefficients.header.stamp,
            ros::Duration(1.0));
        Eigen::Affine3f sensor_transform;
        tf::transformTFToEigen(tf_transform, sensor_transform);
        {
          // poygons
          jsk_recognition_utils::ConvexPolygon convex = jsk_recognition_utils::ConvexPolygon::fromROSMsg(target_polygon.polygon);
          Eigen::Vector3f polygon_normal = convex.getNormal();
          if (polygon_normal.dot(Eigen::Vector3f(sensor_transform.translation()))
              < 0) {
            geometry_msgs::PolygonStamped flipped_polygon;
            std::reverse_copy(
              target_polygon.polygon.points.begin(),
              target_polygon.polygon.points.end(),
              std::back_inserter(flipped_polygon.polygon.points));
            flipped_polygon.header = target_polygon.header;
            flipped_polygons.polygons.push_back(flipped_polygon);
          }
          else {
            flipped_polygons.polygons.push_back(target_polygon);
          }
        }
        
        {
          // coefficients
          Eigen::Vector3f local_normal(target_coefficients.values[0],
                                       target_coefficients.values[1],
                                       target_coefficients.values[2]);
          if (local_normal.dot(Eigen::Vector3f(sensor_transform.translation()))
              < 0) {
            PCLModelCoefficientMsg the_flipped_coefficients;
            PCLIndicesMsg the_flipped_indices;
            for (size_t j = 0; j < target_coefficients.values.size(); j++) {
              the_flipped_coefficients.values.push_back(
                - target_coefficients.values[j]);
            }
            std::reverse_copy(target_indices.indices.begin(), target_indices.indices.end(),
                              std::back_inserter(the_flipped_indices.indices));
            the_flipped_coefficients.header = target_coefficients.header;
            the_flipped_indices.header = target_indices.header;
            flipped_coefficients.coefficients.push_back(the_flipped_coefficients);
            //flipped_coefficients.coefficients.push_back(target_coefficients);
            flipped_indices.cluster_indices.push_back(the_flipped_indices);
            //flipped_indices.cluster_indices.push_back(target_indices);
          }
          else {
            // no need to flip
            flipped_coefficients.coefficients.push_back(target_coefficients);
            flipped_indices.cluster_indices.push_back(target_indices);
          }
        }
      }
      pub_polygons_.publish(flipped_polygons);
      pub_coefficients_.publish(flipped_coefficients);
      if (use_indices_)
        pub_indices_.publish(flipped_indices);
    }
    catch (tf2::TransformException& e) {
      NODELET_ERROR("Failed to lookup transformation: %s", e.what());
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonFlipper, nodelet::Nodelet);
