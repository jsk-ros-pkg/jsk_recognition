// -*- mode: C++ -*-
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

#include "jsk_pcl_ros_utils/polygon_array_transformer.h"
#include <tf_conversions/tf_eigen.h>
#include "jsk_recognition_utils/geo_util.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros_utils
{

  void PolygonArrayTransformer::onInit()
  {
    ConnectionBasedNodelet::onInit();
    if (!pnh_->getParam("frame_id", frame_id_)) {
      NODELET_FATAL("~frame_id is not specified");
      return;
    }
    listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    polygons_pub_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output_polygons", 1);
    coefficients_pub_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output_coefficients", 1);
    onInitPostProcess();
  }

  void PolygonArrayTransformer::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sync_->connectInput(sub_polygons_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&PolygonArrayTransformer::transform, this, _1, _2));
  }

  void PolygonArrayTransformer::unsubscribe()
  {
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }
  
  void PolygonArrayTransformer::computeCoefficients(const geometry_msgs::PolygonStamped& polygon,
                                                    PCLModelCoefficientMsg& coefficient)
  {
    Eigen::Vector3d A, B, C;
    A[0] = polygon.polygon.points[0].x;
    A[1] = polygon.polygon.points[0].y;
    A[2] = polygon.polygon.points[0].z;
    B[0] = polygon.polygon.points[1].x;
    B[1] = polygon.polygon.points[1].y;
    B[2] = polygon.polygon.points[1].z;
    C[0] = polygon.polygon.points[2].x;
    C[1] = polygon.polygon.points[2].y;
    C[2] = polygon.polygon.points[2].z;
    Eigen::Vector3d n = (B - A).cross(C - A).normalized();
    double a = n[0];
    double b = n[1];
    double c = n[2];
    double d = -(a * A[0] + b * A[1] + c * A[2]);
    coefficient.header = polygon.header;
    coefficient.values.push_back(a);
    coefficient.values.push_back(b);
    coefficient.values.push_back(c);
    coefficient.values.push_back(d);
    
  }

  void PolygonArrayTransformer::transformModelCoefficient(const Eigen::Affine3d& transform,
                                                          const PCLModelCoefficientMsg& coefficient,
                                                          PCLModelCoefficientMsg& result)
  {
    jsk_recognition_utils::Plane plane(coefficient.values);
    jsk_recognition_utils::Plane transformed_plane = plane.transform(transform);
    result.header.stamp = coefficient.header.stamp;
    result.header.frame_id = frame_id_;
    transformed_plane.toCoefficients(result.values);
    NODELET_DEBUG("[%f, %f, %f, %f] => [%f, %f, %f, %f]",
                  coefficient.values[0], coefficient.values[1], coefficient.values[2], coefficient.values[3],
                  result.values[0], result.values[1], result.values[2], result.values[3]);
  }

  void PolygonArrayTransformer::transformPolygon(const Eigen::Affine3d& transform,
                                                 const geometry_msgs::PolygonStamped& polygon,
                                                 geometry_msgs::PolygonStamped& result)
  {
    result.header = polygon.header;
    result.header.frame_id = frame_id_;
    for (size_t i = 0; i < polygon.polygon.points.size(); i++) {
      Eigen::Vector4d point;
      point[0] = polygon.polygon.points[i].x;
      point[1] = polygon.polygon.points[i].y;
      point[2] = polygon.polygon.points[i].z;
      point[3] = 1;             // homogenious
      Eigen::Vector4d transformed_point_eigen = transform.inverse() * point;
      geometry_msgs::Point32 transformed_point;
      transformed_point.x = transformed_point_eigen[0];
      transformed_point.y = transformed_point_eigen[1];
      transformed_point.z = transformed_point_eigen[2];
      result.polygon.points.push_back(transformed_point);
    }
  }
  
  void PolygonArrayTransformer::transform(const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
                                          const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients)
  {
    if (polygons->polygons.size() != coefficients->coefficients.size()) {
      NODELET_ERROR("the size of polygons(%lu) does not match with the size of coefficients(%lu)",
                    polygons->polygons.size(),
                    coefficients->coefficients.size());
      return;
    }

    jsk_recognition_msgs::PolygonArray transformed_polygon_array;
    jsk_recognition_msgs::ModelCoefficientsArray transformed_model_coefficients_array;
    transformed_polygon_array.header = polygons->header;
    transformed_model_coefficients_array.header = coefficients->header;
    transformed_polygon_array.header.frame_id = frame_id_;
    transformed_model_coefficients_array.header.frame_id = frame_id_;
    for (size_t i = 0; i < polygons->polygons.size(); i++) {
      geometry_msgs::PolygonStamped polygon = polygons->polygons[i];
      PCLModelCoefficientMsg coefficient = coefficients->coefficients[i];

      if (polygon.header.frame_id != coefficient.header.frame_id) {
        NODELET_ERROR("frame_id of polygon[%lu] is %s and frame_id of coefficient[%lu] is %s, they does not point to the same frame_id",
                      i, polygon.header.frame_id.c_str(),
                      i, coefficient.header.frame_id.c_str());
        return;
      }
      listener_->waitForTransform(coefficient.header.frame_id,
                                  frame_id_,
                                  coefficient.header.stamp,
                                  ros::Duration(1.0));
      if (listener_->canTransform(coefficient.header.frame_id,
                                  frame_id_,
                                  coefficient.header.stamp)) {
        tf::StampedTransform transform; // header -> frame_id_
        listener_->lookupTransform(coefficient.header.frame_id, frame_id_,
                                   //ros::Time(0.0), transform);
                                   coefficient.header.stamp, transform);
        Eigen::Affine3d eigen_transform;
        tf::transformTFToEigen(transform, eigen_transform);
        PCLModelCoefficientMsg transformed_coefficient;
        //transformModelCoefficient(eigen_transform, coefficient, transformed_coefficient);
        
        geometry_msgs::PolygonStamped transformed_polygon;
        if (polygon.polygon.points.size() == 0) {
          transformed_polygon.header = polygon.header;
          transformed_polygon.header.frame_id = frame_id_;
          transformed_polygon_array.polygons.push_back(transformed_polygon);
          transformed_coefficient.values = coefficient.values;
          transformed_coefficient.header = polygon.header;
          transformed_coefficient.header.frame_id = frame_id_;
          transformed_model_coefficients_array.coefficients.push_back(transformed_coefficient);
        }
        else {
          transformPolygon(eigen_transform, polygon, transformed_polygon);
          //computeCoefficients(transformed_polygon, transformed_coefficient);
          transformModelCoefficient(eigen_transform, coefficient,
                                    transformed_coefficient);
          if (isnan(transformed_coefficient.values[0]) ||
              isnan(transformed_coefficient.values[1]) ||
              isnan(transformed_coefficient.values[2]) ||
              isnan(transformed_coefficient.values[3])) {
            continue;
          }
          transformed_polygon_array.polygons.push_back(transformed_polygon);
          transformed_model_coefficients_array.coefficients.push_back(transformed_coefficient);
        }
      }
      else {
        NODELET_ERROR("cannot lookup transform from %s to %s at %f",
                      frame_id_.c_str(), coefficient.header.frame_id.c_str(),
                      coefficient.header.stamp.toSec());
        return;
      }
    }
    polygons_pub_.publish(transformed_polygon_array);
    coefficients_pub_.publish(transformed_model_coefficients_array);
  }
  
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonArrayTransformer, nodelet::Nodelet);
