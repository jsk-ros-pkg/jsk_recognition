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

#include "jsk_pcl_ros/snapit.h"
#include <pluginlib/class_list_macros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <Eigen/StdVector>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/distances.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/common/centroid.h>
#include <visualization_msgs/Marker.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <algorithm>  

namespace jsk_pcl_ros
{
  void SnapIt::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_.reset(new tf::TransformListener());
    polygon_aligned_pub_ = advertise<geometry_msgs::PoseStamped>(
      *pnh_, "output/plane_aligned", 1);
    convex_aligned_pub_ = advertise<geometry_msgs::PoseStamped>(
      *pnh_, "output/convex_aligned", 1);
  }

  void SnapIt::subscribe()
  {
    sub_polygons_.subscribe(*pnh_, "input/polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input/polygon_coefficients", 1);
    sync_polygon_
      = boost::make_shared<message_filters::Synchronizer<SyncPolygonPolicy> >(100);
    sync_polygon_->connectInput(sub_polygons_, sub_coefficients_);
    sync_polygon_->registerCallback(
      boost::bind(&SnapIt::polygonCallback, this, _1, _2));
    polygon_align_sub_ = pnh_->subscribe("input/plane_align", 1,
                                         &SnapIt::polygonAlignCallback, this);
    convex_align_sub_ = pnh_->subscribe("input/convex_align", 1,
                                        &SnapIt::convexAlignCallback, this);
  }

  void SnapIt::unsubscribe()
  {
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
    polygon_align_sub_.shutdown();
    convex_align_sub_.shutdown();
  }

  void SnapIt::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    
  }
  
  void SnapIt::polygonCallback(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    polygons_ = polygon_msg;
  }

  void SnapIt::polygonAlignCallback(
    const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!polygons_) {
      NODELET_ERROR("no polygon is ready");
      polygon_aligned_pub_.publish(pose_msg);
      return;
    }
    std::vector<ConvexPolygon::Ptr> convexes
      = createConvexes(pose_msg->header.frame_id,
                       pose_msg->header.stamp,
                       polygons_);
    Eigen::Affine3d pose_eigend;
    Eigen::Affine3f pose_eigen;
    tf::poseMsgToEigen(pose_msg->pose, pose_eigend);
    convertEigenAffine3(pose_eigend, pose_eigen);
    Eigen::Vector3f pose_point(pose_eigen.translation());
    double min_distance = DBL_MAX;
    ConvexPolygon::Ptr min_convex;
    for (size_t i = 0; i < convexes.size(); i++) {
      ConvexPolygon::Ptr convex = convexes[i];
      double d = convex->distanceToPoint(pose_point);
      if (d < min_distance) {
        min_convex = convex;
        min_distance = d;
      }
    }
    if (min_convex) {
      geometry_msgs::PoseStamped aligned_pose = alignPose(pose_eigen, min_convex);
      aligned_pose.header = pose_msg->header;
      polygon_aligned_pub_.publish(aligned_pose);
    }
    else {
      polygon_aligned_pub_.publish(pose_msg);
    }
  }
  
  void SnapIt::convexAlignCallback(
      const geometry_msgs::PoseStamped::ConstPtr& pose_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!polygons_) {
      NODELET_ERROR("no polygon is ready");
      convex_aligned_pub_.publish(pose_msg);
      return;
    }
    std::vector<ConvexPolygon::Ptr> convexes
      = createConvexes(pose_msg->header.frame_id,
                       pose_msg->header.stamp,
                       polygons_);
    Eigen::Affine3d pose_eigend;
    Eigen::Affine3f pose_eigen;
    tf::poseMsgToEigen(pose_msg->pose, pose_eigend);
    convertEigenAffine3(pose_eigend, pose_eigen);
    Eigen::Vector3f pose_point(pose_eigen.translation());
    double min_distance = DBL_MAX;
    ConvexPolygon::Ptr min_convex;
    for (size_t i = 0; i < convexes.size(); i++) {
      ConvexPolygon::Ptr convex = convexes[i];
      if (convex->isProjectableInside(pose_point)) {
        double d = convex->distanceToPoint(pose_point);
        if (d < min_distance) {
          min_distance = d;
          min_convex = convex;
        }
      }
    }
    if (min_convex) {
      geometry_msgs::PoseStamped aligned_pose = alignPose(pose_eigen, min_convex);
      aligned_pose.header = pose_msg->header;
      convex_aligned_pub_.publish(aligned_pose);
    }
    else {
      convex_aligned_pub_.publish(pose_msg);
    }
  }

  geometry_msgs::PoseStamped SnapIt::alignPose(
    Eigen::Affine3f& pose, ConvexPolygon::Ptr convex)
  {
    Eigen::Affine3f aligned_pose(pose);
    Eigen::Vector3f original_point(pose.translation());
    Eigen::Vector3f projected_point;
    convex->project(original_point, projected_point);
    
    Eigen::Vector3f normal = convex->getNormal();
    Eigen::Vector3f old_normal;
    old_normal[0] = pose(0, 2);
    old_normal[1] = pose(1, 2);
    old_normal[2] = pose(2, 2);
    Eigen::Quaternionf rot;
    if (normal.dot(old_normal) < 0) {
      normal = - normal;
    }
    // Vertices vs = convex->getVertices();
    // for (size_t i = 0; i < vs.size(); i++) {
    //   NODELET_INFO("aligned vs: [%f, %f, %f]", vs[i][0], vs[i][1], vs[i][2]);
    // }
    // std::vector<float> coefficients;
    // convex->toCoefficients(coefficients);
    // NODELET_INFO("aligned c: [%f, %f, %f, %f]", coefficients[0], coefficients[1], coefficients[2], coefficients[3]);
    // NODELET_INFO("on: [%f, %f, %f]", old_normal[0], old_normal[1], old_normal[2]);
    // NODELET_INFO("n: [%f, %f, %f]", normal[0], normal[1], normal[2]);
    rot.setFromTwoVectors(old_normal, normal);
    
    //aligned_pose.rotate(rot);
    //aligned_pose.rotate(rot * aligned_pose.rotation());
    //aligned_pose.translation() = Eigen::Vector3f(0, 0, 0);
    
    aligned_pose = aligned_pose * rot;
    aligned_pose.translation() = projected_point;
    //NODELET_INFO("projected_point: [%f, %f, %f]", projected_point[0], projected_point[1], projected_point[2]);
    //aligned_pose.translation() = projected_point;
    Eigen::Affine3d aligned_posed;
    convertEigenAffine3(aligned_pose, aligned_posed);
    geometry_msgs::PoseStamped ret;
    tf::poseEigenToMsg(aligned_posed, ret.pose);
    return ret;
  }
  
  std::vector<ConvexPolygon::Ptr> SnapIt::createConvexes(
    const std::string& frame_id, const ros::Time& stamp,
    jsk_recognition_msgs::PolygonArray::ConstPtr polygons)
  {
    std::vector<ConvexPolygon::Ptr> result;
    try
    {
      for (size_t i = 0; i < polygons->polygons.size(); i++) {
        geometry_msgs::PolygonStamped polygon = polygons->polygons[i];
        Vertices vertices;
        for (size_t j = 0; j < polygon.polygon.points.size(); j++) {
          // geometry_msgs::PointStamped in_point, out_point;
          // in_point.header.frame_id = polygon.header.frame_id;
          // in_point.header.stamp = stamp;
          Eigen::Vector4d p;
          p[0] = polygon.polygon.points[j].x;
          p[1] = polygon.polygon.points[j].y;
          p[2] = polygon.polygon.points[j].z;
          p[3] = 1;
          // pointFromXYZToXYZ<geometry_msgs::Point32, geometry_msgs::Point>(
          //   polygon.polygon.points[j], in_point.point);
          //NODELET_INFO("%s -> %s", polygon.header.frame_id.c_str(), frame_id.c_str());
          // tf_listener_->transformPoint(frame_id, in_point, out_point);
          tf::StampedTransform transform;
          tf_listener_->lookupTransform(polygon.header.frame_id, frame_id, stamp, transform);
          Eigen::Affine3d eigen_transform;
          tf::transformTFToEigen(transform, eigen_transform);
          Eigen::Vector4d transformed_pointd = eigen_transform.inverse() * p;
          //tf::transformTFToEigen(transform, eigen_transform);
          
          Eigen::Vector3f transformed_point;
          // pointFromXYZToVector<geometry_msgs::Point, Eigen::Vector3f>(
          //   out_point.point, transformed_point);
          transformed_point[0] = transformed_pointd[0];
          transformed_point[1] = transformed_pointd[1];
          transformed_point[2] = transformed_pointd[2];
          //NODELET_INFO("v: [%f, %f, %f]", transformed_point[0], transformed_point[1], transformed_point[2]);
          vertices.push_back(transformed_point);
        }
        std::reverse(vertices.begin(), vertices.end());
        ConvexPolygon::Ptr convex(new ConvexPolygon(vertices));
        result.push_back(convex);
      }
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    catch (tf2::ExtrapolationException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
    return result;
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::SnapIt, nodelet::Nodelet);
