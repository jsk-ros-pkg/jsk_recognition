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
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <visualization_msgs/MarkerArray.h>
#include <algorithm>

namespace jsk_pcl_ros
{
  void SnapIt::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_ = TfListenerSingleton::getInstance();
    pnh_->param("use_service", use_service_, false);
    polygon_aligned_pub_ = advertise<geometry_msgs::PoseStamped>(
      *pnh_, "output/plane_aligned", 1);
    convex_aligned_pub_ = advertise<geometry_msgs::PoseStamped>(
      *pnh_, "output/convex_aligned", 1);
    convex_aligned_pose_array_pub_ = advertise<geometry_msgs::PoseArray>(
      *pnh_, "output/convex_aligned_pose_array", 1);
    if (use_service_) {
      subscribe();
      align_footstep_srv_ = pnh_->advertiseService(
        "align_footstep", &SnapIt::footstepAlignServiceCallback, this);
    }
    onInitPostProcess();
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
    convex_align_polygon_sub_ = pnh_->subscribe(
      "input/convex_align_polygon", 1,
      &SnapIt::convexAlignPolygonCallback, this);
  }

  void SnapIt::unsubscribe()
  {
    if (!use_service_) {
      sub_polygons_.unsubscribe();
      sub_coefficients_.unsubscribe();
      polygon_align_sub_.shutdown();
      convex_align_sub_.shutdown();
    }
    polygons_.reset();
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
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes
      = createConvexes(pose_msg->header.frame_id,
                       pose_msg->header.stamp,
                       polygons_);
    Eigen::Affine3d pose_eigend;
    Eigen::Affine3f pose_eigen;
    tf::poseMsgToEigen(pose_msg->pose, pose_eigend);
    convertEigenAffine3(pose_eigend, pose_eigen);
    Eigen::Vector3f pose_point(pose_eigen.translation());
    double min_distance = DBL_MAX;
    jsk_recognition_utils::ConvexPolygon::Ptr min_convex;
    for (size_t i = 0; i < convexes.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr convex = convexes[i];
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

  bool SnapIt::footstepAlignServiceCallback(
    jsk_recognition_msgs::SnapFootstep::Request& req,
    jsk_recognition_msgs::SnapFootstep::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    jsk_footstep_msgs::FootstepArray input_footsteps = req.input;
    res.output.header = input_footsteps.header;
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes
      = createConvexes(input_footsteps.header.frame_id,
                       input_footsteps.header.stamp,
                       polygons_);
    for (size_t i = 0; i < input_footsteps.footsteps.size(); i++) {
      jsk_footstep_msgs::Footstep footstep = input_footsteps.footsteps[i];
      Eigen::Affine3d pose_eigend;
      Eigen::Affine3f pose_eigen;
      tf::poseMsgToEigen(footstep.pose, pose_eigen);
      Eigen::Vector3f pose_point(pose_eigen.translation());
      int min_index = findNearestConvex(pose_point, convexes);
      jsk_footstep_msgs::Footstep aligned_footstep;
      if (min_index != -1) {
        jsk_recognition_utils::ConvexPolygon::Ptr min_convex = convexes[min_index];
        geometry_msgs::PoseStamped aligned_pose = alignPose(pose_eigen, min_convex);
        aligned_footstep.pose = aligned_pose.pose;
      }
      else {
        aligned_footstep.pose = footstep.pose;
        //convex_aligned_pub_.publish(pose_msg); // shoud we publish this?
      }
      aligned_footstep.leg = footstep.leg;
      aligned_footstep.dimensions = footstep.dimensions;
      aligned_footstep.duration = footstep.duration;
      aligned_footstep.footstep_group = footstep.footstep_group;
      res.output.footsteps.push_back(aligned_footstep);
    }
    return true;
  }

  void SnapIt::convexAlignPolygonCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& poly_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::PoseArray pose_array;
    pose_array.header = poly_msg->header;
    if (!polygons_) {
      NODELET_ERROR("no polygon is ready");
      return;
    }
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes
      = createConvexes(poly_msg->header.frame_id,
                       poly_msg->header.stamp,
                       polygons_);
    for (size_t i = 0; i < poly_msg->polygon.points.size(); i++) {
      geometry_msgs::Point32 p = poly_msg->polygon.points[i];
      Eigen::Vector3f pose_point(p.x, p.y, p.z);
      int min_index = findNearestConvex(pose_point, convexes);
      if (min_index == -1) {
        NODELET_ERROR("cannot project onto convex");
        return;
      }
      else {
        jsk_recognition_utils::ConvexPolygon::Ptr min_convex = convexes[min_index];
        Eigen::Affine3f pose_eigen = Eigen::Affine3f::Identity();
        pose_eigen.translate(pose_point);
        geometry_msgs::PoseStamped aligned_pose = alignPose(pose_eigen, min_convex);
        aligned_pose.header = poly_msg->header;
        pose_array.poses.push_back(aligned_pose.pose);
      }
    }
    convex_aligned_pose_array_pub_.publish(pose_array);
  }
  
  int SnapIt::findNearestConvex(
    const Eigen::Vector3f& pose_point, 
    const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
  {
    int min_index = -1;
    double min_distance = DBL_MAX;
    jsk_recognition_utils::ConvexPolygon::Ptr min_convex;
    for (size_t i = 0; i < convexes.size(); i++) {
      jsk_recognition_utils::ConvexPolygon::Ptr convex = convexes[i];
      if (convex->isProjectableInside(pose_point)) {
        double d = convex->distanceToPoint(pose_point);
        if (d < min_distance) {
          min_distance = d;
          min_convex = convex;
          min_index = i;
        }
      }
    }
    return min_index;
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
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes
      = createConvexes(pose_msg->header.frame_id,
                       pose_msg->header.stamp,
                       polygons_);
    Eigen::Affine3d pose_eigend;
    Eigen::Affine3f pose_eigen;
    tf::poseMsgToEigen(pose_msg->pose, pose_eigend);
    convertEigenAffine3(pose_eigend, pose_eigen);
    Eigen::Vector3f pose_point(pose_eigen.translation());
    int min_index = findNearestConvex(pose_point, convexes);
    if (min_index != -1) {
      jsk_recognition_utils::ConvexPolygon::Ptr min_convex = convexes[min_index];
      geometry_msgs::PoseStamped aligned_pose = alignPose(pose_eigen, min_convex);
      aligned_pose.header = pose_msg->header;
      convex_aligned_pub_.publish(aligned_pose);
    }
    else {
      convex_aligned_pub_.publish(pose_msg); // shoud we publish this?
    }
  }

  geometry_msgs::PoseStamped SnapIt::alignPose(
    Eigen::Affine3f& pose, jsk_recognition_utils::ConvexPolygon::Ptr convex)
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
    rot.setFromTwoVectors(old_normal, normal);
    aligned_pose = aligned_pose * rot;
    aligned_pose.translation() = projected_point;
    Eigen::Affine3d aligned_posed;
    convertEigenAffine3(aligned_pose, aligned_posed);
    geometry_msgs::PoseStamped ret;
    tf::poseEigenToMsg(aligned_posed, ret.pose);
    return ret;
  }
  
  std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> SnapIt::createConvexes(
    const std::string& frame_id, const ros::Time& stamp,
    jsk_recognition_msgs::PolygonArray::ConstPtr polygons)
  {
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> result;
    try
    {
      for (size_t i = 0; i < polygons->polygons.size(); i++) {
        geometry_msgs::PolygonStamped polygon = polygons->polygons[i];
        jsk_recognition_utils::Vertices vertices;
        
        tf::StampedTransform transform = lookupTransformWithDuration(
          tf_listener_,
          polygon.header.frame_id, frame_id, stamp, ros::Duration(5.0));
        for (size_t j = 0; j < polygon.polygon.points.size(); j++) {
          Eigen::Vector4d p;
          p[0] = polygon.polygon.points[j].x;
          p[1] = polygon.polygon.points[j].y;
          p[2] = polygon.polygon.points[j].z;
          p[3] = 1;
          Eigen::Affine3d eigen_transform;
          tf::transformTFToEigen(transform, eigen_transform);
          Eigen::Vector4d transformed_pointd = eigen_transform.inverse() * p;
          Eigen::Vector3f transformed_point;
          transformed_point[0] = transformed_pointd[0];
          transformed_point[1] = transformed_pointd[1];
          transformed_point[2] = transformed_pointd[2];
          vertices.push_back(transformed_point);
        }
        std::reverse(vertices.begin(), vertices.end());
        jsk_recognition_utils::ConvexPolygon::Ptr convex(new jsk_recognition_utils::ConvexPolygon(vertices));
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
