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

#include "jsk_pcl_ros/occluded_plane_estimator.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

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
  void OccludedPlaneEstimator::onInit()
  {
    PCLNodelet::onInit();
    polygon_pub_ = pnh_->advertise<jsk_pcl_ros::PolygonArray>("output_polygons", 1);
    coefficient_pub_ = pnh_->advertise<jsk_pcl_ros::ModelCoefficientsArray>("output_coefficients", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OccludedPlaneEstimator::configCallback, this, _1, _2);
    srv_->setCallback (f);
    
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_polygons_.subscribe(*pnh_, "input_polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input_coefficients", 1);
    sub_static_polygons_.subscribe(*pnh_, "input_static_polygons", 1);
    sub_static_coefficients_.subscribe(*pnh_, "input_static_coefficients", 1);
    
    sync_->connectInput(sub_polygons_, sub_coefficients_, sub_static_polygons_, sub_static_coefficients_);
    sync_->registerCallback(boost::bind(&OccludedPlaneEstimator::estimate,
                                        this, _1, _2, _3, _4));
  }

  void OccludedPlaneEstimator::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    plane_distance_threshold_ = config.plane_distance_threshold;
    plane_angle_threshold_ = config.plane_angle_threshold;
  }

  void OccludedPlaneEstimator::estimate(const jsk_pcl_ros::PolygonArray::ConstPtr& polygons,
                                       const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& coefficients,
                                       const jsk_pcl_ros::PolygonArray::ConstPtr& static_polygons,
                                       const jsk_pcl_ros::ModelCoefficientsArray::ConstPtr& static_coefficients)
  {
    boost::mutex::scoped_lock(mutex_);
    // error check
    if (polygons->polygons.size() != coefficients->coefficients.size()) {
      NODELET_ERROR("the size of the input polygon array and model coefficients array is not same");
      return;
    }
    if (static_polygons->polygons.size() != static_coefficients->coefficients.size()) {
      NODELET_ERROR("the size of the input static polygon array and static model coefficients array is not same");
      return;
    }
    jsk_pcl_ros::PolygonArray result_polygons;
    jsk_pcl_ros::ModelCoefficientsArray result_coefficients;
    result_polygons = *polygons;
    result_coefficients = *coefficients;
    for (size_t i = 0; i < static_polygons->polygons.size(); i++) {
      // looking for the nearest polygon from the static polygon
      geometry_msgs::PolygonStamped static_polygon = static_polygons->polygons[i];
      PCLModelCoefficientMsg static_coefficient = static_coefficients->coefficients[i];

      // linear search, it may not fast enough if we need to tuckle against
      // larger scene
      int nearest_index = -1;
      double min_angle_distance = DBL_MAX;
      for (size_t j = 0; j < polygons->polygons.size(); j++) {
        geometry_msgs::PolygonStamped candidate_polygon = polygons->polygons[j];
        PCLModelCoefficientMsg candidate_coefficient = coefficients->coefficients[j];
        if (candidate_polygon.header.frame_id != static_polygon.header.frame_id) {
          NODELET_ERROR("frame_id of static polygon and candidate polygon are not the same one: %s and %s",
                        static_polygon.header.frame_id.c_str(),
                        candidate_polygon.header.frame_id.c_str());
          continue;
        }
        // first, compute the angle distance
        Eigen::Vector3f a_normal(candidate_coefficient.values[0], candidate_coefficient.values[1], candidate_coefficient.values[2]);
        Eigen::Vector3f b_normal(static_coefficient.values[0], static_coefficient.values[1], static_coefficient.values[2]);
        double a_distance = candidate_coefficient.values[3];
        double b_distance = static_coefficient.values[3];
        if (a_normal.norm() != 1.0) {
          a_distance = a_distance / a_normal.norm();
          a_normal = a_normal / a_normal.norm();
        }
        if (b_normal.norm() != 1.0) {
          b_distance = b_distance / b_normal.norm();
          b_normal = b_normal / b_normal.norm();
        }
        if (a_normal.dot(b_normal) < 0) {
          b_distance = - b_distance;
          b_normal = - b_normal;
        }
        NODELET_INFO("[%f, %f, %f] - %f  --- [%f, %f, %f] - %f",
                     a_normal[0], a_normal[1], a_normal[2], a_distance,
                     b_normal[0], b_normal[1], b_normal[2], b_distance);
        NODELET_INFO("%lu - %lu distance: %f", i, j, fabs(fabs(a_distance) - fabs(b_distance)));
        if (fabs(fabs(a_distance) - fabs(b_distance)) > plane_distance_threshold_) {
          continue;
        }
        double theta = fabs(acos(a_normal.dot(b_normal)));
        NODELET_INFO("%lu - %lu angle: %f", i, j, theta);
        if (theta > M_PI / 2.0) {
          theta = M_PI  - theta;
        }
        if (theta > plane_angle_threshold_) {
          continue;
        }
        if (min_angle_distance > theta) {
          min_angle_distance = theta;
          nearest_index = j;
        }
      }

      if (nearest_index != -1) {
        // merged into -1
        NODELET_INFO("merging %lu into %d", i, nearest_index);
        // project the points to the plane before run qhull
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType (pcl::SACMODEL_PLANE);
        pcl::ModelCoefficients plane_coefficients;
        plane_coefficients.values = coefficients->coefficients[nearest_index].values;
        proj.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients>(plane_coefficients));
        pcl::PointCloud<pcl::PointXYZ> cloud;
        geometry_msgs::PolygonStamped nearest_polygon = result_polygons.polygons[nearest_index];
        
        for (size_t j = 0; j < static_polygon.polygon.points.size(); j++) {
          pcl::PointXYZ p;
          p.x = static_polygon.polygon.points[j].x;
          p.y = static_polygon.polygon.points[j].y;
          p.z = static_polygon.polygon.points[j].z;
          cloud.points.push_back(p);
        }
        for (size_t j = 0; j < nearest_polygon.polygon.points.size(); j++) {
          pcl::PointXYZ p;
          p.x = nearest_polygon.polygon.points[j].x;
          p.y = nearest_polygon.polygon.points[j].y;
          p.z = nearest_polygon.polygon.points[j].z;
          cloud.points.push_back(p);
        }
        pcl::PointCloud<pcl::PointXYZ> projected_cloud;
        proj.setInputCloud(cloud.makeShared());
        proj.filter(projected_cloud);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(projected_cloud.makeShared());
        chull.setDimension(2);
        pcl::PointCloud<pcl::PointXYZ> chull_output;
        chull.reconstruct(chull_output);
        // rewriting the points...
        nearest_polygon.polygon.points.clear();
        for (size_t j = 0; j < chull_output.points.size(); j++) {
          geometry_msgs::Point32 p;
          p.x = chull_output.points[j].x;
          p.y = chull_output.points[j].y;
          p.z = chull_output.points[j].z;
          nearest_polygon.polygon.points.push_back(p);
        }
        result_polygons.polygons[nearest_index] = nearest_polygon;
      }
    }
    polygon_pub_.publish(result_polygons);
    coefficient_pub_.publish(result_coefficients);
  }
  
}

typedef jsk_pcl_ros::OccludedPlaneEstimator OccludedPlaneEstimator;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, OccludedPlaneEstimator, OccludedPlaneEstimator, nodelet::Nodelet);
