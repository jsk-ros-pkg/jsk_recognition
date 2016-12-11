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
#include "jsk_pcl_ros/hinted_stick_finder.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d_omp.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseStamped.h>

namespace jsk_pcl_ros
{
  void HintedStickFinder::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&HintedStickFinder::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("use_normal", use_normal_, false);
    pnh_->param("not_synchronize", not_synchronize_, false);
    
    pub_line_filtered_indices_ = advertise<PCLIndicesMsg>(
      *pnh_, "debug/line_filtered_indices", 1);
    pub_line_filtered_normal_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "debug/line_filtered_normal", 1);
    pub_cylinder_marker_ = advertise<visualization_msgs::Marker>(
      *pnh_, "debug/cylinder_marker", 1);
    pub_cylinder_pose_ = advertise<geometry_msgs::PoseStamped>(
      *pnh_, "output/cylinder_pose", 1);
    pub_inliers_ = advertise<PCLIndicesMsg>(
      *pnh_, "output/inliers", 1);
    pub_coefficients_ = advertise<PCLModelCoefficientMsg>(
      *pnh_, "output/coefficients", 1);
    onInitPostProcess();
  }

  void HintedStickFinder::subscribe()
  {
    if (!not_synchronize_) {
      sub_polygon_.subscribe(*pnh_, "input/hint/line", 1);
      sub_info_.subscribe(*pnh_, "input/camera_info", 1);
      sub_cloud_.subscribe(*pnh_, "input", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<ASyncPolicy> >(100);
      sync_->connectInput(sub_polygon_, sub_info_, sub_cloud_);
      sync_->registerCallback(boost::bind(&HintedStickFinder::detect, this,
                                          _1, _2, _3));
    }
    else {
      sub_no_sync_cloud_ = pnh_->subscribe(
        "input", 1, &HintedStickFinder::cloudCallback, this);
      sub_no_sync_camera_info_ = pnh_->subscribe(
        "input/camera_info", 1, &HintedStickFinder::infoCallback, this);
      sub_no_sync_polygon_ = pnh_->subscribe(
        "input/hint/line", 1, &HintedStickFinder::hintCallback, this);
    }
  }


  void HintedStickFinder::cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!latest_hint_ || !latest_camera_info_) {
        // not yet ready
        NODELET_WARN_THROTTLE(1, "~input/hint/lline or ~input/camera_info is not ready");
        return;
      }
    }
    detect(latest_hint_, latest_camera_info_, cloud_msg);
  }


  void HintedStickFinder::hintCallback(
    const geometry_msgs::PolygonStamped::ConstPtr& hint_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_hint_ = hint_msg;
  }

  void HintedStickFinder::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_camera_info_ = info_msg;
  }
  
  
  void HintedStickFinder::unsubscribe()
  {
    if (!not_synchronize_) {
      sub_polygon_.unsubscribe();
      sub_info_.unsubscribe();
      sub_cloud_.unsubscribe();
    }
    else {
      sub_no_sync_cloud_.shutdown();
      sub_no_sync_camera_info_.shutdown();
      sub_no_sync_polygon_.shutdown();
    }
  }

  void HintedStickFinder::detect(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg,
    const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_WARN("starting detection");
    ros::Time start_time = ros::Time::now();
    image_geometry::PinholeCameraModel model;
    model.fromCameraInfo(camera_info_msg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // convert 2-D point into 3-D ray
    Eigen::Vector3f a, b;
    jsk_recognition_utils::ConvexPolygon::Ptr polygon = polygonFromLine(polygon_msg, model, a, b);
    pcl::PointIndices::Ptr candidate_indices
      (new pcl::PointIndices);
    
    filterPointCloud(cloud, polygon, *candidate_indices);
    pcl::PointCloud<pcl::Normal>::Ptr normals
      (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr normals_cloud
      (new pcl::PointCloud<pcl::PointXYZ>);
    if (!use_normal_) {
      normalEstimate(cloud, candidate_indices, *normals, *normals_cloud);
    }
    else {
      // we don't need to compute normal
      pcl::PointCloud<pcl::Normal>::Ptr all_normals
        (new pcl::PointCloud<pcl::Normal>);
      pcl::fromROSMsg(*cloud_msg, *all_normals);
      pcl::ExtractIndices<pcl::PointXYZ> xyz_extract;
      xyz_extract.setInputCloud(cloud);
      xyz_extract.setIndices(candidate_indices);
      xyz_extract.filter(*normals_cloud);
      
      pcl::ExtractIndices<pcl::Normal> normal_extract;
      normal_extract.setInputCloud(all_normals);
      normal_extract.setIndices(candidate_indices);
      normal_extract.filter(*normals);
    }
    fittingCylinder(normals_cloud, normals, a, b);
    ros::Time end_time = ros::Time::now();
    NODELET_WARN("detection time: %f", (end_time - start_time).toSec());
  }

  void HintedStickFinder::normalEstimate(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const pcl::PointIndices::Ptr indices,
    pcl::PointCloud<pcl::Normal>& normals,
    pcl::PointCloud<pcl::PointXYZ>& normals_cloud)
  {
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    ne.setIndices(indices);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree
      (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod(tree);
    ne.setRadiusSearch (0.03);
    ne.compute (normals);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.filter(normals_cloud);
  }

  bool HintedStickFinder::rejected2DHint(
    const jsk_recognition_utils::Cylinder::Ptr& cylinder,
    const Eigen::Vector3f& a,
    const Eigen::Vector3f& b)
  {
    Eigen::Vector3f hint_dir((b - a));
    hint_dir[2] = 0;
    hint_dir.normalize();
    Eigen::Vector3f cylinder_dir(cylinder->getDirection());
    cylinder_dir[2] = 0;
    cylinder_dir.normalize();
    double ang = acos(cylinder_dir.dot(hint_dir));
    NODELET_INFO("angle: %f", ang);
    return !(ang < eps_2d_angle_ || (M_PI - ang) < eps_2d_angle_);
  }
  
  void HintedStickFinder::fittingCylinder(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr& cloud_normals,
    const Eigen::Vector3f& a,
    const Eigen::Vector3f& b)
  {
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    Eigen::Vector3f normal = (a - b).normalized();
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_CYLINDER);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (outlier_threshold_);
    seg.setMaxIterations (max_iteration_);
    seg.setNormalDistanceWeight (0.1);
    seg.setRadiusLimits(min_radius_, max_radius_);
    // seg.setEpsAngle(eps_angle_);
    // seg.setAxis(normal);
    seg.setProbability(min_probability_);
    seg.setInputCloud(filtered_cloud);
    seg.setInputNormals(cloud_normals);
    for (size_t i = 0; i < cylinder_fitting_trial_; i++) {
      seg.segment(*inliers, *coefficients);
      if (inliers->indices.size() > min_inliers_) {
        Eigen::Vector3f dir(coefficients->values[3],
                            coefficients->values[4],
                            coefficients->values[5]);
        if (dir.dot(Eigen::Vector3f(0, -1, 0)) < 0) {
          dir = -dir;
        }
        jsk_recognition_utils::Cylinder::Ptr cylinder(new jsk_recognition_utils::Cylinder(Eigen::Vector3f(
                                                                                            coefficients->values[0],
                                                                                            coefficients->values[1],
                                                                                            coefficients->values[2]),
                                                                                          dir,
                                                                                          coefficients->values[6]));
        pcl::PointIndices::Ptr cylinder_indices
          (new pcl::PointIndices);
        cylinder->filterPointCloud(*filtered_cloud,
                                   outlier_threshold_,
                                   *cylinder_indices);
        double height = 0;
        Eigen::Vector3f center;
        cylinder->estimateCenterAndHeight(
          *filtered_cloud, *cylinder_indices,
          center, height);
        if (!rejected2DHint(cylinder, a, b)) {
          Eigen::Vector3f uz = Eigen::Vector3f(
            dir).normalized();
          // build maker
          visualization_msgs::Marker marker;
          cylinder->toMarker(marker, center, uz, height);
          pcl_conversions::fromPCL(filtered_cloud->header, marker.header);
          pub_cylinder_marker_.publish(marker);
          geometry_msgs::PoseStamped pose;
          pose.header = marker.header;
          pose.pose = marker.pose;
          pub_cylinder_pose_.publish(pose);
          
          PCLIndicesMsg ros_inliers;
          pcl_conversions::fromPCL(*inliers, ros_inliers);
          pub_inliers_.publish(ros_inliers);
          PCLModelCoefficientMsg ros_coefficients;
          ros_coefficients.header = pcl_conversions::fromPCL(coefficients->header);
          ros_coefficients.values.push_back(center[0]);
          ros_coefficients.values.push_back(center[1]);
          ros_coefficients.values.push_back(center[2]);
          ros_coefficients.values.push_back(dir[0]);
          ros_coefficients.values.push_back(dir[1]);
          ros_coefficients.values.push_back(dir[2]);
          ros_coefficients.values.push_back(coefficients->values[6]);
          ros_coefficients.values.push_back(height);
          pub_coefficients_.publish(ros_coefficients);
        return;
        }
      }
      NODELET_WARN("failed to detect cylinder [%lu/%d]", i, cylinder_fitting_trial_);
    }
  }
  
  void HintedStickFinder::filterPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const jsk_recognition_utils::ConvexPolygon::Ptr polygon,
    pcl::PointIndices& output_indices)
  {
    output_indices.indices.clear();
    for (size_t i = 0; i < cloud->points.size(); i++) {
      pcl::PointXYZ p = cloud->points[i];
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
        if (polygon->isProjectableInside(p.getVector3fMap())) {
          if (polygon->distanceSmallerThan(p.getVector3fMap(), filter_distance_)) {
            output_indices.indices.push_back(i);
          }
        }
      }
    }
    output_indices.header = cloud->header;
    PCLIndicesMsg ros_indices;
    pcl_conversions::fromPCL(output_indices, ros_indices);
    pub_line_filtered_indices_.publish(ros_indices);
  }


  jsk_recognition_utils::ConvexPolygon::Ptr HintedStickFinder::polygonFromLine(
    const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg,
    const image_geometry::PinholeCameraModel& model,
    Eigen::Vector3f& a,
    Eigen::Vector3f& b)
  {
    cv::Point2d point_a(polygon_msg->polygon.points[0].x,
                        polygon_msg->polygon.points[0].y);
    cv::Point2d point_b(polygon_msg->polygon.points[1].x,
                        polygon_msg->polygon.points[1].y);
    cv::Point3d ray_a = model.projectPixelTo3dRay(point_a);
    cv::Point3d ray_b = model.projectPixelTo3dRay(point_b);
    a = Eigen::Vector3f(ray_a.x, ray_a.y, ray_a.z);
    b = Eigen::Vector3f(ray_b.x, ray_b.y, ray_b.z);
    // 20m is far enough??
    Eigen::Vector3f far_a = 20.0 * a;
    Eigen::Vector3f far_b = 20.0 * b;
    Eigen::Vector3f O(0, 0, 0);
    jsk_recognition_utils::Vertices vertices;
    vertices.push_back(O);
    vertices.push_back(far_a);
    vertices.push_back(far_b);
    jsk_recognition_utils::ConvexPolygon::Ptr polygon (new jsk_recognition_utils::ConvexPolygon(vertices));
    return polygon;
  }

  void HintedStickFinder::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    min_radius_ = config.min_radius;
    max_radius_ = config.max_radius;
    filter_distance_ = config.filter_distance;
    outlier_threshold_ = config.outlier_threshold;
    max_iteration_ = config.max_iteration;
    eps_angle_ = config.eps_angle;
    min_probability_ = config.min_probability;
    cylinder_fitting_trial_ = config.cylinder_fitting_trial;
    min_inliers_ = config.min_inliers;
    eps_2d_angle_ = config.eps_2d_angle;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HintedStickFinder, nodelet::Nodelet);
