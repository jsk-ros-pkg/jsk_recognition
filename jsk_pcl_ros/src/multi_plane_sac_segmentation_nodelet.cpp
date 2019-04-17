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

#include "jsk_pcl_ros/multi_plane_sac_segmentation.h"
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include "jsk_recognition_utils/pcl_util.h"
#include <eigen_conversions/eigen_msg.h>

namespace jsk_pcl_ros
{
  void MultiPlaneSACSegmentation::onInit()
  {
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&MultiPlaneSACSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("use_normal", use_normal_, false);
    pnh_->param("use_clusters", use_clusters_, false);
    pnh_->param("use_imu_parallel", use_imu_parallel_, false);
    pnh_->param("use_imu_perpendicular", use_imu_perpendicular_, false);
    
    if (use_imu_perpendicular_ && use_imu_parallel_) {
      NODELET_ERROR("Cannot use ~use_imu_perpendicular and ~use_imu_parallel at the same time");
      return;
    }
    if (use_imu_perpendicular_ || use_imu_parallel_) {
      tf_listener_ = TfListenerSingleton::getInstance();
    }
    ////////////////////////////////////////////////////////
    // publishers
    ////////////////////////////////////////////////////////
    pub_inliers_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output_indices", 1);
    pub_coefficients_
      = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(*pnh_, "output_coefficients", 1);
    pub_polygons_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output_polygons", 1);
    onInitPostProcess();
  }

  void MultiPlaneSACSegmentation::subscribe()
  {
    ////////////////////////////////////////////////////////
    // subscriber
    ////////////////////////////////////////////////////////
    if (use_imu_perpendicular_ || use_imu_parallel_) {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_imu_.subscribe(*pnh_, "input_imu", 1);
      if (use_normal_) {
        sub_normal_.subscribe(*pnh_, "input_normal", 1);
        sync_normal_imu_ = boost::make_shared<NormalImuSynchronizer>(100);
        sync_normal_imu_->connectInput(sub_input_, sub_normal_, sub_imu_);
        sync_normal_imu_->registerCallback(
          boost::bind(
            &MultiPlaneSACSegmentation::segmentWithImu,
            this, _1, _2, _3));
      }
      else {
        sync_imu_ = boost::make_shared<message_filters::Synchronizer<SyncImuPolicy> >(100);
        sync_imu_->connectInput(sub_input_, sub_imu_);
        sync_imu_->registerCallback(boost::bind(
                                      &MultiPlaneSACSegmentation::segmentWithImu,
                                      this, _1, _2));
      }
    }
    else if (use_normal_) {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_normal_.subscribe(*pnh_, "input_normal", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_normal_);
      sync_->registerCallback(boost::bind(&MultiPlaneSACSegmentation::segment,
                                          this, _1, _2));
    }
    else if (use_clusters_) {
      NODELET_INFO("use clusters");
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_clusters_.subscribe(*pnh_, "input_clusters", 1);
      sync_cluster_
        = boost::make_shared<message_filters::Synchronizer<SyncClusterPolicy> >(100);
      sync_cluster_->connectInput(sub_input_, sub_clusters_);
      sync_cluster_->registerCallback(
        boost::bind(&MultiPlaneSACSegmentation::segmentWithClusters,
                    this, _1, _2));
    }
    else {
      sub_ = pnh_->subscribe("input", 1, &MultiPlaneSACSegmentation::segment, this);
    }
  }

  void MultiPlaneSACSegmentation::unsubscribe()
  {
    ////////////////////////////////////////////////////////
    // subscriber
    ////////////////////////////////////////////////////////
    if (use_normal_) {
      sub_input_.unsubscribe();
      sub_normal_.unsubscribe();
    }
    else if (use_clusters_) {
      sub_input_.unsubscribe();
      sub_clusters_.unsubscribe();
    }
    else {
      sub_.shutdown();
    }
  }

  void MultiPlaneSACSegmentation::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    outlier_threshold_ = config.outlier_threshold;
    min_inliers_ = config.min_inliers;
    min_points_ = config.min_points;
    max_iterations_ = config.max_iterations;
    eps_angle_ = config.eps_angle;
    normal_distance_weight_ = config.normal_distance_weight;
    min_trial_ = config.min_trial;
  }
  
  void MultiPlaneSACSegmentation::applyRecursiveRANSAC(
      const pcl::PointCloud<PointT>::Ptr& input,
      const pcl::PointCloud<pcl::Normal>::Ptr& normal,
      const Eigen::Vector3f& imu_vector,
      std::vector<pcl::PointIndices::Ptr>& output_inliers,
      std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients,
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& output_polygons)
  {
    pcl::PointCloud<PointT>::Ptr rest_cloud (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr rest_normal (new pcl::PointCloud<pcl::Normal>);
    *rest_cloud = *input;
    *rest_normal = *normal;
    int counter = 0;
    while (true) {
      NODELET_INFO("apply RANSAC: %d", counter);
      ++counter;
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      if (!use_normal_) {
        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients (true);
        seg.setRadiusLimits(0.3, std::numeric_limits<double>::max ());
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (outlier_threshold_);
        seg.setInputCloud(rest_cloud);
        seg.setMaxIterations (max_iterations_);
        if (use_imu_parallel_) {
          seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
          seg.setAxis(imu_vector);
          seg.setEpsAngle(eps_angle_);
        }
        else if (use_imu_perpendicular_) {
          seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
          seg.setAxis(imu_vector);
          seg.setEpsAngle(eps_angle_);
        }
        else {
          seg.setModelType (pcl::SACMODEL_PLANE);
        }
        seg.segment (*inliers, *coefficients);
      }
      else {
        pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;
        seg.setOptimizeCoefficients (true);
        
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (outlier_threshold_);
        seg.setNormalDistanceWeight(normal_distance_weight_);
        seg.setInputCloud(rest_cloud);
        seg.setInputNormals(rest_normal);
        seg.setMaxIterations (max_iterations_);
        if (use_imu_parallel_) {
          seg.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
          seg.setAxis(imu_vector);
          seg.setEpsAngle(eps_angle_);
        }
        else if (use_imu_perpendicular_) {
          seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
          seg.setAxis(imu_vector);
          seg.setEpsAngle(eps_angle_);
        }
        else {
          seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);  
        }
        seg.segment (*inliers, *coefficients);
      }
      NODELET_INFO("inliers: %lu", inliers->indices.size());
      if (inliers->indices.size() >= min_inliers_) {
        output_inliers.push_back(inliers);
        output_coefficients.push_back(coefficients);
        jsk_recognition_utils::ConvexPolygon::Ptr convex = jsk_recognition_utils::convexFromCoefficientsAndInliers<PointT>(
          input, inliers, coefficients);
        output_polygons.push_back(convex);
      }
      else {
        if (min_trial_ <= counter) {
          return;
        }
      }
      
      // prepare for next loop
      pcl::PointCloud<PointT>::Ptr next_rest_cloud
        (new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr next_rest_normal
        (new pcl::PointCloud<pcl::Normal>);
      pcl::ExtractIndices<PointT> ex;
      ex.setInputCloud(rest_cloud);
      ex.setIndices(inliers);
      ex.setNegative(true);
      ex.setKeepOrganized(true);
      ex.filter(*next_rest_cloud);
      if (use_normal_) {
        pcl::ExtractIndices<pcl::Normal> ex_normal;
        ex_normal.setInputCloud(rest_normal);
        ex_normal.setIndices(inliers);
        ex_normal.setNegative(true);
        ex_normal.setKeepOrganized(true);
        ex_normal.filter(*next_rest_normal);
      }
      if (next_rest_cloud->points.size() < min_points_) {
        NODELET_INFO("no more enough points are left");
        return;
      }
      rest_cloud = next_rest_cloud;
      rest_normal = next_rest_normal;
    }
  }

  void MultiPlaneSACSegmentation::segmentWithClusters(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& clusters)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *input);
    std::vector<pcl::PointIndices::Ptr> cluster_indices
      = pcl_conversions::convertToPCLPointIndices(clusters->cluster_indices);
    pcl::ExtractIndices<PointT> ex;
    ex.setInputCloud(input);
    ex.setKeepOrganized(true);
    std::vector<pcl::PointIndices::Ptr> all_inliers;
    std::vector<pcl::ModelCoefficients::Ptr> all_coefficients;
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> all_convexes;
    for (size_t i = 0; i < cluster_indices.size(); i++) {
      pcl::PointIndices::Ptr indices = cluster_indices[i];
      pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
      pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
      ex.setIndices(indices);
      
      ex.filter(*cluster_cloud);
      std::vector<pcl::PointIndices::Ptr> inliers;
      std::vector<pcl::ModelCoefficients::Ptr> coefficients;
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes;
      Eigen::Vector3f dummy_imu_vector;
      applyRecursiveRANSAC(cluster_cloud, normal, dummy_imu_vector, inliers, coefficients, convexes);
      appendVector(all_inliers, inliers);
      appendVector(all_coefficients, coefficients);
      appendVector(all_convexes, convexes);
    }
    publishResult(msg->header, all_inliers, all_coefficients, all_convexes);
  }

  void MultiPlaneSACSegmentation::publishResult(
    const std_msgs::Header& header,
    const std::vector<pcl::PointIndices::Ptr>& inliers,
    const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
    const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes)
  {
    jsk_recognition_msgs::ClusterPointIndices ros_indices_output;
    jsk_recognition_msgs::ModelCoefficientsArray ros_coefficients_output;
    jsk_recognition_msgs::PolygonArray ros_polygon_output;
    ros_indices_output.header = header;
    ros_coefficients_output.header = header;
    ros_polygon_output.header = header;
    ros_indices_output.cluster_indices
      = pcl_conversions::convertToROSPointIndices(inliers, header);
    ros_coefficients_output.coefficients
      = pcl_conversions::convertToROSModelCoefficients(coefficients, header);
    pub_inliers_.publish(ros_indices_output);
    pub_coefficients_.publish(ros_coefficients_output);
    for (size_t i = 0; i < convexes.size(); i++) {
      geometry_msgs::PolygonStamped polygon;
      polygon.header = header;
      polygon.polygon = convexes[i]->toROSMsg();
      ros_polygon_output.polygons.push_back(polygon);
    }
    pub_polygons_.publish(ros_polygon_output);
  }

  void MultiPlaneSACSegmentation::segmentWithImu(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    segmentWithImu(msg, sensor_msgs::PointCloud2::ConstPtr(), imu_msg);
  }
  
  void MultiPlaneSACSegmentation::segmentWithImu(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::PointCloud2::ConstPtr& normal_msg,
    const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr
      normal (new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*msg, *input);
    if (normal_msg) {
      pcl::fromROSMsg(*normal_msg, *normal);
    }
    geometry_msgs::Vector3Stamped stamped_imu, transformed_stamped_imu;
    stamped_imu.header = imu_msg->header;
    stamped_imu.vector = imu_msg->linear_acceleration;
    try {
      tf_listener_->waitForTransform(
        msg->header.frame_id, imu_msg->header.frame_id, imu_msg->header.stamp,
        ros::Duration(0.1));
      tf_listener_->transformVector(
        msg->header.frame_id, stamped_imu, transformed_stamped_imu); 
      Eigen::Vector3d imu_vectord;
      Eigen::Vector3f imu_vector;
      tf::vectorMsgToEigen(transformed_stamped_imu.vector, imu_vectord);
      jsk_recognition_utils::pointFromVectorToVector<Eigen::Vector3d, Eigen::Vector3f>(
        imu_vectord, imu_vector);
      imu_vector = - imu_vector;
      
      std::vector<pcl::PointIndices::Ptr> inliers;
      std::vector<pcl::ModelCoefficients::Ptr> coefficients;
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes;
      applyRecursiveRANSAC(input, normal, imu_vector,
                           inliers, coefficients, convexes);
      publishResult(msg->header, inliers, coefficients, convexes);                     
    }
    catch (tf2::ConnectivityException &e) {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::InvalidArgumentException &e) {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::ExtrapolationException& e) {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    
  }
  
  void MultiPlaneSACSegmentation::segment(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::PointCloud2::ConstPtr& msg_normal)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normal (new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*msg, *input);
    if (use_normal_) {
      pcl::fromROSMsg(*msg_normal, *normal);
    }
    std::vector<pcl::PointIndices::Ptr> inliers;
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    std::vector<jsk_recognition_utils::ConvexPolygon::Ptr> convexes;
    Eigen::Vector3f dummy_imu_vector;
    applyRecursiveRANSAC(input, normal, dummy_imu_vector,
                         inliers, coefficients, convexes);
    publishResult(msg->header, inliers, coefficients, convexes); 
  }
  
  void MultiPlaneSACSegmentation::segment(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    segment(msg, sensor_msgs::PointCloud2::ConstPtr());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::MultiPlaneSACSegmentation, nodelet::Nodelet);
