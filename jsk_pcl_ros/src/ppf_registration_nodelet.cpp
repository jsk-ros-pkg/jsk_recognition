// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include <pcl/features/ppf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ppf_registration.h>
#include "jsk_pcl_ros/ppf_registration.h"

namespace jsk_pcl_ros
{
  void PPFRegistration::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PPFRegistration::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_points_array_ = advertise<jsk_recognition_msgs::PointsArray>(*pnh_, "output/points_array", 1);
    pub_pose_array_ = advertise<geometry_msgs::PoseArray>(*pnh_, "output/pose_array", 1);
    pub_pose_stamped_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output/pose_stamped", 1);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output/cloud", 1);
    onInitPostProcess();
  }

  void PPFRegistration::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input/cloud", 1);
    sub_reference_array_.subscribe(*pnh_, "input/reference_array", 1);
    sub_reference_cloud_.subscribe(*pnh_, "input/reference_cloud", 1);
    if (use_array_)
    {
      if (approximate_sync_)
      {
        array_async_ = boost::make_shared<message_filters::Synchronizer<ArrayApproximateSyncPolicy> >(queue_size_);
        array_async_->connectInput(sub_input_, sub_reference_array_);
        array_async_->registerCallback(boost::bind(&PPFRegistration::ArrayRegistration, this, _1, _2));
      }
      else
      {
        array_sync_ = boost::make_shared<message_filters::Synchronizer<ArraySyncPolicy> >(queue_size_);
        array_sync_->connectInput(sub_input_, sub_reference_array_);
        array_sync_->registerCallback(boost::bind(&PPFRegistration::ArrayRegistration, this, _1, _2));
      }
    }
    else
    {
      if (approximate_sync_)
      {
        cloud_async_ = boost::make_shared<message_filters::Synchronizer<CloudApproximateSyncPolicy> >(queue_size_);
        cloud_async_->connectInput(sub_input_, sub_reference_cloud_);
        cloud_async_->registerCallback(boost::bind(&PPFRegistration::CloudRegistration, this, _1, _2));
      }
      else
      {
        cloud_sync_ = boost::make_shared<message_filters::Synchronizer<CloudSyncPolicy> >(queue_size_);
        cloud_sync_->connectInput(sub_input_, sub_reference_cloud_);
        cloud_sync_->registerCallback(boost::bind(&PPFRegistration::CloudRegistration, this, _1, _2));
      }
    }
  }

  void PPFRegistration::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_reference_array_.unsubscribe();
    sub_reference_cloud_.unsubscribe();
  }

  pcl::PointCloud<pcl::PointNormal>::Ptr PPFRegistration::calculateNormals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    float normal_estimation_search_radius = float(search_radius_);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal> ());
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation_filter;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_calculated (new pcl::PointCloud<pcl::PointNormal> ());
    normal_estimation_filter.setInputCloud (cloud);
    normal_estimation_filter.setSearchMethod (search_tree);
    normal_estimation_filter.setRadiusSearch (normal_estimation_search_radius);
    normal_estimation_filter.compute (*cloud_normals);
    pcl::concatenateFields (*cloud, *cloud_normals, *cloud_calculated);

    // DEBUG
    NODELET_INFO_STREAM("cloud with normals size:" << cloud_calculated->points.size());
    return cloud_calculated;
  }

  void PPFRegistration::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    use_array_ = config.use_array;
    queue_size_ = config.queue_size;
    approximate_sync_ = config.approximate_sync;
    search_radius_ = config.search_radius;
    sampling_rate_ = config.sampling_rate;
  }

  void PPFRegistration::CloudRegistration
  (const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
   const sensor_msgs::PointCloud2::ConstPtr& input_reference_cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg(*input_cloud, *cloud);

    //calculate normals for target
    cloud_with_normals = calculateNormals (cloud);

    // training from reference
    // convert from ROSMsg -> pcl PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg(*input_reference_cloud, *reference_cloud);

    // calculate normal for reference
    reference_cloud_with_normals = calculateNormals (reference_cloud);

    // ppf estimation registration
    pcl::PointCloud<pcl::PPFSignature>::Ptr reference_cloud_ppf (new pcl::PointCloud<pcl::PPFSignature> ());
    ppf_estimator.setInputCloud (reference_cloud_with_normals);
    ppf_estimator.setInputNormals (reference_cloud_with_normals);
    ppf_estimator.compute (*reference_cloud_ppf);

    // hashmap search
    pcl::PPFHashMapSearch::Ptr hashmap_search (new pcl::PPFHashMapSearch (12.0f / 180.0f * float (M_PI), 0.05f));
    hashmap_search->setInputFeatureCloud (reference_cloud_ppf);

    // register references to target
    // set parameters for the PPF registration procedure
    ppf_registration.setSceneReferencePointSamplingRate (sampling_rate_);
    ppf_registration.setPositionClusteringThreshold (float(position_clustering_threshold_));
    ppf_registration.setRotationClusteringThreshold (float(rotation_clustering_threshold_) / 180.0f * float (M_PI));
    ppf_registration.setSearchMethod (hashmap_search);
    ppf_registration.setInputSource (reference_cloud_with_normals);
    ppf_registration.setInputTarget (cloud_with_normals);
    pcl::PointCloud<pcl::PointNormal> cloud_output_subsampled;
    ppf_registration.align (cloud_output_subsampled);

    // get ppf transformation
    Eigen::Matrix4f mat = ppf_registration.getFinalTransformation ();
    Eigen::Affine3f pose (mat);
    // DEBUG
    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    NODELET_INFO_STREAM( "Matrix:\n" << mat.format(CleanFmt));

    // transform reference
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::transformPointCloud (*reference_cloud, *cloud_output, pose);

    //convert pcl to ROSMsg
    sensor_msgs::PointCloud2 cloud_msg;
    toROSMsg(*cloud_output, cloud_msg);
    geometry_msgs::Pose pose_msg_;
    geometry_msgs::PoseStamped pose_stamped_msg;
    tf::poseEigenToMsg(pose, pose_msg_);
    pose_stamped_msg.pose = pose_msg_;

    // publish
    pose_stamped_msg.header = input_cloud->header;
    cloud_msg.header = input_cloud->header;
    pub_pose_stamped_.publish(pose_stamped_msg);
    pub_cloud_.publish(cloud_msg);
  }

  void PPFRegistration::ArrayRegistration
  (const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
   const jsk_recognition_msgs::PointsArray::ConstPtr& input_reference_points_array)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::fromROSMsg(*input_cloud, *cloud);

    //calculate normals for target
    cloud_with_normals = calculateNormals (cloud);

    jsk_recognition_msgs::PointsArray::Ptr points_array_msg (new jsk_recognition_msgs::PointsArray ());
    geometry_msgs::PoseArray::Ptr pose_array_msg (new geometry_msgs::PoseArray ());
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr > reference_cloud_vector;
    std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr > reference_cloud_with_normals_vector;
    std::vector<pcl::PPFHashMapSearch::Ptr > hashmap_search_vector;

    // training from reference
    for (size_t reference_i=0; reference_i < input_reference_points_array->cloud_list.size(); reference_i++)
    {
      sensor_msgs::PointCloud2 input_reference_cloud = input_reference_points_array->cloud_list[reference_i];

      // convert from ROSMsg -> pcl PointCloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr reference_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::fromROSMsg(input_reference_cloud, *reference_cloud);
      reference_cloud_vector.push_back(reference_cloud);

      // calculate normal for reference
      reference_cloud_with_normals = calculateNormals (reference_cloud);
      reference_cloud_with_normals_vector.push_back(reference_cloud_with_normals);

      // ppf estimation registration
      pcl::PointCloud<pcl::PPFSignature>::Ptr reference_cloud_ppf (new pcl::PointCloud<pcl::PPFSignature> ());
      ppf_estimator.setInputCloud (reference_cloud_with_normals);
      ppf_estimator.setInputNormals (reference_cloud_with_normals);
      ppf_estimator.compute (*reference_cloud_ppf);

      // hashmap search
      pcl::PPFHashMapSearch::Ptr hashmap_search (new pcl::PPFHashMapSearch (12.0f / 180.0f * float (M_PI), 0.05f));
      hashmap_search->setInputFeatureCloud (reference_cloud_ppf);
      hashmap_search_vector.push_back(hashmap_search);
    }

    // register references to target
    for (size_t reference_i=0; reference_i < input_reference_points_array->cloud_list.size(); reference_i++)
    {
      // set parameters for the PPF registration procedure
      ppf_registration.setSceneReferencePointSamplingRate (sampling_rate_);
      ppf_registration.setPositionClusteringThreshold (float(position_clustering_threshold_));
      ppf_registration.setRotationClusteringThreshold (float(rotation_clustering_threshold_) / 180.0f * float (M_PI));
      ppf_registration.setSearchMethod (hashmap_search_vector[reference_i]);
      ppf_registration.setInputSource (reference_cloud_with_normals_vector[reference_i]);
      ppf_registration.setInputTarget (cloud_with_normals);
      pcl::PointCloud<pcl::PointNormal> cloud_output_subsampled;
      ppf_registration.align (cloud_output_subsampled);

      // get ppf transformation
      Eigen::Matrix4f mat = ppf_registration.getFinalTransformation ();
      Eigen::Affine3f pose (mat);
      // DEBUG
      Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
      NODELET_INFO_STREAM( "Matrix:\n" << mat.format(CleanFmt));

      // transform reference
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_output (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*reference_cloud_vector[reference_i], *cloud_output, pose);

      //convert pcl to ROSMsg
      sensor_msgs::PointCloud2 cloud_msg_;
      toROSMsg(*cloud_output, cloud_msg_);
      geometry_msgs::Pose pose_msg_;
      tf::poseEigenToMsg(pose, pose_msg_);
      points_array_msg->cloud_list.push_back(cloud_msg_);
      pose_array_msg->poses.push_back(pose_msg_);
    }
    // publish
    pose_array_msg->header = input_cloud->header;
    points_array_msg->header = input_cloud->header;
    pub_pose_array_.publish(pose_array_msg);
    pub_points_array_.publish(points_array_msg);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PPFRegistration, nodelet::Nodelet);
