// -*- mode: c++; indent-tabs-mode: nil; -*-
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

#include "jsk_pcl_ros/icp_registration.h"
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid_3d.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include "jsk_pcl_ros_utils/transform_pointcloud_in_bounding_box.h"
#include <image_geometry/pinhole_camera_model.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>
#include <pcl/registration/correspondence_estimation_normal_shooting.h>
#include <jsk_recognition_utils/pcl_ros_util.h>

namespace jsk_pcl_ros
{
  void ICPRegistration::onInit()
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    ConnectionBasedNodelet::onInit();
    tf_listener_ = TfListenerSingleton::getInstance();
    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(
        &ICPRegistration::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("use_normal", use_normal_, false);
    pnh_->param("align_box", align_box_, false);
    pnh_->param("synchronize_reference", synchronize_reference_, false);
    pnh_->param("transform_3dof", transform_3dof_, false);
    pnh_->param("use_offset_pose", use_offset_pose_, false);
    ////////////////////////////////////////////////////////
    // Publishers
    ////////////////////////////////////////////////////////
    pub_result_pose_ = advertise<geometry_msgs::PoseStamped>(*pnh_,
      "output_pose", 1);
    pub_result_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_,
      "output", 1);
    pub_debug_source_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_,
      "debug/source", 1);
    pub_debug_target_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_,
      "debug/target", 1);
    pub_debug_flipped_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_,
      "debug/flipped", 1);
    pub_debug_result_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_,
      "debug/result", 1);
    pub_icp_result = advertise<jsk_recognition_msgs::ICPResult>(*pnh_,
      "icp_result", 1);
    pub_latest_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/latest_time", 1);
    pub_average_time_ = advertise<std_msgs::Float32>(
      *pnh_, "output/average_time", 1);
    srv_icp_align_with_box_ = pnh_->advertiseService("icp_service", &ICPRegistration::alignWithBoxService, this);
    srv_icp_align_ = pnh_->advertiseService(
      "icp_align", &ICPRegistration::alignService, this);
    if (!synchronize_reference_) {
      sub_reference_ = pnh_->subscribe("input_reference", 1,
                                       &ICPRegistration::referenceCallback,
                                       this);
      sub_reference_array_ = pnh_->subscribe("input_reference_array", 1,
                                           &ICPRegistration::referenceArrayCallback,
                                             this);
      sub_reference_add = pnh_->subscribe("input_reference_add", 1,
                                          &ICPRegistration::referenceAddCallback,
                                          this);
    }
    done_init_ = true;
    onInitPostProcess();
  }

  void ICPRegistration::subscribe()
  {
    ////////////////////////////////////////////////////////
    // Subscription
    ////////////////////////////////////////////////////////
    sub_camera_info_ = pnh_->subscribe("input/camera_info", 1,
                                       &ICPRegistration::cameraInfoCallback,
                                       this);
    if (!synchronize_reference_) {
      if (align_box_) {
        sub_input_.subscribe(*pnh_, "input", 1);
        sub_box_.subscribe(*pnh_, "input_box", 1);
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(sub_input_, sub_box_);
        sync_->registerCallback(boost::bind(
                                            &ICPRegistration::alignWithBox,
                                            this, _1, _2));
      }
      else if (use_offset_pose_) {
        sub_input_.subscribe(*pnh_, "input", 1);
        sub_offset_.subscribe(*pnh_, "input_offset", 1);
        sync_offset_ = boost::make_shared<message_filters::Synchronizer<OffsetSyncPolicy> >(100);
        sync_offset_->connectInput(sub_input_, sub_offset_);
        sync_offset_->registerCallback(boost::bind(&ICPRegistration::alignWithOffset, this, _1, _2));
      }
      else {
        sub_ = pnh_->subscribe("input", 1,
                               &ICPRegistration::align,
                               this);
      }
    }
    else {
      sub_sync_input_.subscribe(*pnh_, "input", 1);
      sub_sync_reference_.subscribe(*pnh_, "reference", 1);
      sync_reference_ = boost::make_shared<message_filters::Synchronizer<ReferenceSyncPolicy> >(100);
      sync_reference_->connectInput(sub_sync_input_, sub_sync_reference_);
      sync_reference_->registerCallback(boost::bind(&ICPRegistration::align, this, _1, _2));
    }
  }

  void ICPRegistration::unsubscribe()
  {
    sub_camera_info_.shutdown();
    if (!synchronize_reference_) {
      if (align_box_) {
        sub_input_.unsubscribe();
        sub_box_.unsubscribe();
      }
      else {
        sub_.shutdown();
      }
    }
    else {
      sub_sync_input_.unsubscribe();
      sub_sync_reference_.unsubscribe();
    }
  }
  
  void ICPRegistration::publishDebugCloud(
      ros::Publisher& pub,
      const pcl::PointCloud<PointT>& cloud,
      const std_msgs::Header& header)
  {
    if (pub.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(cloud, ros_cloud);
      ros_cloud.header = header;
      pub.publish(ros_cloud);
    }
  }
  
  void ICPRegistration::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    algorithm_ = config.algorithm;
    correspondence_algorithm_ = config.correspondence_algorithm;
    use_flipped_initial_pose_ = config.use_flipped_initial_pose;
    max_iteration_ = config.max_iteration;
    correspondence_distance_ = config.correspondence_distance;
    transform_epsilon_ = config.transform_epsilon;
    euclidean_fittness_epsilon_ = config.euclidean_fittness_epsilon;
    rotation_epsilon_ = config.rotation_epsilon;
    ransac_iterations_ = config.ransac_iterations;
    ransac_outlier_threshold_ = config.ransac_outlier_threshold;
    correspondence_randomness_ = config.correspondence_randomness;
    maximum_optimizer_iterations_ = config.maximum_optimizer_iterations;
    ndt_resolution_ = config.ndt_resolution;
    ndt_step_size_ = config.ndt_step_size;
    ndt_outlier_ratio_ = config.ndt_outlier_ratio;
  }

  bool ICPRegistration::alignWithBoxService(
    jsk_recognition_msgs::ICPAlignWithBox::Request& req, 
    jsk_recognition_msgs::ICPAlignWithBox::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (reference_cloud_list_.size() == 0) {
      NODELET_FATAL("no reference is specified");
      return false;
    }
    try
    {
      Eigen::Affine3f offset;
      pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
      jsk_pcl_ros_utils::transformPointcloudInBoundingBox<PointT>(
        req.target_box, req.target_cloud,
        *output, offset,
        *tf_listener_);
      Eigen::Affine3f inversed_offset = offset.inverse();
      res.result = alignPointcloudWithReferences(output, inversed_offset, req.target_cloud.header);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
      return false;
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
      return false;
    }
    return true;
  }

  bool ICPRegistration::alignService(
    jsk_recognition_msgs::ICPAlign::Request& req, 
    jsk_recognition_msgs::ICPAlign::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    std::vector<pcl::PointCloud<PointT>::Ptr> tmp_reference_cloud_list
      = reference_cloud_list_;  // escape
    try
    {
      // first, update reference
      std::vector<pcl::PointCloud<PointT>::Ptr> new_references;
      pcl::PointCloud<PointT>::Ptr reference_cloud (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(req.reference_cloud, *reference_cloud);
      pcl::PointCloud<PointT>::Ptr non_nan_reference_cloud (new pcl::PointCloud<PointT>);
      for (size_t i = 0; i < reference_cloud->points.size(); i++) {
        PointT p = reference_cloud->points[i];
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
          non_nan_reference_cloud->points.push_back(p);
        }
      }
      new_references.push_back(non_nan_reference_cloud);
      reference_cloud_list_ = new_references; // replace references
      NODELET_INFO("reference points: %lu/%lu",
                   non_nan_reference_cloud->points.size(),
                   reference_cloud->points.size());
      Eigen::Affine3f offset = Eigen::Affine3f::Identity();
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(req.target_cloud, *cloud);
      res.result = alignPointcloudWithReferences(cloud,
                                                 offset,
                                                 req.target_cloud.header);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
      reference_cloud_list_ = tmp_reference_cloud_list;
      return false;
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
      reference_cloud_list_ = tmp_reference_cloud_list;
      return false;
    }
    reference_cloud_list_ = tmp_reference_cloud_list;
    return true;
  }

  
  void ICPRegistration::alignWithBox(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_init_) {
      NODELET_WARN("not yet initialized");
      return;
    }
    if (reference_cloud_list_.size() == 0) {
      NODELET_FATAL("no reference is specified");
      jsk_recognition_msgs::ICPResult result;
      result.name = std::string("NONE");
      result.score = DBL_MAX;
      result.header = box_msg->header;
      result.pose = box_msg->pose;
      pub_icp_result.publish(result);
      return;
    }
    try
    {
      Eigen::Affine3f offset;
      pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
      jsk_pcl_ros_utils::transformPointcloudInBoundingBox<PointT>(
        *box_msg, *msg,
        *output, offset,
        *tf_listener_);
      Eigen::Affine3f inversed_offset = offset.inverse();
      jsk_recognition_msgs::ICPResult result = alignPointcloudWithReferences(output, inversed_offset, msg->header);
      pub_icp_result.publish(result);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
  }

  void ICPRegistration::alignWithOffset(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const geometry_msgs::PoseStamped::ConstPtr& offset_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_init_) {
      NODELET_WARN("not yet initialized");
      return;
    }
    if (reference_cloud_list_.size() == 0) {
      NODELET_FATAL("no reference is specified");
      jsk_recognition_msgs::ICPResult result;
      result.name = std::string("NONE");
      result.score = DBL_MAX;
      result.header = offset_msg->header;
      result.pose = offset_msg->pose;
      pub_icp_result.publish(result);
      return;
    }
    try
    {
      if (!jsk_recognition_utils::isSameFrameId(msg->header.frame_id,
                                                offset_msg->header.frame_id)) {
        NODELET_ERROR("frame_id does not match. cloud: %s, pose: %s",
                          msg->header.frame_id.c_str(),
                          offset_msg->header.frame_id.c_str());
        return;
      }
      Eigen::Affine3f offset;
      pcl::PointCloud<PointT>::Ptr input (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(*msg, *input);
      pcl::PointCloud<PointT>::Ptr output (new pcl::PointCloud<PointT>);
      tf::poseMsgToEigen(offset_msg->pose, offset);
      
      Eigen::Affine3f inversed_offset = offset.inverse();
      pcl::transformPointCloud(*input, *output, inversed_offset);
      jsk_recognition_msgs::ICPResult result = alignPointcloudWithReferences(output, offset, msg->header);
      pub_icp_result.publish(result);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
  }


  void ICPRegistration::align(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_init_) {
      NODELET_WARN("not yet initialized");
      return;
    }
    if (reference_cloud_list_.size() == 0) {
      NODELET_FATAL("no reference is specified");
      return;
    }
    
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    Eigen::Affine3f offset = Eigen::Affine3f::Identity();
    // remove nan
    pcl::PointCloud<PointT>::Ptr non_nan_cloud (new pcl::PointCloud<PointT>);
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
        non_nan_cloud->points.push_back(p);
      }
    }
    jsk_recognition_msgs::ICPResult result = alignPointcloudWithReferences(non_nan_cloud, offset, msg->header);
    pub_icp_result.publish(result);
  }
  
  void ICPRegistration::align(const sensor_msgs::PointCloud2::ConstPtr& msg,
                              const sensor_msgs::PointCloud2::ConstPtr& reference_msg)
  {
    {
      boost::mutex::scoped_lock lock(mutex_);
      if (!done_init_) {
        NODELET_WARN("not yet initialized");
        return;
      }
      reference_cloud_list_.resize(0);
      pcl::PointCloud<PointT>::Ptr reference_cloud (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(*reference_msg, *reference_cloud);
      // remove nan
      pcl::PointCloud<PointT>::Ptr non_nan_reference_cloud (new pcl::PointCloud<PointT>);
      for (size_t i = 0; i < reference_cloud->points.size(); i++) {
        PointT p = reference_cloud->points[i];
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
          non_nan_reference_cloud->points.push_back(p);
        }
      }
      reference_cloud_list_.push_back(non_nan_reference_cloud);
    }
    align(msg);
  }

  jsk_recognition_msgs::ICPResult ICPRegistration::alignPointcloudWithReferences(
    pcl::PointCloud<PointT>::Ptr& cloud,
    const Eigen::Affine3f& offset,
    const std_msgs::Header& header)
  {
    jsk_recognition_utils::ScopedWallDurationReporter r
      = timer_.reporter(pub_latest_time_, pub_average_time_);
    double min_score = DBL_MAX;
    size_t max_index = 0;
    pcl::PointCloud<PointT>::Ptr best_transformed_cloud;
    pcl::PointCloud<PointT>::Ptr best_reference;
    Eigen::Affine3d best_transform_result;
    Eigen::Affine3f best_offset_result;
    jsk_recognition_msgs::ICPResult result;
    if (cloud->empty()) {
      sensor_msgs::PointCloud2 empty_cloud;
      empty_cloud.header = header;
      pub_result_cloud_.publish(empty_cloud);

      pcl::PointCloud<PointT> empty_pcl_cloud;
      publishDebugCloud(pub_debug_source_cloud_, empty_pcl_cloud, header);
      publishDebugCloud(pub_debug_target_cloud_, empty_pcl_cloud, header);
      publishDebugCloud(pub_debug_result_cloud_, empty_pcl_cloud, header);

      geometry_msgs::PoseStamped empty_pose;
      empty_pose.header = header;
      pub_result_pose_.publish(empty_pose);

      result.header = header;
      result.score = min_score;
      result.name = "nan";
      return result;
    }
    for (size_t i = 0; i < reference_cloud_list_.size(); i++) {
      Eigen::Affine3f offset_result;
      pcl::PointCloud<PointT>::Ptr transformed_cloud(new pcl::PointCloud<PointT>);
      Eigen::Affine3d transform_result;
      double score = scorePointcloudAlignment(
        cloud,
        reference_cloud_list_[i],
        offset,
        offset_result,
        transformed_cloud,
        transform_result);
      if (score < min_score) {
        max_index = i;
        min_score = score;
        best_transform_result = transform_result;
        best_transformed_cloud = transformed_cloud;
        best_reference = reference_cloud_list_[i];
        best_offset_result = offset_result;
      }
    }
    
    NODELET_INFO("best score is: %f", min_score);
    if (pub_result_cloud_.getNumSubscribers() > 0) {
      sensor_msgs::PointCloud2 ros_final;
      pcl::toROSMsg(*best_transformed_cloud, ros_final);
      ros_final.header = header;
      pub_result_cloud_.publish(ros_final);
    }
    geometry_msgs::PoseStamped ros_result_pose;
    ros_result_pose.header = header;
    tf::poseEigenToMsg(best_transform_result, ros_result_pose.pose);
    pub_result_pose_.publish(ros_result_pose);
    publishDebugCloud(pub_debug_source_cloud_, *best_reference, header);
    publishDebugCloud(pub_debug_target_cloud_, *cloud, header);
    pcl::PointCloud<PointT>::Ptr transformed_cloud_for_debug_result
      (new pcl::PointCloud<PointT>);
    pcl::transformPointCloud(
      *best_transformed_cloud, *transformed_cloud_for_debug_result,
      best_offset_result.inverse());
    publishDebugCloud(pub_debug_result_cloud_,
                      *transformed_cloud_for_debug_result, header);
    result.header = ros_result_pose.header;
    result.pose = ros_result_pose.pose;
    result.score = min_score;
    std::stringstream ss;
    ss << max_index;
    result.name = ss.str();
    return result;
  }

  void ICPRegistration::cameraInfoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_msg_ = msg;
  }

  double ICPRegistration::alignPointcloud(
    pcl::PointCloud<PointT>::Ptr& cloud,
    pcl::PointCloud<PointT>::Ptr& reference,
    const Eigen::Affine3f& offset,
    pcl::PointCloud<PointT>::Ptr& output_cloud,
    Eigen::Affine3d& output_transform)
  {
    if (algorithm_ == 0 || algorithm_ == 1) {
      return alignPointcloudWithICP(cloud, reference, offset, output_cloud, output_transform);
    }
    else {
      return alignPointcloudWithNDT(cloud, reference, offset, output_cloud, output_transform);
    }
  }

  
  double ICPRegistration::alignPointcloudWithNDT(
    pcl::PointCloud<PointT>::Ptr& cloud,
    pcl::PointCloud<PointT>::Ptr& reference,
    const Eigen::Affine3f& offset,
    pcl::PointCloud<PointT>::Ptr& output_cloud,
    Eigen::Affine3d& output_transform)
  {
    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    if (reference->points.empty ()) {
      NODELET_ERROR("Input Reference Cloud is empty!");
      return DBL_MAX;
    }
    ndt.setInputSource(reference);
    if (cloud->points.empty ()) {
      NODELET_ERROR("Input Target Cloud is empty!");
      return DBL_MAX;
    }
    ndt.setInputTarget(cloud);
    pcl::PointCloud<PointT> final;
    ndt.align(final);
    pcl::transformPointCloud(final, *output_cloud, offset);
    Eigen::Matrix4f transformation = ndt.getFinalTransformation ();
    Eigen::Matrix4d transformation_d;
    jsk_recognition_utils::convertMatrix4<Eigen::Matrix4f, Eigen::Matrix4d>(
      transformation, transformation_d);
    Eigen::Affine3d offsetd;
    convertEigenAffine3(offset, offsetd);
    output_transform = offsetd * Eigen::Affine3d(transformation_d);
    return ndt.getFitnessScore();
  }
  
  double ICPRegistration::alignPointcloudWithICP(
    pcl::PointCloud<PointT>::Ptr& cloud,
    pcl::PointCloud<PointT>::Ptr& reference,
    const Eigen::Affine3f& offset,
    pcl::PointCloud<PointT>::Ptr& output_cloud,
    Eigen::Affine3d& output_transform)
  {
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    if (use_normal_) {
      pcl::IterativeClosestPointWithNormals<PointT, PointT> icp_with_normal;
      icp = icp_with_normal;
    }
    // icp.setInputSource(cloud);
    // icp.setInputTarget(reference_cloud_);
    if (algorithm_ == 1) {
      pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
      gicp.setRotationEpsilon(rotation_epsilon_);
      gicp.setCorrespondenceRandomness(correspondence_randomness_);
      gicp.setMaximumOptimizerIterations(maximum_optimizer_iterations_);
      icp = gicp;
    }
    if (correspondence_algorithm_ == 1) { // Projective
      if (!camera_info_msg_) {
        NODELET_ERROR("no camera info is available yet");
        return DBL_MAX;
      }
      image_geometry::PinholeCameraModel model;
      bool model_success_p = model.fromCameraInfo(camera_info_msg_);
      if (!model_success_p) {
        NODELET_ERROR("failed to create camera model");
        return DBL_MAX;
      }
      pcl::registration::CorrespondenceEstimationOrganizedProjection<PointT, PointT, float>::Ptr
        corr_projection (new pcl::registration::CorrespondenceEstimationOrganizedProjection<PointT, PointT, float>);
      corr_projection->setFocalLengths(model.fx(), model.fy());
      corr_projection->setCameraCenters(model.cx(), model.cy());
      icp.setCorrespondenceEstimation(corr_projection);
    }
    
    if (reference->points.empty ()) {
      NODELET_ERROR("Input Reference Cloud is empty!");
      return DBL_MAX;
    }
    icp.setInputSource(reference);
    if (cloud->points.empty ()) {
      NODELET_ERROR("Input Target Cloud is empty!");
      return DBL_MAX;
    }
    icp.setInputTarget(cloud);

    if (transform_3dof_) {
      boost::shared_ptr<pcl::registration::WarpPointRigid3D<PointT, PointT> > warp_func
        (new pcl::registration::WarpPointRigid3D<PointT, PointT>);
      boost::shared_ptr<pcl::registration::TransformationEstimationLM<PointT, PointT> > te
        (new pcl::registration::TransformationEstimationLM<PointT, PointT>);
      te->setWarpFunction(warp_func);
      icp.setTransformationEstimation(te);
    }

    icp.setMaxCorrespondenceDistance (correspondence_distance_);
    icp.setMaximumIterations (max_iteration_);
    icp.setTransformationEpsilon (transform_epsilon_);
    icp.setEuclideanFitnessEpsilon (euclidean_fittness_epsilon_);
    icp.setRANSACIterations(ransac_iterations_);
    icp.setRANSACOutlierRejectionThreshold(ransac_outlier_threshold_);
    pcl::PointCloud<PointT> final;
    icp.align(final);
    pcl::transformPointCloud(final, *output_cloud, offset);
    // NODELET_INFO_STREAM("ICP converged: " << icp.hasConverged());
    // NODELET_INFO_STREAM("ICP score: " << icp.getFitnessScore());
    Eigen::Matrix4f transformation = icp.getFinalTransformation ();
    Eigen::Matrix4d transformation_d;
    jsk_recognition_utils::convertMatrix4<Eigen::Matrix4f, Eigen::Matrix4d>(
      transformation, transformation_d);
    Eigen::Affine3d offsetd;
    convertEigenAffine3(offset, offsetd);
    output_transform = offsetd * Eigen::Affine3d(transformation_d);
    return icp.getFitnessScore();
  }

  // compute ICP and return score
  double ICPRegistration::scorePointcloudAlignment(
    pcl::PointCloud<PointT>::Ptr& cloud,
    pcl::PointCloud<PointT>::Ptr& reference,
    const Eigen::Affine3f& offset,
    Eigen::Affine3f& offset_result,
    pcl::PointCloud<PointT>::Ptr transformed_cloud,
    Eigen::Affine3d& transform_result)
  {
    pcl::PointCloud<PointT>::Ptr transformed_cloud_for_debug_result
      (new pcl::PointCloud<PointT>);
    double score = alignPointcloud(cloud, reference, offset,
                                   transformed_cloud, transform_result);
    pcl::transformPointCloud(
      *transformed_cloud, *transformed_cloud_for_debug_result,
      offset.inverse());
    offset_result = offset;
    if (use_flipped_initial_pose_) {
      pcl::PointCloud<PointT>::Ptr flipped_transformed_cloud
        (new pcl::PointCloud<PointT>);
      Eigen::Affine3d flipped_transform_result;
      Eigen::Affine3f flipped_offset
      = offset * Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1));
      pcl::PointCloud<PointT>::Ptr flipped_cloud (new pcl::PointCloud<PointT>);
      pcl::transformPointCloud(
        *cloud, *flipped_cloud,
        Eigen::Affine3f(Eigen::AngleAxisf(M_PI, Eigen::Vector3f(0, 0, 1))));
      double flipped_score
        = alignPointcloud(flipped_cloud, reference, flipped_offset,
                          flipped_transformed_cloud,
                          flipped_transform_result);
      NODELET_INFO("flipped score: %f", flipped_score);
      if (flipped_score < score) {
        score = flipped_score;
        transformed_cloud = flipped_transformed_cloud;
        transform_result = flipped_transform_result;
        pcl::transformPointCloud(
          *transformed_cloud, *transformed_cloud_for_debug_result,
          flipped_offset.inverse());
        offset_result = flipped_offset;
      }
    }
    return score;
  }

  void ICPRegistration::referenceCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_init_) {
      NODELET_WARN("not yet initialized");
      return;
    }
    reference_cloud_list_.resize(0);
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::PointCloud<PointT>::Ptr non_nan_cloud (new pcl::PointCloud<PointT>);
    for (size_t i = 0; i < cloud->points.size(); i++) {
      PointT p = cloud->points[i];
      if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
        non_nan_cloud->points.push_back(p);
      }
    }
    reference_cloud_list_.push_back(non_nan_cloud);
  }

  void ICPRegistration::referenceArrayCallback(
    const jsk_recognition_msgs::PointsArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_init_) {
      NODELET_WARN("not yet initialized");
      return;
    }
    reference_cloud_list_.resize(0);
    for (size_t i = 0; i < msg->cloud_list.size(); i++) {
      pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
      pcl::fromROSMsg(msg->cloud_list[i], *cloud);
      reference_cloud_list_.push_back(cloud);
    }
  }
  
  void ICPRegistration::referenceAddCallback(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!done_init_) {
      NODELET_WARN("not yet initialized");
      return;
    }
    //reference_cloud_list_.resize(0); //not 0
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    reference_cloud_list_.push_back(cloud);
    ROS_INFO("reference_num: %zd", reference_cloud_list_.size()-1);
  }  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ICPRegistration, nodelet::Nodelet);
