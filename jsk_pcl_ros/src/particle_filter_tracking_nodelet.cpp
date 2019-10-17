/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
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

#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros/particle_filter_tracking.h"
#include <pcl/tracking/impl/distance_coherence.hpp>
#include <pcl/tracking/impl/approx_nearest_pair_point_cloud_coherence.hpp>
#include <pluginlib/class_list_macros.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <pcl/kdtree/kdtree_flann.h>

using namespace pcl::tracking;

namespace jsk_pcl_ros
{
  
  void ParticleFilterTracking::onInit(void)
  {
    // Suppress huge amount of error message.
    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_ALWAYS);
    // not implemented yet
    ConnectionBasedNodelet::onInit();
    
    track_target_set_ = false;
    new_cloud_ = false;
    
    default_step_covariance_.resize(6);
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ParticleFilterTracking::config_callback, this, _1, _2);
    srv_->setCallback(f);

    int particle_num;
    pnh_->param("particle_num", particle_num, max_particle_num_ - 1);
    bool use_normal;
    pnh_->param("use_normal", use_normal, false);
    bool use_hsv;
    pnh_->param("use_hsv", use_hsv, true);
    pnh_->param("track_target_name", track_target_name_,
                std::string("track_result"));
    std::vector<double> initial_noise_covariance(6, 0.00001);
    jsk_topic_tools::readVectorParameter(
      *pnh_, "initial_noise_covariance",
      initial_noise_covariance);
    std::vector<double> default_initial_mean(6, 0.0);
    jsk_topic_tools::readVectorParameter(
      *pnh_, "default_initial_mean", default_initial_mean);
    
    //First the track target is not set
    double octree_resolution = 0.01;
    pnh_->getParam("octree_resolution", octree_resolution);
    pnh_->param("align_box", align_box_, false);
    pnh_->param("BASE_FRAME_ID", base_frame_id_, std::string("NONE"));
    if (base_frame_id_.compare("NONE") != 0) {
      listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    }
    target_cloud_.reset(new pcl::PointCloud<PointT>());
    pnh_->param("not_use_reference_centroid", not_use_reference_centroid_,
                false);
    pnh_->param("not_publish_tf", not_publish_tf_, false);
    pnh_->param("reversed", reversed_, false);
    
    int thread_nr;
    pnh_->param("thread_nr", thread_nr, omp_get_num_procs());

    if (!reversed_) {
      boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleXYZRPY> > tracker
        (new KLDAdaptiveParticleFilterOMPTracker<PointT, ParticleXYZRPY>(thread_nr));
      tracker->setMaximumParticleNum(max_particle_num_);
      tracker->setDelta(delta_);
      tracker->setEpsilon(epsilon_);
      tracker->setBinSize(bin_size_);
      tracker_ = tracker;
    }
    else {
      boost::shared_ptr<ReversedParticleFilterOMPTracker<PointT, ParticleXYZRPY> > tracker
        (new ReversedParticleFilterOMPTracker<PointT, ParticleXYZRPY>(thread_nr));
      // boost::shared_ptr<ReversedParticleFilterTracker<PointT, ParticleXYZRPY> > tracker
      //   (new ReversedParticleFilterTracker<PointT, ParticleXYZRPY>());
      reversed_tracker_ = tracker;
    }
    tracker_set_trans(Eigen::Affine3f::Identity());
    tracker_set_step_noise_covariance(default_step_covariance_);
    tracker_set_initial_noise_covariance(initial_noise_covariance);
    tracker_set_initial_noise_mean(default_initial_mean);
    tracker_set_iteration_num(iteration_num_);
    tracker_set_particle_num(particle_num);
    tracker_set_resample_likelihood_thr(resample_likelihood_thr_);
    tracker_set_use_normal(use_normal);
    
    //Setup coherence object for tracking
    bool enable_cache;
    bool enable_organized;
    pnh_->param("enable_cache", enable_cache, false);
    pnh_->param("enable_organized", enable_organized, false);
    ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence;
    if (enable_cache) {
      double cache_bin_size_x, cache_bin_size_y, cache_bin_size_z;
      pnh_->param("cache_size_x", cache_bin_size_x, 0.01);
      pnh_->param("cache_size_y", cache_bin_size_y, 0.01);
      pnh_->param("cache_size_z", cache_bin_size_z, 0.01);
      coherence.reset(new CachedApproxNearestPairPointCloudCoherence<PointT>(
                        cache_bin_size_x, cache_bin_size_y, cache_bin_size_z));
    }else if(enable_organized){
      coherence.reset(new OrganizedNearestPairPointCloudCoherence<PointT>());
    }
    else {
      coherence.reset(new ApproxNearestPairPointCloudCoherence<PointT>());
    }
    

    boost::shared_ptr<DistanceCoherence<PointT> >
      distance_coherence(new DistanceCoherence<PointT>);
    coherence->addPointCoherence(distance_coherence);

    //add HSV coherence
    if (use_hsv) {
        boost::shared_ptr<HSVColorCoherence<PointT> > hsv_color_coherence
          = boost::shared_ptr<HSVColorCoherence<PointT> >(new HSVColorCoherence<PointT>());
        coherence->addPointCoherence(hsv_color_coherence);
    }
    
     boost::shared_ptr<pcl::search::Octree<PointT> > search
       (new pcl::search::Octree<PointT>(octree_resolution));
    //boost::shared_ptr<pcl::search::KdTree<PointT> > search(new pcl::search::KdTree<PointT>());
    coherence->setSearchMethod(search);
    double max_distance;
    pnh_->param("max_distance", max_distance, 0.01);
    coherence->setMaximumDistance(max_distance);
    pnh_->param("use_change_detection", use_change_detection_, false);
    tracker_set_cloud_coherence(coherence);

    //Set publish setting
    particle_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "particle", 1);
    track_result_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>(
      "track_result", 1);
    pose_stamped_publisher_ = pnh_->advertise<geometry_msgs::PoseStamped>(
      "track_result_pose", 1);
    pub_latest_time_ = pnh_->advertise<std_msgs::Float32>(
      "output/latest_time", 1);
    pub_average_time_ = pnh_->advertise<std_msgs::Float32>(
      "output/average_time", 1);
    pub_rms_angle_ = pnh_->advertise<std_msgs::Float32>(
      "output/rms_angle_error", 1);
    pub_rms_distance_ = pnh_->advertise<std_msgs::Float32>(
      "output/rms_distance_error", 1);
    pub_velocity_ = pnh_->advertise<geometry_msgs::TwistStamped>(
      "output/velocity", 1);
    pub_velocity_norm_ = pnh_->advertise<std_msgs::Float32>(
      "output/velocity_norm", 1);
    pub_no_move_raw_ = pnh_->advertise<std_msgs::Bool>(
      "output/no_move_raw", 1);
    pub_no_move_ = pnh_->advertise<std_msgs::Bool>(
      "output/no_move", 1);
    pub_skipped_ = pnh_->advertise<std_msgs::Bool>(
      "output/skipped", 1);
    //Set subscribe setting
    if (use_change_detection_) {
      pub_change_cloud_marker_ = pnh_->advertise<visualization_msgs::MarkerArray>(
        "output/change_marker", 1);
      pub_tracker_status_ = pnh_->advertise<jsk_recognition_msgs::TrackerStatus>(
        "output/tracker_status", 1);
      sub_input_cloud_.subscribe(*pnh_, "input", 4);
      sub_change_cloud_.subscribe(*pnh_, "input_change", 4);
      change_sync_ = boost::make_shared<message_filters::Synchronizer<SyncChangePolicy> >(100);
      change_sync_->connectInput(sub_input_cloud_, sub_change_cloud_);
      change_sync_->registerCallback(
        boost::bind(
          &ParticleFilterTracking::cloud_change_cb,
          this, _1, _2));
    }
    else {
      sub_ = pnh_->subscribe("input", 1, &ParticleFilterTracking::cloud_cb,this);
    }
    if (align_box_) {
      sub_input_.subscribe(*pnh_, "renew_model", 1);
      sub_box_.subscribe(*pnh_, "renew_box", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_box_);
      sync_->registerCallback(
        boost::bind(
          &ParticleFilterTracking::renew_model_with_box_topic_cb,
          this, _1, _2));
    }
    else {
      sub_update_model_ = pnh_->subscribe(
        "renew_model", 1, &ParticleFilterTracking::renew_model_topic_cb,this);
      sub_update_with_marker_model_
        = pnh_->subscribe("renew_model_with_marker", 1, &ParticleFilterTracking::renew_model_with_marker_topic_cb, this);
    }

    pnh_->param("marker_to_pointcloud_sampling_nums", marker_to_pointcloud_sampling_nums_, 10000);
    renew_model_srv_
      = pnh_->advertiseService(
        "renew_model", &ParticleFilterTracking::renew_model_cb, this);

    onInitPostProcess();
  }

  void ParticleFilterTracking::config_callback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mtx_);
    max_particle_num_ = config.max_particle_num;
    iteration_num_ = config.iteration_num;
    resample_likelihood_thr_ = config.resample_likelihood_thr;
    delta_ = config.delta;
    epsilon_ = config.epsilon;
    bin_size_.x = config.bin_size_x;
    bin_size_.y = config.bin_size_y;
    bin_size_.z = config.bin_size_z;
    bin_size_.roll = config.bin_size_roll;
    bin_size_.pitch = config.bin_size_pitch;
    bin_size_.yaw = config.bin_size_yaw;
    default_step_covariance_[0] = config.default_step_covariance_x;
    default_step_covariance_[1] = config.default_step_covariance_y;
    default_step_covariance_[2] = config.default_step_covariance_z;
    default_step_covariance_[3] = config.default_step_covariance_roll;
    default_step_covariance_[4] = config.default_step_covariance_pitch;
    default_step_covariance_[5] = config.default_step_covariance_yaw;
    static_velocity_thr_ = config.static_velocity_thr;
    change_cloud_near_threshold_ = config.change_cloud_near_thr;
    if (tracker_ || reversed_tracker_) 
    {
      NODELET_INFO("update tracker parameter");
      tracker_set_step_noise_covariance(default_step_covariance_);
      tracker_set_iteration_num(iteration_num_);
      tracker_set_resample_likelihood_thr(resample_likelihood_thr_);
      tracker_set_maximum_particle_num(max_particle_num_);
      tracker_set_delta(delta_);
      tracker_set_epsilon(epsilon_);
      tracker_set_bin_size(bin_size_);
    }
  }
  
  //Publish the current particles
  void ParticleFilterTracking::publish_particles()
  {
    PointCloudStatePtr particles = tracker_get_particles();
    if (particles && new_cloud_ && particle_publisher_.getNumSubscribers()) {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud
        (new pcl::PointCloud<pcl::PointXYZ>());
      for (size_t i = 0; i < particles->points.size(); i++)
      {
        pcl::PointXYZ point;
        point.x = particles->points[i].x;
        point.y = particles->points[i].y;
        point.z = particles->points[i].z;
        particle_cloud->points.push_back(point);
      }
      //publish particle_cloud
      sensor_msgs::PointCloud2 particle_pointcloud2;
      pcl::toROSMsg(*particle_cloud, particle_pointcloud2);
      particle_pointcloud2.header.frame_id = reference_frame_id();
      particle_pointcloud2.header.stamp = stamp_;
      particle_publisher_.publish(particle_pointcloud2);
    }
  }
  //Publish model reference point cloud
  void ParticleFilterTracking::publish_result()
  {
    ParticleXYZRPY result = tracker_get_result();
    Eigen::Affine3f transformation = tracker_to_eigen_matrix(result);

    //Publisher object transformation
    tf::Transform tfTransformation;
    tf::transformEigenToTF((Eigen::Affine3d) transformation, tfTransformation);

    if (!not_publish_tf_) {
      static tf::TransformBroadcaster tfBroadcaster;
      tfBroadcaster.sendTransform(tf::StampedTransform(
                                    tfTransformation, stamp_,
                                    reference_frame_id(), track_target_name_));
    }
    //Publish Pose
    geometry_msgs::PoseStamped result_pose_stamped;
    result_pose_stamped.header.frame_id = reference_frame_id();
    result_pose_stamped.header.stamp = stamp_;
    tf::Quaternion q;
    tf::poseTFToMsg(tfTransformation, result_pose_stamped.pose);
    pose_stamped_publisher_.publish(result_pose_stamped);
    //Publish model reference point cloud
    pcl::PointCloud<PointT>::Ptr result_cloud
      (new pcl::PointCloud<PointT>());
    pcl::transformPointCloud<PointT>(
      *(tracker_get_reference_cloud()), *result_cloud, transformation);
    sensor_msgs::PointCloud2 result_pointcloud2;
    pcl::toROSMsg(*result_cloud, result_pointcloud2);
    result_pointcloud2.header.frame_id = reference_frame_id();
    result_pointcloud2.header.stamp = stamp_;
    track_result_publisher_.publish(result_pointcloud2);

    if (counter_ > 0) {         // publish velocity
      geometry_msgs::TwistStamped twist;
      twist.header.frame_id = reference_frame_id();
      twist.header.stamp = stamp_;
      double dt = (stamp_ - prev_stamp_).toSec();
      twist.twist.linear.x = (result.x - prev_result_.x) / dt;
      twist.twist.linear.y = (result.y - prev_result_.y) / dt;
      twist.twist.linear.z = (result.z - prev_result_.z) / dt;
      twist.twist.angular.x = (result.roll - prev_result_.roll) / dt;
      twist.twist.angular.y = (result.pitch - prev_result_.pitch) / dt;
      twist.twist.angular.z = (result.yaw - prev_result_.yaw) / dt;
      pub_velocity_.publish(twist);
      Eigen::Vector3f vel(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
      std_msgs::Float32 velocity_norm;
      velocity_norm.data = vel.norm();
      pub_velocity_norm_.publish(velocity_norm);
      bool is_static = vel.norm() < static_velocity_thr_;
      no_move_buffer_.addValue(is_static);
      std_msgs::Bool no_move_raw, no_move;
      no_move_raw.data = is_static;
      no_move.data = no_move_buffer_.isAllTrueFilled();
      pub_no_move_.publish(no_move);
      pub_no_move_raw_.publish(no_move_raw);
    }
    
    Eigen::Affine3f diff_trans = transformation.inverse() * initial_pose_;
    double distance_error = Eigen::Vector3f(diff_trans.translation()).norm();
    double angle_error = Eigen::AngleAxisf(diff_trans.rotation()).angle();
    distance_error_buffer_.push_back(distance_error);
    angle_error_buffer_.push_back(angle_error);
    double distance_rms = rms(distance_error_buffer_);
    double angle_rms = rms(angle_error_buffer_);
    std_msgs::Float32 ros_distance_rms, ros_angle_rms;
    ros_distance_rms.data = distance_rms;
    ros_angle_rms.data = angle_rms;
    pub_rms_distance_.publish(ros_distance_rms);
    pub_rms_angle_.publish(ros_angle_rms);
    prev_result_ = result;
    prev_stamp_ = stamp_;
    ++counter_;
  }
  
  std::string ParticleFilterTracking::reference_frame_id()
  {
    if (base_frame_id_.compare("NONE") == 0) {
      return frame_id_;
    }
    else {
      return base_frame_id_;
    }
  }
  
  void ParticleFilterTracking::reset_tracking_target_model(
    const pcl::PointCloud<PointT>::ConstPtr &recieved_target_cloud)
  {
    pcl::PointCloud<PointT>::Ptr new_target_cloud(new pcl::PointCloud<PointT>);
    std::vector<int> indices;
    new_target_cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(
      *recieved_target_cloud, *new_target_cloud, indices);
    
    if (base_frame_id_.compare("NONE") != 0) {
      tf::Transform transform_result
        = change_pointcloud_frame(new_target_cloud);
      reference_transform_ = transform_result * reference_transform_;;
    }

    if (!recieved_target_cloud->points.empty()) {
      //prepare the model of tracker's target
      Eigen::Affine3f trans = Eigen::Affine3f::Identity(); 
      pcl::PointCloud<PointT>::Ptr transed_ref(new pcl::PointCloud<PointT>);
      if (!align_box_) {
        if (!not_use_reference_centroid_) {
          Eigen::Vector4f c;
          pcl::compute3DCentroid(*new_target_cloud, c);
          trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
        }
      }
      else {
        Eigen::Affine3d trans_3d = Eigen::Affine3d::Identity();
        tf::transformTFToEigen(reference_transform_, trans_3d);
        trans = (Eigen::Affine3f) trans_3d;
      }
      pcl::transformPointCloud(*new_target_cloud, *transed_ref, trans.inverse());
      //set reference model and trans
      {
        boost::mutex::scoped_lock lock(mtx_);
        tracker_set_reference_cloud(transed_ref);
        tracker_set_trans(trans);
        tracker_reset_tracking();
        initial_pose_ = Eigen::Affine3f(trans);
      }
      track_target_set_ = true;
      NODELET_INFO("RESET TARGET MODEL");
    }
    else {
      track_target_set_ = false;
      NODELET_ERROR("TARGET MODEL POINTS SIZE IS 0 !! Stop TRACKING");
    }
  } 
  
  tf::Transform ParticleFilterTracking::change_pointcloud_frame(
    pcl::PointCloud<PointT>::Ptr cloud)
  {
    tf::Transform tfTransformation;
    tf::StampedTransform tfTransformationStamped;
    ros::Time now = ros::Time::now();
    try {
      listener_->waitForTransform(base_frame_id_, frame_id_, now,
                                  ros::Duration(2.0));
      listener_->lookupTransform(base_frame_id_, frame_id_, now,
                                 tfTransformationStamped);
      //frame_id_ = base_frame_id_;
    }
    catch(tf::TransformException ex) {
      NODELET_ERROR("%s",ex.what());
      tfTransformation = tf::Transform(tf::Quaternion(0, 0, 0, 1));
    }
    tfTransformation = tf::Transform(tfTransformationStamped.getBasis(),
                                     tfTransformationStamped.getOrigin());
    Eigen::Affine3f trans; Eigen::Affine3d trans_3d;
    tf::transformTFToEigen(tfTransformation, trans_3d);
    trans = (Eigen::Affine3f) trans_3d;
    pcl::transformPointCloud(*cloud, *cloud, trans);
    return tfTransformation;
  }

  void ParticleFilterTracking::publish_tracker_status(const std_msgs::Header& header,
                                                      const bool is_tracking)
  {
    jsk_recognition_msgs::TrackerStatus tracker_status;
    tracker_status.header = header;
    tracker_status.is_tracking = is_tracking;
    pub_tracker_status_.publish(tracker_status);
  }
  
  void ParticleFilterTracking::cloud_change_cb(const sensor_msgs::PointCloud2::ConstPtr &pc_msg,
                                               const sensor_msgs::PointCloud2::ConstPtr &change_cloud_msg)
  {
    if (no_move_buffer_.isAllTrueFilled()) {
      jsk_recognition_utils::ScopedWallDurationReporter r
        = timer_.reporter(pub_latest_time_, pub_average_time_);
      // change change_cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr change_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*change_cloud_msg, *change_cloud);
      if (change_cloud->points.size() == 0) {
        stamp_ = pc_msg->header.stamp;
        publish_result();
        publish_tracker_status(pc_msg->header, false);
        return;
      }
      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      kdtree.setInputCloud(change_cloud);
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      pcl::PointXYZRGB p;
      p.x = prev_result_.x;
      p.y = prev_result_.y;
      p.z = prev_result_.z;
      if (kdtree.radiusSearch(p, change_cloud_near_threshold_, k_indices, k_sqr_distances, 1) > 0) {
        NODELET_INFO("change detection triggered!");
        // there is near pointcloud
        cloud_cb(*pc_msg);
        r.setIsEnabled(false);
        no_move_buffer_.clear();
        publish_tracker_status(pc_msg->header, true);
      }
      else {
        // publish previous result
        stamp_ = pc_msg->header.stamp;
        publish_result();
        publish_tracker_status(pc_msg->header, false);
      }
    }
    else {
      publish_tracker_status(pc_msg->header, true);
      cloud_cb(*pc_msg);
    }
  }
  
  //OpenNI Grabber's cloud Callback function
  void ParticleFilterTracking::cloud_cb(const sensor_msgs::PointCloud2 &pc)
  {
    if (track_target_set_) {
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
      frame_id_ = pc.header.frame_id;
      stamp_ = pc.header.stamp;
      std::vector<int> indices;
      pcl::fromROSMsg(pc, *cloud);
      cloud->is_dense = false;
      {
        jsk_recognition_utils::ScopedWallDurationReporter r
          = timer_.reporter(pub_latest_time_, pub_average_time_);
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
        if (base_frame_id_.compare("NONE")!=0) {
          change_pointcloud_frame(cloud);
        }
        cloud_pass_downsampled_.reset(new pcl::PointCloud<PointT>);
        pcl::copyPointCloud(*cloud, *cloud_pass_downsampled_);
        if (!cloud_pass_downsampled_->points.empty()) {
          boost::mutex::scoped_lock lock(mtx_);
          tracker_set_input_cloud(cloud_pass_downsampled_);
          tracker_compute();
          publish_particles();
          publish_result();
        }
        new_cloud_ = true;
      }
    }
  }

  void ParticleFilterTracking::renew_model_topic_cb(
    const sensor_msgs::PointCloud2 &pc)
  {
    pcl::PointCloud<PointT>::Ptr new_target_cloud
      (new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(pc, *new_target_cloud);
    frame_id_ = pc.header.frame_id;
    reset_tracking_target_model(new_target_cloud);
  }

  void ParticleFilterTracking::renew_model_with_marker_topic_cb(const visualization_msgs::Marker &marker)
  {
    if(marker.type == visualization_msgs::Marker::TRIANGLE_LIST && !marker.points.empty()){
      ROS_INFO("Reset Tracker Model with renew_model_with_marker_topic_cb");
      pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
      jsk_recognition_utils::markerMsgToPointCloud(marker,
                            marker_to_pointcloud_sampling_nums_,
                            *cloud
                            );

      Eigen::Affine3f trans;
      tf::poseMsgToEigen(marker.pose, trans);
      pcl::transformPointCloud(*cloud, *cloud, trans);

      frame_id_ = marker.header.frame_id;
      reset_tracking_target_model(cloud);
    }else{
      ROS_ERROR(" Marker Models type is not TRIANGLE ");
      ROS_ERROR("   OR   ");
      ROS_ERROR(" Marker Points is empty ");
    }
  }

  void ParticleFilterTracking::renew_model_with_box_topic_cb(
    const sensor_msgs::PointCloud2::ConstPtr &pc_ptr,
    const jsk_recognition_msgs::BoundingBox::ConstPtr &bb_ptr)
  {
    pcl::PointCloud<PointT>::Ptr new_target_cloud
      (new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(*pc_ptr, *new_target_cloud);
    frame_id_ = pc_ptr->header.frame_id;
    tf::poseMsgToTF(bb_ptr->pose, reference_transform_);
    reset_tracking_target_model(new_target_cloud);
  }
  
  bool ParticleFilterTracking::renew_model_cb(
    jsk_recognition_msgs::SetPointCloud2::Request &req,
    jsk_recognition_msgs::SetPointCloud2::Response &res)
  {
    pcl::PointCloud<PointT>::Ptr new_target_cloud(new pcl::PointCloud<PointT>());
    pcl::fromROSMsg(req.cloud, *new_target_cloud);
    frame_id_ = req.cloud.header.frame_id;
    reset_tracking_target_model(new_target_cloud);
    return true;
  }

  void ParticleFilterTracking::tracker_set_trans(
    const Eigen::Affine3f& trans)
  {
    Eigen::Vector3f pos = trans.translation();
    NODELET_INFO("trans: [%f, %f, %f]", pos[0], pos[1], pos[2]);
    if (reversed_) {
      reversed_tracker_->setTrans(trans);
    }
    else {
      tracker_->setTrans(trans);
    }
  }

  void ParticleFilterTracking::tracker_set_step_noise_covariance(
    const std::vector<double>& covariance)
  {
    if (reversed_) {
      reversed_tracker_->setStepNoiseCovariance(covariance);
    }
    else {
      tracker_->setStepNoiseCovariance(covariance);
    }
  }

  void ParticleFilterTracking::tracker_set_initial_noise_covariance(
    const std::vector<double>& covariance)
  {
    if (reversed_) {
      reversed_tracker_->setInitialNoiseCovariance(covariance);
    }
    else {
      tracker_->setInitialNoiseCovariance(covariance);
    }
  }

  void ParticleFilterTracking::tracker_set_initial_noise_mean(
    const std::vector<double>& mean)
  {
    if (reversed_) {
      reversed_tracker_->setInitialNoiseMean(mean);
    }
    else {
      tracker_->setInitialNoiseMean(mean);
    }
  }

  void ParticleFilterTracking::tracker_set_iteration_num(const int num)
  {
    if (reversed_) {
      reversed_tracker_->setIterationNum(num);
    }
    else {
      tracker_->setIterationNum(num);
    }
  }

  void ParticleFilterTracking::tracker_set_particle_num(const int num)
  {
    if (reversed_) {
      reversed_tracker_->setParticleNum(num);
    }
    else {
      tracker_->setParticleNum(num);
    }
  }

  void ParticleFilterTracking::tracker_set_resample_likelihood_thr(double thr)
  {
    if (reversed_) {
      reversed_tracker_->setResampleLikelihoodThr(thr);
    }
    else {
      tracker_->setResampleLikelihoodThr(thr);
    }
  }

  void ParticleFilterTracking::tracker_set_use_normal(bool use_normal)
  {
    if (reversed_) {
      reversed_tracker_->setUseNormal(use_normal);
    }
    else {
      tracker_->setUseNormal(use_normal);
    }
  }
  
  void ParticleFilterTracking::tracker_set_cloud_coherence(
    ApproxNearestPairPointCloudCoherence<PointT>::Ptr coherence)
  {
    if (reversed_) {
      reversed_tracker_->setCloudCoherence(coherence);
    }
    else {
      tracker_->setCloudCoherence(coherence);
    }
  }

  void ParticleFilterTracking::tracker_set_maximum_particle_num(int num)
  {
    if (!reversed_) {
      tracker_->setMaximumParticleNum(num);
    }
  }
  
  void ParticleFilterTracking::tracker_set_delta(double delta)
  {
    if (!reversed_) {
      tracker_->setDelta(delta);
    }
  }
  
  void ParticleFilterTracking::tracker_set_epsilon(double epsilon)
  {
    if (!reversed_) {
      tracker_->setEpsilon(epsilon);
    }
  }
  
  void ParticleFilterTracking::tracker_set_bin_size(
    const ParticleXYZRPY bin_size)
  {
    if (!reversed_) {
      tracker_->setBinSize(bin_size);
    }
  }

  ParticleFilterTracking::PointCloudStatePtr
  ParticleFilterTracking::tracker_get_particles()
  {
    if (!reversed_) {
      return tracker_->getParticles();
    }
    else {
      return reversed_tracker_->getParticles();
    }
  }

  ParticleXYZRPY ParticleFilterTracking::tracker_get_result()
  {
    if (!reversed_) {
      return tracker_->getResult();
    }
    else {
      return reversed_tracker_->getResult();
    }
  }

  Eigen::Affine3f ParticleFilterTracking::tracker_to_eigen_matrix(
    const ParticleXYZRPY& result)
  {
    if (!reversed_) {
      return tracker_->toEigenMatrix(result);
    }
    else {
      return reversed_tracker_->toEigenMatrix(result);
    }
  }

  pcl::PointCloud<ParticleFilterTracking::PointT>::ConstPtr
  ParticleFilterTracking::tracker_get_reference_cloud()
  {
    if (!reversed_) {
      return tracker_->getReferenceCloud();
    }
    else {
      return reversed_tracker_->getReferenceCloud();
    }
  }

  void ParticleFilterTracking::tracker_set_reference_cloud(
    pcl::PointCloud<PointT>::Ptr ref)
  {
    if (!reversed_) {
      tracker_->setReferenceCloud(ref);
    }
    else {
      reversed_tracker_->setReferenceCloud(ref);
    }
    counter_ = 0;
    no_move_buffer_.clear();
  }

  void ParticleFilterTracking::tracker_reset_tracking()
  {
    if (!reversed_) {
      tracker_->resetTracking();
    }
    else {
      reversed_tracker_->resetTracking();
    }
  }

  void ParticleFilterTracking::tracker_set_input_cloud(
    pcl::PointCloud<PointT>::Ptr input)
  {
    if (!reversed_) {
      tracker_->setInputCloud(input);
    }
    else {
      reversed_tracker_->setInputCloud(input);
    }
  }
  void ParticleFilterTracking::tracker_compute()
  {
    if (!reversed_) {
      tracker_->compute();
    }
    else {
      reversed_tracker_->compute();
    }
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ParticleFilterTracking, nodelet::Nodelet);
