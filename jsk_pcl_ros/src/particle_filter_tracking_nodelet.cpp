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


#include "jsk_pcl_ros/particle_filter_tracking.h"
#include <pluginlib/class_list_macros.h>
#include <jsk_topic_tools/rosparam_utils.h>

using namespace pcl::tracking;

namespace jsk_pcl_ros
{
  
  void ParticleFilterTracking::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();

    // read parameters
    int thread_nr = omp_get_num_procs();
    pnh_->getParam("thread_nr", thread_nr);
    default_step_covariance_.resize(6);
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&ParticleFilterTracking::config_callback, this, _1, _2);
    srv_->setCallback(f);

    int particle_num = 600;
    pnh_->getParam("particle_num", particle_num);
    bool use_normal = false;
    pnh_->getParam("use_normal", use_normal);
    bool use_hsv = true;
    pnh_->getParam("use_hsv", use_hsv);
    track_target_name_ = "track_result";
    pnh_->getParam("track_target_name", track_target_name_);
    std::vector<double> initial_noise_covariance = std::vector<double>(6, 0.00001);
    jsk_topic_tools::readVectorParameter(
      *pnh_, "initial_noise_covariance",
      initial_noise_covariance);
    std::vector<double> default_initial_mean = std::vector<double>(6, 0.0);
    jsk_topic_tools::readVectorParameter(
      *pnh_, "default_initial_mean", default_initial_mean);
    //First the track target is not set
    double octree_resolution = 0.01;
    pnh_->getParam("octree_resolution", octree_resolution);
    pnh_->param("align_box", align_box_, false);
    pnh_->param("BASE_FRAME_ID", base_frame_id_, std::string("NONE"));
    track_target_set_ = false;
    new_cloud_ = false;
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> > tracker
      (new KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY>(thread_nr));

    tracker->setMaximumParticleNum(max_particle_num_);
    tracker->setDelta(delta_);
    tracker->setEpsilon(epsilon_);
    tracker->setBinSize(bin_size_);

    //Set all parameters for  ParticleFilterTracker<pcl::PointXYZRGBA, pcl::PointXYZ>

    tracker_ = tracker;
    tracker_->setTrans(Eigen::Affine3f::Identity());
    tracker_->setStepNoiseCovariance(default_step_covariance_);
    tracker_->setInitialNoiseCovariance(initial_noise_covariance);
    tracker_->setInitialNoiseMean(default_initial_mean);
    tracker_->setIterationNum(iteration_num_);
    tracker_->setParticleNum(particle_num);
    tracker_->setResampleLikelihoodThr(resample_likelihood_thr_);
    tracker_->setUseNormal(use_normal);
    
    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr
      coherence(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>);

    boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> >
      distance_coherence(new DistanceCoherence<pcl::PointXYZRGBA>);
    coherence->addPointCoherence(distance_coherence);

    //add HSV coherence
    if (use_hsv) {
        boost::shared_ptr<HSVColorCoherence<pcl::PointXYZRGBA> > hsv_color_coherence
            = boost::shared_ptr<HSVColorCoherence<pcl::PointXYZRGBA> >(new HSVColorCoherence<pcl::PointXYZRGBA>());
        coherence->addPointCoherence(hsv_color_coherence);
    }
    
    boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGBA> > search(new pcl::search::Octree<pcl::PointXYZRGBA>(octree_resolution));
    coherence->setSearchMethod(search);
    coherence->setMaximumDistance(octree_resolution);

    tracker_->setCloudCoherence(coherence);

    //Set publish setting
    particle_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("particle", 1);
    track_result_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("track_result", 1);
    pose_stamped_publisher_ = pnh_->advertise<geometry_msgs::PoseStamped>("track_result_pose", 1);
    //Set subscribe setting
    sub_ = pnh_->subscribe("input", 1, &ParticleFilterTracking::cloud_cb,this);
    if (align_box_) {
      sub_input_.subscribe(*pnh_, "renew_model", 1);
      sub_box_.subscribe(*pnh_, "renew_box", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_box_);
      sync_->registerCallback(boost::bind(
                                &ParticleFilterTracking::renew_model_with_box_topic_cb,
                                this, _1, _2));
    }
    else {
      sub_update_model_ = pnh_->subscribe("renew_model", 1, &ParticleFilterTracking::renew_model_topic_cb,this);
    }
    renew_model_srv_
      = pnh_->advertiseService("renew_model", &ParticleFilterTracking::renew_model_cb, this);
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
    if (tracker_) 
    {
      NODELET_INFO("update tracker parameter");
      tracker_->setStepNoiseCovariance(default_step_covariance_);
      tracker_->setIterationNum(iteration_num_);
      tracker_->setResampleLikelihoodThr(resample_likelihood_thr_);
      tracker_->setMaximumParticleNum(max_particle_num_);
      tracker_->setDelta(delta_);
      tracker_->setEpsilon(epsilon_);
      tracker_->setBinSize(bin_size_);
    }
  }
  
  //Publish the current particles
  void ParticleFilterTracking::publish_particles()
  {
    ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY>::PointCloudStatePtr particles = tracker_->getParticles();
    if (particles && new_cloud_ && particle_publisher_.getNumSubscribers()) {
      //Set pointCloud with particle's points
      pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud(new pcl::PointCloud<pcl::PointXYZ>());
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
    ParticleXYZRPY result = tracker_->getResult();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix(result);

    //Publisher object transformation
    tf::Transform tfTransformation;
    tf::transformEigenToTF((Eigen::Affine3d) transformation, tfTransformation);

    static tf::TransformBroadcaster tfBroadcaster;
    tfBroadcaster.sendTransform(tf::StampedTransform(tfTransformation, stamp_, reference_frame_id(), track_target_name_));
    //Publish Pose
    geometry_msgs::PoseStamped result_pose_stamped;
    result_pose_stamped.header.frame_id = reference_frame_id();
    result_pose_stamped.header.stamp = stamp_;
    tf::Quaternion q; 
    tfTransformation.getBasis().getRotation(q);
    result_pose_stamped.pose.orientation.x = q.getX(); result_pose_stamped.pose.orientation.y=q.getY(); result_pose_stamped.pose.orientation.z=q.getZ(), result_pose_stamped.pose.orientation.w=q.getW();
    result_pose_stamped.pose.position.x=tfTransformation.getOrigin().getX(), result_pose_stamped.pose.position.y=tfTransformation.getOrigin().getY(), result_pose_stamped.pose.position.z=tfTransformation.getOrigin().getZ();
    pose_stamped_publisher_.publish(result_pose_stamped);
    //Publish model reference point cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud<pcl::PointXYZRGBA>(*(tracker_->getReferenceCloud()), *result_cloud, transformation);
    sensor_msgs::PointCloud2 result_pointcloud2;
    pcl::toROSMsg(*result_cloud, result_pointcloud2);
    result_pointcloud2.header.frame_id = reference_frame_id();
    result_pointcloud2.header.stamp = stamp_;
    track_result_publisher_.publish(result_pointcloud2);
  }
  std::string ParticleFilterTracking::reference_frame_id()
  {
    if (base_frame_id_.compare("NONE") == 0) return frame_id_;
    else return base_frame_id_;
  }
  
  void ParticleFilterTracking::reset_tracking_target_model(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &recieved_target_cloud)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::vector<int> indices;
    new_target_cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*recieved_target_cloud, *new_target_cloud, indices);
    
    if (base_frame_id_.compare("NONE") != 0) {
      tf::Transform transform_result = change_pointcloud_frame(new_target_cloud);
      reference_transform_ = transform_result * reference_transform_;;      
    } 

    if (!recieved_target_cloud->points.empty()) {
      //prepare the model of tracker's target
      Eigen::Vector4f c;
      Eigen::Affine3f trans = Eigen::Affine3f::Identity(); 
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref(new pcl::PointCloud<pcl::PointXYZRGBA>);        
      if (!align_box_) {
        pcl::compute3DCentroid(*new_target_cloud, c);
        trans.translation().matrix() = Eigen::Vector3f(c[0], c[1], c[2]);
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
        tracker_->setReferenceCloud(transed_ref);
        tracker_->setTrans(trans);
        tracker_->resetTracking();
      }
      track_target_set_ = true;
      ROS_INFO("RESET TARGET MODEL");
    }
    else {
      track_target_set_ = false;
      ROS_INFO("TARGET MODEL POINTS SIZE IS 0 !! Stop TRACKING");
    }
  } 
  
  tf::Transform ParticleFilterTracking::change_pointcloud_frame(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
  {
    tf::Transform tfTransformation;
    tf::StampedTransform tfTransformationStamped;
    ros::Time now = ros::Time::now();
    try {
      listener_.waitForTransform(base_frame_id_, frame_id_, now, ros::Duration(2.0));
      listener_.lookupTransform(base_frame_id_, frame_id_, now, tfTransformationStamped);
      //frame_id_ = base_frame_id_;
    }
    catch(tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      tfTransformation = tf::Transform(tf::Quaternion(0, 0, 0, 1));
    }
    tfTransformation = tf::Transform(tfTransformationStamped.getBasis(), tfTransformationStamped.getOrigin());
    Eigen::Affine3f trans; Eigen::Affine3d trans_3d;
    tf::transformTFToEigen(tfTransformation, trans_3d);
    trans = (Eigen::Affine3f) trans_3d;
    pcl::transformPointCloud(*cloud, *cloud, trans);
    return tfTransformation;
  }

  
  //OpenNI Grabber's cloud Callback function
  void ParticleFilterTracking::cloud_cb(const sensor_msgs::PointCloud2 &pc)
  {
    if (track_target_set_) {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      frame_id_ = pc.header.frame_id;
      stamp_ = pc.header.stamp;
      std::vector<int> indices;
      pcl::fromROSMsg(pc, *cloud);
      cloud->is_dense = false;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
      if (base_frame_id_.compare("NONE")!=0) {
        change_pointcloud_frame(cloud);
      }
      cloud_pass_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::copyPointCloud(*cloud, *cloud_pass_downsampled_);
      if (!cloud_pass_downsampled_->points.empty()) {
        boost::mutex::scoped_lock lock(mtx_);
        tracker_->setInputCloud(cloud_pass_downsampled_);
        tracker_->compute();
        publish_particles();
        publish_result();
      }
      new_cloud_ = true;
    }
  }

  void ParticleFilterTracking::renew_model_topic_cb(const sensor_msgs::PointCloud2 &pc)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(pc, *new_target_cloud);
    frame_id_ = pc.header.frame_id;
    reset_tracking_target_model(new_target_cloud);
  }
  void ParticleFilterTracking::renew_model_with_box_topic_cb(const sensor_msgs::PointCloud2::ConstPtr &pc_ptr, const jsk_pcl_ros::BoundingBox::ConstPtr &bb_ptr)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(*pc_ptr, *new_target_cloud);
    frame_id_ = pc_ptr->header.frame_id;
    reference_transform_ = tf::Transform(tf::Quaternion(bb_ptr->pose.orientation.x, bb_ptr->pose.orientation.y, bb_ptr->pose.orientation.z, bb_ptr->pose.orientation.w), tf::Vector3(bb_ptr->pose.position.x, bb_ptr->pose.position.y, bb_ptr->pose.position.z));
    reset_tracking_target_model(new_target_cloud);
  }
  
  bool ParticleFilterTracking::renew_model_cb(jsk_pcl_ros::SetPointCloud2::Request &req,
                                              jsk_pcl_ros::SetPointCloud2::Response &res)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(req.cloud, *new_target_cloud);
    frame_id_ = req.cloud.header.frame_id;
    reset_tracking_target_model(new_target_cloud);
    return true;
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ParticleFilterTracking, nodelet::Nodelet);
