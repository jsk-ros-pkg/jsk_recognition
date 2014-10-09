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
  
  void ParticleFilterTracking::onInit(void){
    // not implemented yet
    PCLNodelet::onInit();

    // read parameters
    int thread_nr = omp_get_num_procs();
    pnh_->getParam("thread_nr", thread_nr);
    int max_particle_num = 1000;
    pnh_->getParam("max_particle_num", max_particle_num);
    int particle_num = 600;
    pnh_->getParam("particle_num", particle_num);
    double delta = 0.99;
    pnh_->getParam("delta", delta);
    double epsilon = 0.2;
    pnh_->getParam("epsilon", epsilon);
    bool use_normal = false;
    pnh_->getParam("use_normal", use_normal);
    bool use_hsv = true;
    pnh_->getParam("use_hsv", use_hsv);
    int iteration_num = 1;
    pnh_->getParam("iteration_num", iteration_num);
    double resample_likelihood_thr = 0.0;
    pnh_->getParam("resample_likelihood_thr", resample_likelihood_thr);
    track_target_name_ = "track_result";
    pnh_->getParam("track_target_name", track_target_name_);

    std::vector<double> bin_size_vector(6);
    if (!jsk_topic_tools::readVectorParameter(
          *pnh_, "bin_size", bin_size_vector)) {
      for (size_t i = 0; i < 6; i++) {
        bin_size_vector[i] = 0.1;
      }
    }
    ParticleXYZRPY bin_size;
    bin_size.x = bin_size_vector[0];
    bin_size.y = bin_size_vector[1];
    bin_size.z = bin_size_vector[2];
    bin_size.roll = bin_size_vector[3];
    bin_size.pitch = bin_size_vector[4];
    bin_size.yaw = bin_size_vector[5];

    std::vector<double> default_step_covariance(6);
    if (!jsk_topic_tools::readVectorParameter(
          *pnh_,
          "default_step_covariance",
          default_step_covariance)) {
      for (size_t i = 0; i < default_step_covariance.size(); i++) {
        default_step_covariance[i] = 0.015 * 0.015;
      }
      default_step_covariance[3] *= 40.0;
      default_step_covariance[4] *= 40.0;
      default_step_covariance[5] *= 40.0;
    }
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    jsk_topic_tools::readVectorParameter(
      *pnh_, "initial_noise_covariance",
      initial_noise_covariance);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);
    jsk_topic_tools::readVectorParameter(
      *pnh_, "default_initial_mean", default_initial_mean);
    //First the track target is not set
    double octree_resolution = 0.01;
    pnh_->getParam("octree_resolution", octree_resolution);
    track_target_set_ = false;
    
    new_cloud_ = false;
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> > tracker
      (new KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> (thread_nr));

    //Set all parameters for KLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::PointXYZ>OMPTracker
    
    tracker->setMaximumParticleNum (max_particle_num);
    tracker->setDelta (delta);
    tracker->setEpsilon (epsilon);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilterTracker<pcl::PointXYZRGBA, pcl::PointXYZ>

    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (iteration_num);
    tracker_->setParticleNum (particle_num);
    tracker_->setResampleLikelihoodThr(resample_likelihood_thr);
    tracker_->setUseNormal (use_normal);

    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr coherence = ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA> ());

    boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> > (new DistanceCoherence<pcl::PointXYZRGBA> ());
    coherence->addPointCoherence (distance_coherence);

    //add HSV coherence
    if (use_hsv)
    {
        boost::shared_ptr<HSVColorCoherence<pcl::PointXYZRGBA> > hsv_color_coherence
            = boost::shared_ptr<HSVColorCoherence<pcl::PointXYZRGBA> > (new HSVColorCoherence<pcl::PointXYZRGBA> ());
        coherence->addPointCoherence (hsv_color_coherence);
    }
    
    boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGBA> > search (new pcl::search::Octree<pcl::PointXYZRGBA> (octree_resolution));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (octree_resolution);

    tracker_->setCloudCoherence (coherence);

    //Set subscribe setting
    sub_ = pnh_->subscribe("input", 1, &ParticleFilterTracking::cloud_cb,this);
    sub_update_model_ = pnh_->subscribe("renew_model", 1, &ParticleFilterTracking::renew_model_topic_cb,this);
    srv_ = pnh_->advertiseService("renew_model", &ParticleFilterTracking::renew_model_cb, this);
    //Set publish setting
    particle_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("particle", 1);
    track_result_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("track_result", 1);
  }

  //Publish the current particles
  void ParticleFilterTracking::publish_particles ()
  {
    ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY>::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles && new_cloud_ && particle_publisher_.getNumSubscribers())
      {
        //Set pointCloud with particle's points
        pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
        for (size_t i = 0; i < particles->points.size (); i++)
          {
            pcl::PointXYZ point;
            point.x = particles->points[i].x;
            point.y = particles->points[i].y;
            point.z = particles->points[i].z;
            particle_cloud->points.push_back (point);
          }
        //publish particle_cloud
        sensor_msgs::PointCloud2 particle_pointcloud2;
        pcl::toROSMsg(*particle_cloud, particle_pointcloud2);
        particle_pointcloud2.header.frame_id = frame_id_;
        particle_pointcloud2.header.stamp = stamp_;
        particle_publisher_.publish(particle_pointcloud2);
      }
  }

  //Publish model reference point cloud
  void ParticleFilterTracking::publish_result ()
  {
    ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

    //Publisher object transformation
    tf::Transform tfTransformation;
    tf::transformEigenToTF((Eigen::Affine3d) transformation, tfTransformation);

    static tf::TransformBroadcaster tfBroadcaster;
    tfBroadcaster.sendTransform(tf::StampedTransform(tfTransformation, stamp_, frame_id_, track_target_name_));

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::transformPointCloud<pcl::PointXYZRGBA> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

    //Publish model reference point cloud
    sensor_msgs::PointCloud2 result_pointcloud2;
    pcl::toROSMsg(*result_cloud, result_pointcloud2);
    result_pointcloud2.header.frame_id = frame_id_;
    result_pointcloud2.header.stamp = stamp_;
    track_result_publisher_.publish(result_pointcloud2);
  }

  void
  ParticleFilterTracking::reset_tracking_target_model(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &recieved_target_cloud)
  {
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
      std::vector<int> indices;
      new_target_cloud->is_dense = false;
      pcl::removeNaNFromPointCloud(*recieved_target_cloud, *new_target_cloud, indices);

    if(!recieved_target_cloud->points.empty()){
      //prepare the model of tracker's target
      Eigen::Vector4f c;
      Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);

      pcl::compute3DCentroid (*new_target_cloud, c);
      trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
      pcl::transformPointCloud(*new_target_cloud, *transed_ref, trans.inverse());
      //set reference model and trans
      {
        boost::mutex::scoped_lock lock(mtx_);
        tracker_->setReferenceCloud (transed_ref);
        tracker_->setTrans (trans);
        tracker_->resetTracking();
      }
      track_target_set_ = true;
      ROS_INFO("RESET TARGET MODEL");
    }else{
      track_target_set_ = false;
      ROS_INFO("TARGET MODEL POINTS SIZE IS 0 !! Stop TRACKING");
    }
  }

  //OpenNI Grabber's cloud Callback function
  void ParticleFilterTracking::cloud_cb (const sensor_msgs::PointCloud2 &pc)
  {
    if(track_target_set_){
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
      frame_id_ = pc.header.frame_id;
      stamp_ = pc.header.stamp;
      std::vector<int> indices;
      pcl::fromROSMsg(pc, *cloud);
      cloud->is_dense = false;
      pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

      cloud_pass_downsampled_.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::copyPointCloud(*cloud, *cloud_pass_downsampled_);
      if (!cloud_pass_downsampled_->points.empty()){
        boost::mutex::scoped_lock lock(mtx_);
        tracker_->setInputCloud (cloud_pass_downsampled_);
        tracker_->compute ();
        publish_particles();
        publish_result();
      }
      new_cloud_ = true;
    }
  }

  void ParticleFilterTracking::renew_model_topic_cb (const sensor_msgs::PointCloud2 &pc)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(pc, *new_target_cloud);
    reset_tracking_target_model(new_target_cloud);
  }

  bool ParticleFilterTracking::renew_model_cb(jsk_pcl_ros::SetPointCloud2::Request &req,
                                              jsk_pcl_ros::SetPointCloud2::Response &res)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(req.cloud, *new_target_cloud);
    reset_tracking_target_model(new_target_cloud);
    return true;
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ParticleFilterTracking, nodelet::Nodelet);
