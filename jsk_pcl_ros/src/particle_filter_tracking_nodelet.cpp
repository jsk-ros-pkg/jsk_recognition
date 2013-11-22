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

using namespace pcl::tracking;

namespace jsk_pcl_ros
{
  void ParticleFilterTracking::onInit(void){
    // not implemented yet
    PCLNodelet::onInit();

    int thread_nr=8;
    downsampling_grid_size_=0.02;
    new_cloud_ = false;
    counter_ = 0;
    target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());

    ParticleXYZRPY bin_size;
    bin_size.x = bin_size.y = bin_size.z = bin_size.roll = bin_size.pitch = bin_size.yaw = 0.1f;

    boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> > tracker
      (new KLDAdaptiveParticleFilterOMPTracker<pcl::PointXYZRGBA, ParticleXYZRPY> (thread_nr));

    //Set all parameters for KLDAdaptiveParticleFilterTracker<pcl::PointXYZRGBA, pcl::PointXYZ>OMPTracker
    tracker->setMaximumParticleNum (1000);
    tracker->setDelta (0.99);
    tracker->setEpsilon (0.2);
    tracker->setBinSize (bin_size);

    //Set all parameters for  ParticleFilterTracker<pcl::PointXYZRGBA, pcl::PointXYZ>
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    default_step_covariance[3] *= 40.0;
    default_step_covariance[4] *= 40.0;
    default_step_covariance[5] *= 40.0;
    tracker_ = tracker;
    tracker_->setTrans (Eigen::Affine3f::Identity ());
    tracker_->setStepNoiseCovariance (default_step_covariance);
    tracker_->setInitialNoiseCovariance (initial_noise_covariance);
    tracker_->setInitialNoiseMean (default_initial_mean);
    tracker_->setIterationNum (1);
    tracker_->setParticleNum (600);
    tracker_->setResampleLikelihoodThr(0.00);
    tracker_->setUseNormal (false);

    //Setup coherence object for tracking
    ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr coherence = ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA>::Ptr(new ApproxNearestPairPointCloudCoherence<pcl::PointXYZRGBA> ());

    boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> > distance_coherence
      = boost::shared_ptr<DistanceCoherence<pcl::PointXYZRGBA> > (new DistanceCoherence<pcl::PointXYZRGBA> ());
    coherence->addPointCoherence (distance_coherence);

    boost::shared_ptr<pcl::search::Octree<pcl::PointXYZRGBA> > search (new pcl::search::Octree<pcl::PointXYZRGBA> (0.01));
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker_->setCloudCoherence (coherence);

    //Set subscribe setting
    sub_ = pnh_->subscribe("input", 1, &ParticleFilterTracking::cloud_cb,this);
    srv_ = pnh_->advertiseService("renew_model", &ParticleFilterTracking::renewModel_cb, this);
    //Set publish setting
    particle_publiser_ = pnh_->advertise<sensor_msgs::PointCloud2>("particle", 1);
    track_result_publiser_ = pnh_->advertise<sensor_msgs::PointCloud2>("track_result", 1);
  }

  void ParticleFilterTracking::gridSampleApprox (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result, double leaf_size)
  {
    pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
    grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
    grid.setInputCloud (cloud);
    grid.filter (result);
  }

  //Publish the current particles
  void
  ParticleFilterTracking::publishParticles ()
  {
    ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY>::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles && new_cloud_)
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
        particle_publiser_.publish(particle_pointcloud2);
      }
  }

  //Publish model reference point cloud
  void
  ParticleFilterTracking::publishResult ()
  {
    ParticleXYZRPY result = tracker_->getResult ();
    Eigen::Affine3f transformation = tracker_->toEigenMatrix (result);

    //move close to camera a little for better visualization
    transformation.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr result_cloud (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::transformPointCloud<pcl::PointXYZRGBA> (*(tracker_->getReferenceCloud ()), *result_cloud, transformation);

    //Publish  model reference point cloud
    sensor_msgs::PointCloud2 result_pointcloud2;
    pcl::toROSMsg(*result_cloud, result_pointcloud2);
    result_pointcloud2.header.frame_id = frame_id_;
    track_result_publiser_.publish(result_pointcloud2);
  }

  void
  ParticleFilterTracking::resetTrackingTargetModel(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &new_target_cloud)
  {
    //prepare the model of tracker's target
    Eigen::Vector4f c;
    Eigen::Affine3f trans = Eigen::Affine3f::Identity ();
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref (new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr transed_ref_downsampled (new pcl::PointCloud<pcl::PointXYZRGBA>);

    pcl::compute3DCentroid (*new_target_cloud, c);
    trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
    pcl::transformPointCloud(*new_target_cloud, *transed_ref, trans.inverse());
    gridSampleApprox (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);
    //set reference model and trans
    {
      boost::mutex::scoped_lock lock(mtx_);
      tracker_->setReferenceCloud (transed_ref_downsampled);
      tracker_->setTrans (trans);
      tracker_->resetTracking();
    }
    //Reset target Model
    ROS_INFO("RESET TARGET MODEL");
  }

  //OpenNI Grabber's cloud Callback function
  void
  ParticleFilterTracking::cloud_cb (const sensor_msgs::PointCloud2 &pc)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    frame_id_ = pc.header.frame_id;
    std::vector<int> indices;
    pcl::fromROSMsg(pc, *cloud);
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    //when counter_ > 10, try to track.
    if(counter_ > 10){
      cloud_pass_downsampled_.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
      pcl::copyPointCloud(*cloud, *cloud_pass_downsampled_);
      if (!cloud_pass_downsampled_->points.empty()){
        boost::mutex::scoped_lock lock(mtx_);
        tracker_->setInputCloud (cloud_pass_downsampled_);
        tracker_->compute ();
        publishParticles();
        publishResult();
        ROS_INFO("Tracking.. %d", (int)cloud->points.size());
      }else{
        if(counter_%20 == 0){
          ROS_INFO("Not Tracking... Select Point/Area (Press 'h' to show Help)");
        }
      }
      new_cloud_ = true;
    }
    //when counter_ == 10, generate Target Model.
    else if (counter_ == 10){
      ROS_INFO("Target Model Segment Start");
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr segmented_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>);
      initTargetModel(cloud, segmented_cloud_);
      resetTrackingTargetModel(segmented_cloud_);
    }
    counter_++;
  }

  void ParticleFilterTracking::initTargetModel(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                                              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &segmented_cloud)
  {
      std::vector<pcl::PointIndices> cluster_indices;
      euclideanSegment (cloud, cluster_indices);

      // select the cluster to track
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
      extractSegmentCluster (cloud, cluster_indices, 0, *temp_cloud);
      Eigen::Vector4f c;
      pcl::compute3DCentroid<pcl::PointXYZRGBA> (*temp_cloud, c);
      int segment_index = 0;
      double segment_distance = c[0] * c[0] + c[1] * c[1];

      //choose most near cloud to z axis.
      for (size_t i = 1; i < cluster_indices.size (); i++)
        {
          temp_cloud.reset (new pcl::PointCloud<pcl::PointXYZRGBA>);
          extractSegmentCluster (cloud, cluster_indices, int (i), *temp_cloud);
          pcl::compute3DCentroid<pcl::PointXYZRGBA> (*temp_cloud, c);
          double distance = c[0] * c[0] + c[1] * c[1];
          if (distance < segment_distance)
            {
              segment_index = int (i);
              segment_distance = distance;
            }
        }
      extractSegmentCluster (cloud, cluster_indices, segment_index, *segmented_cloud);
  }


  //Execute  euclidean segment and retrun indices.
  void ParticleFilterTracking::euclideanSegment (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                                                 std::vector<pcl::PointIndices> &cluster_indices)
  {
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGBA> ec;
    pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBA> ());

    ec.setClusterTolerance (0.05);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
  }

  void ParticleFilterTracking::extractSegmentCluster (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                                                      const std::vector<pcl::PointIndices> cluster_indices,
                                                      const int segment_index,
                                                      pcl::PointCloud<pcl::PointXYZRGBA> &result)
  {
    pcl::PointIndices segmented_indices = cluster_indices[segment_index];
    for (size_t i = 0; i < segmented_indices.indices.size (); i++)
      {
        pcl::PointXYZRGBA point = cloud->points[segmented_indices.indices[i]];
        result.points.push_back (point);
      }
    result.width = pcl::uint32_t (result.points.size ());
    result.height = 1;
    result.is_dense = true;
  }

  bool ParticleFilterTracking::renewModel_cb(jsk_pcl_ros::SetPointCloud2::Request &req,
                                             jsk_pcl_ros::SetPointCloud2::Response &res)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_target_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::fromROSMsg(req.cloud, *new_target_cloud);
    resetTrackingTargetModel(new_target_cloud);
    return true;
  }
}

typedef jsk_pcl_ros::ParticleFilterTracking ParticleFilterTracking;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ParticleFilterTracking, ParticleFilterTracking, nodelet::Nodelet);
