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
#include "jsk_pcl_ros/feature_registration.h"
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/fpfh_omp.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void FeatureRegistration::onInit()
  {
    DiagnosticNodelet::onInit();

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&FeatureRegistration::configCallback, this, _1, _2);
    srv_->setCallback (f);

    
    // subscribe reference always
    reference_sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_input_reference_.subscribe(*pnh_, "input/reference/cloud", 1);
    sub_input_reference_feature_.subscribe(*pnh_, "input/reference/feature", 1);
    reference_sync_->connectInput(
      sub_input_reference_, sub_input_reference_feature_);
    reference_sync_->registerCallback(boost::bind(&FeatureRegistration::referenceCallback,
                                                  this, _1, _2));
    pub_pose_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output", 1);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output/cloud", 1);
    onInitPostProcess();
  }

  void FeatureRegistration::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_input_feature_.subscribe(*pnh_, "input/feature", 1);
    sync_->connectInput(
      sub_input_, sub_input_feature_);
    sync_->registerCallback(boost::bind(&FeatureRegistration::estimate,
                                        this, _1, _2));
  }

  void FeatureRegistration::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_input_feature_.unsubscribe();
  }

  void FeatureRegistration::referenceCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& feature_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    NODELET_DEBUG("update reference");
    reference_cloud_.reset(new pcl::PointCloud<pcl::PointNormal>);
    reference_feature_.reset(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::fromROSMsg(*cloud_msg, *reference_cloud_);
    pcl::fromROSMsg(*feature_msg, *reference_feature_);
  }

  void FeatureRegistration::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    max_iterations_ = config.max_iterations;
    correspondence_randomness_ = config.correspondence_randomness;
    similarity_threshold_ = config.similarity_threshold;
    max_correspondence_distance_ = config.max_correspondence_distance;
    inlier_fraction_ = config.inlier_fraction;
    transformation_epsilon_ = config.transformation_epsilon;
  }
  
  void FeatureRegistration::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& feature_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!reference_cloud_ || !reference_feature_) {
      NODELET_ERROR("Not yet reference data is available");
      return;
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr
      cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      object_aligned (new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
      feature (new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    pcl::fromROSMsg(*feature_msg, *feature);

    pcl::SampleConsensusPrerejective<pcl::PointNormal,
                                     pcl::PointNormal,
                                     pcl::FPFHSignature33> align;
    
    align.setInputSource(reference_cloud_);
    align.setSourceFeatures(reference_feature_);

    align.setInputTarget(cloud);
    align.setTargetFeatures(feature);

    align.setMaximumIterations(max_iterations_); // Number of RANSAC iterations
    align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
    align.setCorrespondenceRandomness(correspondence_randomness_); // Number of nearest features to use
    align.setSimilarityThreshold(similarity_threshold_); // Polygonal edge length similarity threshold
    align.setMaxCorrespondenceDistance(max_correspondence_distance_); // Inlier threshold
    align.setInlierFraction(inlier_fraction_); // Required inlier fraction for accepting a pose hypothesis
    // Maximum allowable difference between two consecutive transformations
    align.setTransformationEpsilon(transformation_epsilon_);
    align.align (*object_aligned);
  
    if (align.hasConverged ())
    {
      // Print results
      printf ("\n");
      Eigen::Affine3f transformation(align.getFinalTransformation());
      geometry_msgs::PoseStamped ros_pose;
      tf::poseEigenToMsg(transformation, ros_pose.pose);
      ros_pose.header = cloud_msg->header;
      pub_pose_.publish(ros_pose);

      pcl::PointCloud<pcl::PointNormal>::Ptr
        result_cloud (new pcl::PointCloud<pcl::PointNormal>);
      pcl::transformPointCloud(
        *reference_cloud_, *result_cloud, transformation);
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*object_aligned, ros_cloud);
      ros_cloud.header = cloud_msg->header;
      pub_cloud_.publish(ros_cloud);
      
      //pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
    
    }
    else {
      NODELET_WARN("failed to align pointcloud");
    }

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FeatureRegistration, nodelet::Nodelet);
