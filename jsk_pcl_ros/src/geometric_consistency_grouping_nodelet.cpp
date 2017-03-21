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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include "jsk_pcl_ros/geometric_consistency_grouping.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/recognition/cg/geometric_consistency.h>

namespace jsk_pcl_ros
{
  void GeometricConsistencyGrouping::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&GeometricConsistencyGrouping::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_output_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output", 1);
    pub_output_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output/cloud", 1);

    reference_sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_reference_cloud_.subscribe(*pnh_, "input/reference", 1);
    sub_reference_feature_.subscribe(*pnh_, "input/reference/feature", 1);
    reference_sync_->connectInput(sub_reference_cloud_, sub_reference_feature_);
    reference_sync_->registerCallback(
      boost::bind(&GeometricConsistencyGrouping::referenceCallback,
                  this, _1, _2));
    onInitPostProcess();
  }

  void GeometricConsistencyGrouping::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_feature_.subscribe(*pnh_, "input/feature", 1);
    sync_->connectInput(sub_input_, sub_feature_);
    sync_->registerCallback(boost::bind(&GeometricConsistencyGrouping::recognize,
                                        this, _1, _2));
  }

  void GeometricConsistencyGrouping::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_feature_.unsubscribe();
  }

  void GeometricConsistencyGrouping::configCallback(
    Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    gc_size_ = config.gc_size;
    gc_thresh_ = config.gc_thresh;
  }

  void GeometricConsistencyGrouping::referenceCallback(
      const sensor_msgs::PointCloud2::ConstPtr& model_cloud_msg,
      const sensor_msgs::PointCloud2::ConstPtr& model_feature_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    reference_cloud_.reset(new pcl::PointCloud<pcl::PointNormal>);
    reference_feature_.reset(new pcl::PointCloud<pcl::SHOT352>);
    
    pcl::fromROSMsg(*model_cloud_msg, *reference_cloud_);
    pcl::fromROSMsg(*model_feature_msg, *reference_feature_);
  }

  void GeometricConsistencyGrouping::recognize(
    const sensor_msgs::PointCloud2::ConstPtr& scene_cloud_msg,
    const sensor_msgs::PointCloud2::ConstPtr& scene_feature_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!reference_cloud_ || !reference_feature_) {
      NODELET_ERROR_THROTTLE(1.0, "Not yet reference is available");
      return;
    }

    pcl::PointCloud<pcl::SHOT352>::Ptr
      scene_feature (new pcl::PointCloud<pcl::SHOT352>);
    pcl::PointCloud<pcl::PointNormal>::Ptr
      scene_cloud (new pcl::PointCloud<pcl::PointNormal>);
    pcl::fromROSMsg(*scene_cloud_msg, *scene_cloud);
    pcl::fromROSMsg(*scene_feature_msg, *scene_feature);

    pcl::CorrespondencesPtr model_scene_corrs (new pcl::Correspondences ());
    pcl::KdTreeFLANN<pcl::SHOT352> match_search;
    match_search.setInputCloud (reference_feature_);

    for (size_t i = 0; i < scene_feature->size(); ++i)
    {
      std::vector<int> neigh_indices(1);
      std::vector<float> neigh_sqr_dists(1);
      if (!pcl_isfinite (scene_feature->at(i).descriptor[0])) { //skipping NaNs
        continue;
      }
      int found_neighs
        = match_search.nearestKSearch(scene_feature->at(i), 1,
                                      neigh_indices, neigh_sqr_dists);
      //  add match only if the squared descriptor distance is less than 0.25
      // (SHOT descriptor distances are between 0 and 1 by design)
      if (found_neighs == 1 && neigh_sqr_dists[0] < 0.25f) {
        pcl::Correspondence corr (neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
        model_scene_corrs->push_back (corr);
      }
    }

    pcl::GeometricConsistencyGrouping<pcl::PointNormal, pcl::PointNormal> gc_clusterer;
    gc_clusterer.setGCSize(gc_size_);
    gc_clusterer.setGCThreshold(gc_thresh_);

    gc_clusterer.setInputCloud(reference_cloud_);
    gc_clusterer.setSceneCloud(scene_cloud);
    gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

    //gc_clusterer.cluster (clustered_corrs);
    std::vector<pcl::Correspondences> clustered_corrs;
    std::vector<Eigen::Matrix4f,
                Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
    gc_clusterer.recognize(rototranslations, clustered_corrs);
    if (rototranslations.size() > 0) {
      NODELET_INFO("detected %lu objects", rototranslations.size());
      Eigen::Matrix4f result_mat = rototranslations[0];
      Eigen::Affine3f affine(result_mat);
      geometry_msgs::PoseStamped ros_pose;
      tf::poseEigenToMsg(affine, ros_pose.pose);
      ros_pose.header = scene_cloud_msg->header;
      pub_output_.publish(ros_pose);
    }
    else {
      NODELET_WARN("Failed to find object");
    }
    
  }

}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::GeometricConsistencyGrouping, nodelet::Nodelet);
