// -*- mode: C++ -*-
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

#ifndef JSK_PCL_ROS_PARTICLE_FILTER_TRACKING_H_
#define JSK_PCL_ROS_PARTICLE_FILTER_TRACKING_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <jsk_pcl_ros/SetPointCloud2.h>

using namespace pcl::tracking;
namespace jsk_pcl_ros
{
  class ParticleFilterTracking: public pcl_ros::PCLNodelet
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_pass_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_pass_downsampled_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr target_cloud_;

    boost::shared_ptr<ParticleFilterTracker<pcl::PointXYZRGBA, ParticleXYZRPY> > tracker_;
    boost::mutex mtx_;
    bool new_cloud_;
    bool track_target_set_;
    std::string frame_id_;
    std::string track_target_name_;
    ros::Time stamp_;

    ros::Subscriber sub_;
    ros::Subscriber sub_update_model_;
    ros::Publisher particle_publisher_;
    ros::Publisher track_result_publisher_;
    ros::ServiceServer srv_;

    virtual void publish_particles ();
    virtual void publish_result ();

    virtual void reset_tracking_target_model(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &new_target_cloud);
    virtual void cloud_cb (const sensor_msgs::PointCloud2 &pc);
    virtual bool renew_model_cb(jsk_pcl_ros::SetPointCloud2::Request &req,
                               jsk_pcl_ros::SetPointCloud2::Response &response
                               );
    virtual void renew_model_topic_cb(const sensor_msgs::PointCloud2 &pc);

  private:
    virtual void onInit();

  };
}

#endif
