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
#include "jsk_pcl_ros/pointcloud_localization.h"
#include <jsk_recognition_msgs/ICPAlign.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

namespace jsk_pcl_ros
{
  void PointCloudLocalization::onInit()
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    DiagnosticNodelet::onInit();
    tf_listener_ = TfListenerSingleton::getInstance();
    // initialize localize_transform_ as identity
    localize_transform_.setIdentity();
    pnh_->param("global_frame", global_frame_, std::string("map"));
    pnh_->param("odom_frame", odom_frame_, std::string("odom"));
    pnh_->param("leaf_size", leaf_size_, 0.01);
    pnh_->param("initialize_from_tf", initialize_from_tf_, false);
    if (initialize_from_tf_) {
      pnh_->param("initialize_tf", initialize_tf_, std::string("odom_on_ground"));
    }
    pnh_->param("clip_unseen_pointcloud", clip_unseen_pointcloud_, false);
    if (clip_unseen_pointcloud_) {
      pnh_->param("sensor_frame", sensor_frame_, std::string("BODY"));
    }
    pnh_->param("use_normal", use_normal_, false);
    double cloud_rate;
    pnh_->param("cloud_rate", cloud_rate, 10.0);
    double tf_rate;
    pnh_->param("tf_rate", tf_rate, 20.0);
    pub_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    // always subscribe pointcloud...?
    sub_ = pnh_->subscribe("input", 1, &PointCloudLocalization::cloudCallback, this);
    localization_srv_ = pnh_->advertiseService(
      "localize", &PointCloudLocalization::localizationRequest, this);
    update_offset_srv_ = pnh_->advertiseService(
      "update_offset", &PointCloudLocalization::updateOffsetCallback, this);

    // timer to publish cloud
    cloud_timer_ = pnh_->createTimer(
      ros::Duration(1.0 / cloud_rate),
      boost::bind(&PointCloudLocalization::cloudTimerCallback, this, _1));
    // timer to publish tf
    tf_timer_ = pnh_->createTimer(
      ros::Duration(1.0 / tf_rate),
      boost::bind(&PointCloudLocalization::tfTimerCallback, this, _1));

    onInitPostProcess();
  }

  void PointCloudLocalization::subscribe()
  {
    // dummy
  }
  
  void PointCloudLocalization::unsubscribe()
  {
    // dummy
  }

  void PointCloudLocalization::applyDownsampling(
    pcl::PointCloud<pcl::PointNormal>::Ptr in_cloud,
    pcl::PointCloud<pcl::PointNormal>& out_cloud)
  {
    pcl::VoxelGrid<pcl::PointNormal> vg;
    vg.setInputCloud(in_cloud);
    vg.setLeafSize(leaf_size_, leaf_size_, leaf_size_);
    vg.filter(out_cloud);
  }
  
  void PointCloudLocalization::cloudTimerCallback(
    const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(mutex_);
    ros::Time stamp = event.current_real;
    if (all_cloud_) {
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*all_cloud_, ros_cloud);
      ros_cloud.header.stamp = stamp;
      ros_cloud.header.frame_id = global_frame_;
      pub_cloud_.publish(ros_cloud);
    }
  }

  void PointCloudLocalization::tfTimerCallback(
    const ros::TimerEvent& event)
  {
    boost::mutex::scoped_lock lock(tf_mutex_);
    try {
    ros::Time stamp = event.current_real;
    if (initialize_from_tf_ && first_time_) {
      // Update localize_transform_ to points initialize_tf
      tf::StampedTransform transform = lookupTransformWithDuration(
        tf_listener_,
        initialize_tf_, odom_frame_, stamp, ros::Duration(1.0));
      localize_transform_ = transform;

    }
    tf_broadcast_.sendTransform(tf::StampedTransform(localize_transform_,
                                                     stamp,
                                                     global_frame_,
                                                     odom_frame_));
    }
    catch (tf2::TransformException& e) {
      NODELET_FATAL("Failed to lookup transformation: %s", e.what());
    }
  }

  void PointCloudLocalization::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    //NODELET_INFO("cloudCallback");
    latest_cloud_ = cloud_msg;
    if (localize_requested_){
      NODELET_INFO("localization is requested");
      try {
        pcl::PointCloud<pcl::PointNormal>::Ptr
          local_cloud (new pcl::PointCloud<pcl::PointNormal>);
        pcl::fromROSMsg(*latest_cloud_, *local_cloud);
        NODELET_INFO("waiting for tf transformation from %s tp %s",
                     latest_cloud_->header.frame_id.c_str(),
                     global_frame_.c_str());
        if (tf_listener_->waitForTransform(
              latest_cloud_->header.frame_id,
              global_frame_,
              latest_cloud_->header.stamp,
              ros::Duration(1.0))) {
          pcl::PointCloud<pcl::PointNormal>::Ptr
            input_cloud (new pcl::PointCloud<pcl::PointNormal>);
          if (use_normal_) {
            pcl_ros::transformPointCloudWithNormals(global_frame_,
                                                    *local_cloud,
                                                    *input_cloud,
                                                    *tf_listener_);
          }
          else {
            pcl_ros::transformPointCloud(global_frame_,
                                         *local_cloud,
                                         *input_cloud,
                                         *tf_listener_);
          }
          pcl::PointCloud<pcl::PointNormal>::Ptr
            input_downsampled_cloud (new pcl::PointCloud<pcl::PointNormal>);
          applyDownsampling(input_cloud, *input_downsampled_cloud);
          if (isFirstTime()) {
            all_cloud_ = input_downsampled_cloud;
            first_time_ = false;
          }
          else {
            // run ICP
            ros::ServiceClient client
              = pnh_->serviceClient<jsk_recognition_msgs::ICPAlign>("icp_align");
            jsk_recognition_msgs::ICPAlign icp_srv;

            if (clip_unseen_pointcloud_) {
              // Before running ICP, remove pointcloud where we cannot see
              // First, transform reference pointcloud, that is all_cloud_, into
              // sensor frame.
              // And after that, remove points which are x < 0.
              tf::StampedTransform global_sensor_tf_transform
                = lookupTransformWithDuration(
                  tf_listener_,
                  global_frame_,
                  sensor_frame_,
                  cloud_msg->header.stamp,
                  ros::Duration(1.0));
              Eigen::Affine3f global_sensor_transform;
              tf::transformTFToEigen(global_sensor_tf_transform,
                                     global_sensor_transform);
              pcl::PointCloud<pcl::PointNormal>::Ptr sensor_cloud
                (new pcl::PointCloud<pcl::PointNormal>);
              pcl::transformPointCloudWithNormals(
                *all_cloud_,
                *sensor_cloud,
                global_sensor_transform.inverse());
              // Remove negative-x points
              pcl::PassThrough<pcl::PointNormal> pass;
              pass.setInputCloud(sensor_cloud);
              pass.setFilterFieldName("x");
              pass.setFilterLimits(0.0, 100.0);
              pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud
                (new pcl::PointCloud<pcl::PointNormal>);
              pass.filter(*filtered_cloud);
              NODELET_INFO("clipping: %lu -> %lu", sensor_cloud->points.size(), filtered_cloud->points.size());
              // Convert the pointcloud to global frame again
              pcl::PointCloud<pcl::PointNormal>::Ptr global_filtered_cloud
                (new pcl::PointCloud<pcl::PointNormal>);
              pcl::transformPointCloudWithNormals(
                *filtered_cloud,
                *global_filtered_cloud,
                global_sensor_transform);
              pcl::toROSMsg(*global_filtered_cloud,
                            icp_srv.request.target_cloud);
            }
            else {
              pcl::toROSMsg(*all_cloud_,
                            icp_srv.request.target_cloud);
            }
            pcl::toROSMsg(*input_downsampled_cloud,
                          icp_srv.request.reference_cloud);
            
            if (client.call(icp_srv)) {
              Eigen::Affine3f transform;
              tf::poseMsgToEigen(icp_srv.response.result.pose, transform);
              Eigen::Vector3f transform_pos(transform.translation());
              float roll, pitch, yaw;
              pcl::getEulerAngles(transform, roll, pitch, yaw);
              NODELET_INFO("aligned parameter --");
              NODELET_INFO("  - pos: [%f, %f, %f]",
                           transform_pos[0],
                           transform_pos[1],
                           transform_pos[2]);
              NODELET_INFO("  - rot: [%f, %f, %f]", roll, pitch, yaw);
              pcl::PointCloud<pcl::PointNormal>::Ptr
                transformed_input_cloud (new pcl::PointCloud<pcl::PointNormal>);
              if (use_normal_) {
                pcl::transformPointCloudWithNormals(*input_cloud,
                                                    *transformed_input_cloud,
                                                    transform);
              }
              else {
                pcl::transformPointCloud(*input_cloud,
                                         *transformed_input_cloud,
                                         transform);
              }
              pcl::PointCloud<pcl::PointNormal>::Ptr
                concatenated_cloud (new pcl::PointCloud<pcl::PointNormal>);
              *concatenated_cloud = *all_cloud_ + *transformed_input_cloud;
              // update *all_cloud
              applyDownsampling(concatenated_cloud, *all_cloud_);
              // update localize_transform_
              tf::Transform icp_transform;
              tf::transformEigenToTF(transform, icp_transform);
              {
                boost::mutex::scoped_lock tf_lock(tf_mutex_);
                localize_transform_ = localize_transform_ * icp_transform;
              }
            }
            else {
              NODELET_ERROR("Failed to call ~icp_align");
              return;
            }
          }
          localize_requested_ = false;
        }
        else {
          NODELET_WARN("No tf transformation is available");
        }
      }
      catch (tf2::ConnectivityException &e)
      {
        NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
        return;
      }
      catch (tf2::InvalidArgumentException &e)
      {
        NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
        return;
      }
    }
  }

  bool PointCloudLocalization::isFirstTime()
  {
    return first_time_;
  }

  bool PointCloudLocalization::localizationRequest(
    std_srvs::Empty::Request& req,
    std_srvs::Empty::Response& res)
  {
    NODELET_INFO("localize!");
    boost::mutex::scoped_lock lock(mutex_);
    localize_requested_ = true;
    return true;
  }

  bool PointCloudLocalization::updateOffsetCallback(
    jsk_recognition_msgs::UpdateOffset::Request& req,
    jsk_recognition_msgs::UpdateOffset::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    geometry_msgs::TransformStamped next_pose = req.transformation;
    // convert geometry_msgs::TransformStamped into tf::Transform
    tf::StampedTransform tf_transform;
    tf::transformStampedMsgToTF(next_pose, tf_transform);
    // TODO: resolve tf
    localize_transform_ = tf_transform;
    return true;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointCloudLocalization, nodelet::Nodelet);
