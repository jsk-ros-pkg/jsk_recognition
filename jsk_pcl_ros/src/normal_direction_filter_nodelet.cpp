// -*- mode: c++ -*-
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

#include "jsk_pcl_ros/normal_direction_filter.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/rosparam_utils.h>
#include <eigen_conversions/eigen_msg.h>

namespace jsk_pcl_ros
{
  void NormalDirectionFilter::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("use_imu", use_imu_, false);
    if (!use_imu_) {
      std::vector<double> direction;
      if (!jsk_topic_tools::readVectorParameter(*pnh_, "direction", direction)) {
        NODELET_ERROR("You need to specify ~direction");
        return;
      }
      jsk_recognition_utils::pointFromVectorToVector<std::vector<double>, Eigen::Vector3f>(
      direction, static_direction_);
    }
    else {
      tf_listener_ = TfListenerSingleton::getInstance();
    }
    
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &NormalDirectionFilter::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pnh_->param("queue_size", queue_size_, 200);
    pub_ = advertise<PCLIndicesMsg>(*pnh_, "output", 1);
    onInitPostProcess();
  }


  void NormalDirectionFilter::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    eps_angle_ = config.eps_angle;
    angle_offset_ = config.angle_offset;
  }

  void NormalDirectionFilter::subscribe()
  {
    if (!use_imu_) {
      sub_ = pnh_->subscribe("input", 1, &NormalDirectionFilter::filter, this);
    }
    else {
      sub_input_.subscribe(*pnh_, "input", 1);
      sub_imu_.subscribe(*pnh_, "input_imu", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_input_, sub_imu_);
      sync_->registerCallback(boost::bind(&NormalDirectionFilter::filter, this, _1, _2));
    }
  }

  void NormalDirectionFilter::unsubscribe()
  {
    if (!use_imu_) {
      sub_.shutdown();
    }
    else {
      sub_input_.unsubscribe();
      sub_imu_.unsubscribe();
    }
  }

  void NormalDirectionFilter::filterIndices(
    const pcl::PointCloud<pcl::Normal>::Ptr& normal_cloud,
    const Eigen::Vector3f& direction,
    pcl::PointIndices& indices)
  {
    for (size_t i = 0; i < normal_cloud->points.size(); i++) {
      Eigen::Vector3f normal = normal_cloud->points[i].getNormalVector3fMap().normalized();
      if (!std::isnan(normal[0]) &&
          !std::isnan(normal[1]) &&
          !std::isnan(normal[2])) {
        double dot = std::abs(normal.dot(direction));
        if (dot < -1.0) {
          dot = -1.0;
        }
        else if (dot > 1.0) {
          dot = 1.0;
        }
        double angle = acos(dot);
        double angle_diff = std::abs(angle - angle_offset_);
        if (angle_diff < eps_angle_) {
          indices.indices.push_back(i);
        }
      }
    }
  }

  void NormalDirectionFilter::filter(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::Imu::ConstPtr& imu_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*msg, *normal);
    geometry_msgs::Vector3Stamped stamped_imu, transformed_imu;
    stamped_imu.header = imu_msg->header;
    stamped_imu.vector = imu_msg->linear_acceleration;
    try
    {
      tf_listener_->waitForTransform(msg->header.frame_id, imu_msg->header.frame_id, imu_msg->header.stamp,
                                     ros::Duration(0.1));
      tf_listener_->transformVector(
        msg->header.frame_id, stamped_imu, transformed_imu);
      Eigen::Vector3d imu_vectord;
      Eigen::Vector3f imu_vector;
      tf::vectorMsgToEigen(transformed_imu.vector, imu_vectord);
      jsk_recognition_utils::pointFromVectorToVector<Eigen::Vector3d, Eigen::Vector3f>(
        imu_vectord, imu_vector);
      imu_vector.normalize();
      pcl::PointIndices indices;
      filterIndices(normal, imu_vector, indices);
      pcl_msgs::PointIndices ros_indices;
      pcl_conversions::fromPCL(indices, ros_indices);
      ros_indices.header = msg->header;
      pub_.publish(ros_indices);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::ExtrapolationException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
  }
  
  void NormalDirectionFilter::filter(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::fromROSMsg(*msg, *normal);

    pcl::PointIndices indices;
    filterIndices(normal, static_direction_, indices);
    pcl_msgs::PointIndices ros_indices;
    pcl_conversions::fromPCL(indices, ros_indices);
    ros_indices.header = msg->header;
    pub_.publish(ros_indices);
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::NormalDirectionFilter, nodelet::Nodelet);
