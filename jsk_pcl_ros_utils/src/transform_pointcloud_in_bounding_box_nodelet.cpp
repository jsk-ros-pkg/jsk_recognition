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

#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros_utils/transform_pointcloud_in_bounding_box.h"
#include <eigen_conversions/eigen_msg.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/common/transforms.h>

namespace jsk_pcl_ros_utils
{  
  void TransformPointcloudInBoundingBox::onInit()
  {
    PCLNodelet::onInit();
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    ////////////////////////////////////////////////////////
    // pulishers
    ////////////////////////////////////////////////////////
    pub_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_offset_pose_ = pnh_->advertise<geometry_msgs::PoseStamped>(
      "output_offset", 1);

    ////////////////////////////////////////////////////////
    // Subscription
    ////////////////////////////////////////////////////////
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_box_.subscribe(*pnh_, "input_box", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_box_);
    sync_->registerCallback(boost::bind(
                              &TransformPointcloudInBoundingBox::transform,
                              this, _1, _2));
  }

  void TransformPointcloudInBoundingBox::transform(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg)
  {
    try
    {
      pcl::PointCloud<PointT> output;
      Eigen::Affine3f offset;
      transformPointcloudInBoundingBox<PointT>(
        *box_msg, *msg,
        output, offset,
        *tf_listener_);
      sensor_msgs::PointCloud2 ros_output;
      pcl::toROSMsg(output, ros_output);
      pub_cloud_.publish(ros_output);
      //pub_offset_pose_.publish(box_pose_respected_to_cloud);
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
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::TransformPointcloudInBoundingBox,
                        nodelet::Nodelet);
