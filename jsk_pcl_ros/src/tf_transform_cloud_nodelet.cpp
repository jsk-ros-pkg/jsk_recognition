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


#include "jsk_pcl_ros/tf_transform_cloud.h"
#include <pluginlib/class_list_macros.h>

#include <tf2_ros/buffer_client.h>
#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void TfTransformCloud::transform(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    sensor_msgs::PointCloud2 output;
    try
    {
      if (pcl_ros::transformPointCloud(target_frame_id_, *input, output, tf_listener_)) {
        pub_cloud_.publish(output);
      }
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("Transform error: %s", e.what());
    }
  }

  void TfTransformCloud::onInit(void)
  {
    PCLNodelet::onInit();
    sub_cloud_ = pnh_->subscribe("input", 1, &TfTransformCloud::transform, this);
    if (!pnh_->getParam("target_frame_id", target_frame_id_))
    {
      ROS_WARN("~target_frame_id is not specified, using %s", "/base_footprint");
    }

    pub_cloud_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
  }
}

typedef jsk_pcl_ros::TfTransformCloud TfTransformCloud;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, TfTransformCloud, TfTransformCloud, nodelet::Nodelet);
