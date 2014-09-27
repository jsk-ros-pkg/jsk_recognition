/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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


#include "jsk_pcl_ros/centroid_publisher.h"
#include <pluginlib/class_list_macros.h>


#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void CentroidPublisher::extract(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<pcl::PointXYZ> cloud_xyz;
    pcl::fromROSMsg(*input, cloud_xyz);
    Eigen::Vector4f center;
    pcl::compute3DCentroid(cloud_xyz, center);
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(center[0], center[1], center[2]));
    transform.setRotation(tf::createIdentityQuaternion());
    br.sendTransform(tf::StampedTransform(transform, input->header.stamp,
                                          input->header.frame_id, frame));
  }
  
  void CentroidPublisher::onInit(void)
  {
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &CentroidPublisher::extract, this);
    if (!pnh_->getParam("frame", frame))
    {
      ROS_WARN("~frame is not specified, using %s", getName().c_str());
      frame = getName();
    }
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::CentroidPublisher, nodelet::Nodelet);
