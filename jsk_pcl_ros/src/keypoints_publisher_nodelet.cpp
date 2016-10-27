/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuki Furuta and JSK Lab
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

#include "jsk_pcl_ros/keypoints_publisher.h"
#include <pluginlib/class_list_macros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <pcl/range_image/range_image_planar.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>

namespace jsk_pcl_ros
{
  void KeypointsPublisher::onInit(void)
  {
    ConnectionBasedNodelet::onInit();
    
    input_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    keypoints_pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "nerf_keypoints", 10);
    
    onInitPostProcess();
  }

  void KeypointsPublisher::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &KeypointsPublisher::inputCallback, this);
  }

  void KeypointsPublisher::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void KeypointsPublisher::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::fromROSMsg(*input, *input_);
    input_header_ = input->header;
    extractKeypoints(input_);
  }

  void KeypointsPublisher::extractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::RangeImagePlanar rip;
    pcl::RangeImageBorderExtractor ribe;
    rip.createFromPointCloudWithFixedSize(*cloud, cloud->width, cloud->height,
					  319.5, 239.5, 525.0, 525.0, static_cast<Eigen::Affine3f>(Eigen::Translation3f(0.0, 0.0, 0.0)));
    rip.setUnseenToMaxRange();
    ROS_INFO_STREAM("Built range image " << rip.width << "x" << rip.height);

    pcl::NarfKeypoint narf;
    narf.setRangeImageBorderExtractor(&ribe);
    narf.setRangeImage(&rip);
    narf.getParameters().support_size = 0.1;
    narf.getParameters().use_recursive_scale_reduction = true;
    narf.getParameters().calculate_sparse_interest_image = true;

    pcl::PointCloud<int> indices;
    narf.compute(indices);

    pcl::PointCloud<pcl::PointXYZ> result;
    for (int i = 0; i < indices.size(); ++i) {
      result.push_back(cloud->at(indices[i]));
    }
    sensor_msgs::PointCloud2 resMsg;
    pcl::toROSMsg(result, resMsg);
    resMsg.header = input_header_;
    keypoints_pub_.publish(resMsg);
  }
  
}
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::KeypointsPublisher, nodelet::Nodelet);
