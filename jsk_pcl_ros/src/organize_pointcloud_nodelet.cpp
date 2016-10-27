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


#include "jsk_pcl_ros/organize_pointcloud.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void OrganizePointCloud::extract(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    // skip empty cloud
    ROS_INFO_STREAM("received input clouds, convert range image, resolution: " << angular_resolution << ", width(deg): " << angle_width << ", height(deg):" << angle_height << ", min_points:" << min_points);

    if ( input->width < min_points ) return;

    pcl::PointCloud<pcl::PointXYZ> pointCloud;
    pcl::fromROSMsg(*input, pointCloud);

    // We now want to create a range image from the above point cloud, with a 1deg angular resolution
    float angularResolution = (float) (angular_resolution * (M_PI/180.0f));  //   1.0 degree in radians
    float maxAngleWidth     = (float) (angle_width * (M_PI/180.0f));  // 120.0 degree in radians
    float maxAngleHeight    = (float) (angle_height * (M_PI/180.0f));  // 90.0 degree in radians
    Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
    float noiseLevel=0.00;
    float minRange = 0.0f;
    int borderSize = 1;

    pcl::RangeImage rangeImage;
    rangeImage.createFromPointCloud(pointCloud, angularResolution, maxAngleWidth, maxAngleHeight,
                                    sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
    ROS_INFO_STREAM("input image size " << input->width << " x " << input->height << "(=" << input->width * input->height << ")");
    ROS_INFO_STREAM("output image size " << rangeImage.width << " x " << rangeImage.height << "(=" << rangeImage.width * rangeImage.height << ")");
    ROS_DEBUG_STREAM(rangeImage);

    sensor_msgs::PointCloud2 out;
    pcl::toROSMsg(rangeImage, out);
    out.header = input->header;
    pub_.publish(out);
  }

  void OrganizePointCloud::onInit(void)
  {
    ConnectionBasedNodelet::onInit();
    
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pnh_->param<double>("angular_resolution", angular_resolution, 1.0);
    pnh_->param<double>("angle_width", angle_width, 120.0);
    pnh_->param<double>("angle_height", angle_height, 90.0);
    pnh_->param<int>("min_points", min_points, 1000);
    onInitPostProcess();
  }

  void OrganizePointCloud::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &OrganizePointCloud::extract, this);
  }

  void OrganizePointCloud::unsubscribe()
  {
    sub_.shutdown();
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OrganizePointCloud, nodelet::Nodelet);

