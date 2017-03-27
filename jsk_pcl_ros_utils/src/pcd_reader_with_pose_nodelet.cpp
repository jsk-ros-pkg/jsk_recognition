// -*- mode: c++; indent-tabs-mode: nil; -*-
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

#include "jsk_pcl_ros_utils/pcd_reader_with_pose.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <eigen_conversions/eigen_msg.h>
#include <pcl/common/transforms.h>
#include <eigen_conversions/eigen_msg.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>

namespace jsk_pcl_ros_utils
{
  void PCDReaderWithPose::onInit()
  {
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    DiagnosticNodelet::onInit();
    std::string file_name;
    pnh_->param("pcd_file", file_name, std::string(""));
    if (file_name == std::string("") || pcl::io::loadPCDFile (file_name, template_cloud_) == -1){
      NODELET_FATAL("cannot read pcd file %s", file_name.c_str());
      return;
    }
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    onInitPostProcess();
  }
  void PCDReaderWithPose::subscribe()
  {
    sub_teacher_ = pnh_->subscribe("input", 1,
                                       &PCDReaderWithPose::poseCallback,
                                       this);
  }
  void PCDReaderWithPose::unsubscribe()
  {
  }
  void PCDReaderWithPose::poseCallback(
    const geometry_msgs::PoseStamped::ConstPtr& pose_stamped)
  {
    vital_checker_->poke();
    ros::Time now = ros::Time::now();
    Eigen::Affine3f pose_eigen;
    tf::poseMsgToEigen(pose_stamped->pose, pose_eigen);
    sensor_msgs::PointCloud2 ros_out;
    Eigen::Matrix4f transform = pose_eigen.matrix();
    pcl_ros::transformPointCloud(transform ,template_cloud_, ros_out);
    ros_out.header = pose_stamped->header;
    pub_cloud_.publish(ros_out);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PCDReaderWithPose, nodelet::Nodelet);
