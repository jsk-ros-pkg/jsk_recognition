// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#include "jsk_pcl_ros_utils/pointcloud_to_pcd.h"
#include <sensor_msgs/PointCloud2.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_recognition_utils/tf_listener_singleton.h>


#include <pcl/io/io.h>
#include <Eigen/Geometry>

namespace jsk_pcl_ros_utils
{
    PointCloudToPCD::~PointCloudToPCD()
  {
    timer_.stop();
  }

  void PointCloudToPCD::timerCallback (const ros::TimerEvent& event)
  {
    pcl::PCLPointCloud2::ConstPtr cloud;
    cloud = ros::topic::waitForMessage<pcl::PCLPointCloud2>("input", *pnh_);
    if ((cloud->width * cloud->height) == 0)
    {
      return;
    }
  
    ROS_INFO ("Received %d data points in frame %s with the following fields: %s",
            (int)cloud->width * cloud->height,
            cloud->header.frame_id.c_str (),
            pcl::getFieldsList (*cloud).c_str ());
  
    Eigen::Vector4f v = Eigen::Vector4f::Zero ();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();
    if (!fixed_frame_.empty ()) {
        if (!tf_listener_->waitForTransform (fixed_frame_, cloud->header.frame_id, pcl_conversions::fromPCL (cloud->header).stamp, ros::Duration (duration_))) {
        ROS_WARN("Could not get transform!");
        return;
      }
      tf::StampedTransform transform_stamped;
      tf_listener_->lookupTransform (fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header).stamp, transform_stamped); 
      Eigen::Affine3d transform;
      tf::transformTFToEigen(transform_stamped, transform);
      v = Eigen::Vector4f::Zero ();
      v.head<3> () = transform.translation ().cast<float> ();
      q = transform.rotation ().cast<float> ();
    }
  
    std::stringstream ss;
    ss << prefix_ << cloud->header.stamp << ".pcd";
    ROS_INFO ("Data saved to %s", ss.str ().c_str ());
  
    pcl::PCDWriter writer;
    if(binary_)
    {
      if(compressed_)
      {
        writer.writeBinaryCompressed (ss.str (), *cloud, v, q);
      }
      else
      {
        writer.writeBinary (ss.str (), *cloud, v, q);
      }
    }
    else
    {
      writer.writeASCII (ss.str (), *cloud, v, q, 8);
    }
  }
  
  void PointCloudToPCD::onInit()
  {
    PCLNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&PointCloudToPCD::configCallback, this, _1, _2);
    srv_->setCallback (f);
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    if(binary_)
    {
      if(compressed_)
      {
        ROS_INFO_STREAM ("Saving as binary compressed PCD");
      }
      else
      {
        ROS_INFO_STREAM ("Saving as binary PCD");
      }
    }
    else
    {
      ROS_INFO_STREAM ("Saving as binary PCD");
    }
  }

  void PointCloudToPCD::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    prefix_ = config.prefix;
    binary_ = config.binary;
    compressed_ = config.compressed;
    fixed_frame_ = config.fixed_frame;
    duration_ = config.duration;
    timer_ = pnh_->createTimer(ros::Duration(duration_), boost::bind(&PointCloudToPCD::timerCallback, this, _1));
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PointCloudToPCD, nodelet::Nodelet);
