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
#include "jsk_pcl_ros/extract_indices.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#if PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 8
#include <pcl/filters/extract_indices.h>
#else
#include "jsk_pcl_ros/pcl/extract_indices_patch.h"
#endif
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

namespace jsk_pcl_ros
{
  void ExtractIndices::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("keep_organized", keep_organized_, false);
    pnh_->param("negative", negative_, false);
    pnh_->param("max_queue_size", max_queue_size_, 10);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ExtractIndices::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", max_queue_size_);
    sub_indices_.subscribe(*pnh_, "indices", max_queue_size_);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_indices_, sub_cloud_);
      async_->registerCallback(
        boost::bind(&ExtractIndices::convert, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_indices_, sub_cloud_);
      sync_->registerCallback(
        boost::bind(&ExtractIndices::convert, this, _1, _2));
    }
  }

  void ExtractIndices::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void ExtractIndices::convert(
    const PCLIndicesMsg::ConstPtr& indices_msg,
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    vital_checker_->poke();

    pcl::PCLPointCloud2::Ptr input(new pcl::PCLPointCloud2);
    pcl_conversions::toPCL(*cloud_msg, *input);
    pcl::PointIndices::Ptr indices (new pcl::PointIndices ());
    pcl_conversions::toPCL(*indices_msg, *indices);

    // extract pointcloud with indices
    pcl::ExtractIndices<pcl::PCLPointCloud2> extract;
    extract.setInputCloud(input);
    extract.setIndices(indices);
    extract.setKeepOrganized(keep_organized_);
    extract.setNegative(negative_);
    pcl::PCLPointCloud2 output;
    extract.filter(output);

    sensor_msgs::PointCloud2 out_cloud_msg;
#if PCL_MAJOR_VERSION <= 1 && PCL_MINOR_VERSION < 8
    if (indices_msg->indices.empty() || cloud_msg->data.empty()) {
      out_cloud_msg.height = cloud_msg->height;
      out_cloud_msg.width = cloud_msg->width;
    }
#endif
    pcl_conversions::moveFromPCL(output, out_cloud_msg);

    out_cloud_msg.header = cloud_msg->header;
    pub_.publish(out_cloud_msg);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ExtractIndices, nodelet::Nodelet);
