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
#include "jsk_pcl_ros/mask_image_cluster_filter.h"
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <sensor_msgs/image_encodings.h>
#include <pcl/filters/extract_indices.h>


namespace jsk_pcl_ros
{
  void MaskImageClusterFilter::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<PCLIndicesMsg>(
      *pnh_, "output", 1);
    onInitPostProcess();
  }

  void MaskImageClusterFilter::subscribe()
  {
    sub_image_ = pnh_->subscribe("input/mask", 1,
                                 &MaskImageClusterFilter::imageCalback, this);
    sub_info_ = pnh_->subscribe("input/camera_info", 1,
                                &MaskImageClusterFilter::infoCalback, this);
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_target_.subscribe(*pnh_, "target", 1); 
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_input_, sub_target_);
    sync_->registerCallback(boost::bind(&MaskImageClusterFilter::concat, this, _1, _2));
  }
  
  void MaskImageClusterFilter::unsubscribe()
  {
    sub_image_.shutdown();
    sub_info_.shutdown();
    sub_input_.unsubscribe();
    sub_target_.unsubscribe();
  }
  
  void MaskImageClusterFilter::infoCalback(
    const sensor_msgs::CameraInfo::ConstPtr& info_ms)
  {
    boost::mutex::scoped_lock lock(mutex_);
    camera_info_ = info_ms;
  }

  void MaskImageClusterFilter::imageCalback(
    const sensor_msgs::Image::ConstPtr& mask_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
      mask_msg, sensor_msgs::image_encodings::MONO8);
    mask_image_ = cv_ptr->image;
  }

  void MaskImageClusterFilter::concat(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices_input)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (camera_info_ && !mask_image_.empty()) {
      image_geometry::PinholeCameraModel model;
      model.fromCameraInfo(camera_info_);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
        (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *cloud);
      PCLIndicesMsg indices;
      indices.header = cloud_msg->header;
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(cloud);
      for (size_t i = 0; i < indices_input->cluster_indices.size(); i++) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::IndicesPtr vindices;
        vindices.reset (new std::vector<int> (indices_input->cluster_indices[i].indices));
        extract.setIndices(vindices);
        extract.filter(*segmented_cloud);
        bool in_mask = false;
        for (size_t j = 0; j < segmented_cloud->points.size(); j++) {
          pcl::PointXYZRGB p = segmented_cloud->points[j];
          cv::Point2d uv = model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
          // check size
          if (uv.x > 0 && uv.x < mask_image_.cols &&
              uv.y > 0 && uv.y < mask_image_.rows) {
            if (mask_image_.at<uchar>(uv.y, uv.x) == 255) {
              in_mask = true;
              break;
            }
          }
        }
        if (in_mask) {
          vindices->size();
          for(size_t j=0; j < vindices->size(); j++)
            {
              indices.indices.push_back((*vindices)[j]);
            }
        }
      }
      pub_.publish(indices);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::MaskImageClusterFilter, nodelet::Nodelet);


