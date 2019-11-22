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

#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_pcl_ros/roi_clipper.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void ROIClipper::onInit()
  {
    DiagnosticNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    pnh_->param("not_sync", not_sync_, false);
    pnh_->param("keep_organized", keep_organized_, false);
    pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    if (not_sync_) {
      pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output/cloud", 1);
      pub_cloud_indices_ = advertise<PCLIndicesMsg>(*pnh_, "output/cloud_indices", 1);
    }
    onInitPostProcess();
  }

  void ROIClipper::subscribe()
  {
    if (!not_sync_) {
      sub_image_.subscribe(*pnh_, "input/image", 1);
      sub_info_.subscribe(*pnh_, "input/camera_info", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image_, sub_info_);
      sync_->registerCallback(boost::bind(&ROIClipper::clip, this, _1, _2));
    }
    else {
      sub_image_no_sync_ = pnh_->subscribe(
        "input/image", 1,
        &ROIClipper::imageCallback, this);
      sub_info_no_sync_ = pnh_->subscribe(
        "input/camera_info", 1,
        &ROIClipper::infoCallback, this);
      sub_cloud_no_sync_ = pnh_->subscribe(
        "input/cloud", 1,
        &ROIClipper::cloudCallback, this);
    }
  }
  
  void ROIClipper::unsubscribe()
  {
    if (!not_sync_) {
      sub_image_.unsubscribe();
      sub_info_.unsubscribe();
    }
    else {
      sub_image_no_sync_.shutdown();
      sub_info_no_sync_.shutdown();
      sub_cloud_no_sync_.shutdown();
    }
  }
  void ROIClipper::clip(const sensor_msgs::Image::ConstPtr& image_msg,
                        const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg)
  {
    vital_checker_->poke();
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::RGB8);
      cv::Mat image = cv_ptr->image;
      cv::Rect roi(camera_info_msg->roi.x_offset, camera_info_msg->roi.y_offset,
                   camera_info_msg->roi.width, camera_info_msg->roi.height);
      //NODELET_INFO("roi::(%d, %d, %d, %d)", roi.x, roi.y, roi.width, roi.height);
      cv::Mat image_roi = image(roi);
      // cv::imshow("roi", image_roi);
      // cv::waitKey(3);
      cv_bridge::CvImage roi_bridge(image_msg->header,
                                    sensor_msgs::image_encodings::RGB8,
                                    image_roi);
      pub_image_.publish(roi_bridge.toImageMsg());
    }
    catch (cv_bridge::Exception& e)
    {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void ROIClipper::infoCallback(
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_camera_info_ = info_msg;
  }

  void ROIClipper::imageCallback(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (latest_camera_info_) {
      clip(image_msg, latest_camera_info_);
    }
  }

  void ROIClipper::cloudCallback(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    if (latest_camera_info_) {
      image_geometry::PinholeCameraModel model;
      PCLIndicesMsg indices;
      model.fromCameraInfo(latest_camera_info_);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud
        (new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *cloud);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr clipped_cloud
        (new pcl::PointCloud<pcl::PointXYZRGB>);
      cv::Rect region = model.rectifiedRoi();
      pcl::PointXYZRGB nan_point;
      nan_point.x = nan_point.y = nan_point.z
        = std::numeric_limits<float>::quiet_NaN();;
      for (size_t i = 0; i < cloud->points.size(); i++) {
        pcl::PointXYZRGB p = cloud->points[i];
        bool foundp = false;
        if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z)) {
          cv::Point2d uv = model.project3dToPixel(cv::Point3d(p.x, p.y, p.z));
          if (uv.x >= 0 && uv.x <= region.width &&
              uv.y >= 0 && uv.y <= region.height) {
            indices.indices.push_back(i);
            clipped_cloud->points.push_back(p);
            foundp = true;
          }
        }
        if (!foundp && keep_organized_) {
          clipped_cloud->points.push_back(nan_point);
        }
      }
      if (keep_organized_) {
        clipped_cloud->width = cloud->width;
        clipped_cloud->height = cloud->height;
      }
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*clipped_cloud, ros_cloud);
      ros_cloud.header = cloud_msg->header;
      pub_cloud_.publish(ros_cloud);
      indices.header = cloud_msg->header;
      pub_cloud_indices_.publish(indices);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ROIClipper, nodelet::Nodelet);
