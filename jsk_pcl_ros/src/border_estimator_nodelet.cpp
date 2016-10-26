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
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <pcl/range_image/range_image_planar.h>
#include <pcl/range_image/range_image_spherical.h>
#include "jsk_pcl_ros/border_estimator.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  void BorderEstimator::onInit()
  {
    ConnectionBasedNodelet::onInit();
    // planar or spherical
    pnh_->param("model_type", model_type_, std::string("planar"));
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&BorderEstimator::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_border_ = advertise<PCLIndicesMsg>(*pnh_, "output_border_indices", 1);
    pub_veil_ = advertise<PCLIndicesMsg>(*pnh_, "output_veil_indices", 1);
    pub_shadow_ = advertise<PCLIndicesMsg>(*pnh_, "output_shadow_indices", 1);
    pub_range_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output_range_image", 1);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(
      *pnh_, "output_cloud", 1);

    onInitPostProcess();
  }

  void BorderEstimator::subscribe()
  {
    if (model_type_ == "planar") {
      sub_point_.subscribe(*pnh_, "input", 1);
      sub_camera_info_.subscribe(*pnh_, "input_camera_info", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_point_, sub_camera_info_);
      sync_->registerCallback(boost::bind(&BorderEstimator::estimate, this, _1, _2));
    }
    else if (model_type_ == "laser") {
      sub_ = pnh_->subscribe("input", 1, &BorderEstimator::estimate, this);
    }
  }

  void BorderEstimator::unsubscribe()
  {
    if (model_type_ == "planar") {
      sub_point_.unsubscribe();
      sub_camera_info_.unsubscribe();
    }
    else if (model_type_ == "laser") {
      sub_.shutdown();
    }
  }
  
  void BorderEstimator::publishCloud(
    ros::Publisher& pub,
    const pcl::PointIndices& inlier,
    const std_msgs::Header& header)
  {
    PCLIndicesMsg msg;
    msg.header = header;
    msg.indices = inlier.indices;
    pub.publish(msg);
  }

  void BorderEstimator::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);
    pcl::RangeImage range_image;
    if (model_type_ == "sphere") {
      range_image = pcl::RangeImageSpherical();
    }
    range_image.createFromPointCloud(
      *cloud,
      angular_resolution_,
      max_angle_width_, max_angle_height_,
      Eigen::Affine3f::Identity(),
      pcl::RangeImage::CAMERA_FRAME,
      noise_level_,
      min_range_,
      border_size_);
    range_image.setUnseenToMaxRange();
    computeBorder(range_image, msg->header);
  }

  void BorderEstimator::computeBorder(
    const pcl::RangeImage& range_image,
    const std_msgs::Header& header)
  {
    pcl::RangeImageBorderExtractor border_extractor (&range_image);
    pcl::PointCloud<pcl::BorderDescription> border_descriptions;
    border_extractor.compute (border_descriptions);
    pcl::PointIndices border_indices, veil_indices, shadow_indices;
    for (int y = 0; y < (int)range_image.height; ++y) {
      for (int x = 0; x < (int)range_image.width; ++x) {
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER]) {
          border_indices.indices.push_back (y*range_image.width + x);
        }
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT]) {
          veil_indices.indices.push_back (y*range_image.width + x);
        }
        if (border_descriptions.points[y*range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER]) {
          shadow_indices.indices.push_back (y*range_image.width + x);
        }
      }
    }
    publishCloud(pub_border_, border_indices, header);
    publishCloud(pub_veil_, veil_indices, header);
    publishCloud(pub_shadow_, shadow_indices, header);
    cv::Mat image;
    jsk_recognition_utils::rangeImageToCvMat(range_image, image);
    pub_range_image_.publish(
      cv_bridge::CvImage(header,
                         sensor_msgs::image_encodings::BGR8,
                         image).toImageMsg());
    // publish pointcloud
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(range_image, ros_cloud);
    ros_cloud.header = header;
    pub_cloud_.publish(ros_cloud);
  }
  
  void BorderEstimator::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& msg,
    const sensor_msgs::CameraInfo::ConstPtr& info)
  {
    if (msg->height == 1) {
      NODELET_ERROR("[BorderEstimator::estimate] pointcloud must be organized");
      return;
    }
    pcl::RangeImagePlanar range_image;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    Eigen::Affine3f dummytrans = Eigen::Affine3f::Identity();
    float fx = info->P[0];
    float cx = info->P[2];
    float tx = info->P[3];
    float fy = info->P[5];
    float cy = info->P[6];
    range_image.createFromPointCloudWithFixedSize (cloud,
                                                   msg->width,
                                                   msg->height,
                                                   cx, cy,
                                                   fx, fy,
                                                   dummytrans);
    range_image.setUnseenToMaxRange();
    computeBorder(range_image, msg->header);
  }

  void BorderEstimator::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    noise_level_ = config.noise_level;
    min_range_ = config.min_range;
    border_size_ = config.border_size;
    angular_resolution_ = config.angular_resolution;
    max_angle_height_ = config.max_angle_height;
    max_angle_width_ = config.max_angle_width;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::BorderEstimator,
                        nodelet::Nodelet);
