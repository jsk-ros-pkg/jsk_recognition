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

#include "jsk_pcl_ros/heightmap_time_accumulation.h"
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/common/transforms.h>

namespace jsk_pcl_ros
{
  void HeightmapTimeAccumulation::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pub_config_ = pnh_->advertise<jsk_recognition_msgs::HeightmapConfig>(
      "output/config", 1, true);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&HeightmapTimeAccumulation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    sub_config_ = pnh_->subscribe(
      getHeightmapConfigTopic(pnh_->resolveName("input")), 1,
      &HeightmapTimeAccumulation::configTopicCallback, this);
    if (!pnh_->getParam("center_frame_id", center_frame_id_)) {
      NODELET_FATAL("no ~center_frame_id is specified");
      return;
    }
    if (!pnh_->getParam("fixed_frame_id", fixed_frame_id_)) {
      NODELET_FATAL("no ~fixed_frame_id is specified");
      return;
    }
    int tf_queue_size;
    pnh_->param("tf_queue_size", tf_queue_size, 10);
    prev_from_center_to_fixed_ = Eigen::Affine3f::Identity();
    tf_ = TfListenerSingleton::getInstance();
    pub_output_ = pnh_->advertise<sensor_msgs::Image>("output", 1, true);
    sub_previous_pointcloud_ = pnh_->subscribe<sensor_msgs::PointCloud2>(
      "input/prev_pointcloud", 5, 
      &HeightmapTimeAccumulation::prevPointCloud, this);
    sub_heightmap_.subscribe(*pnh_, "input", 10);
    tf_filter_.reset(new tf::MessageFilter<sensor_msgs::Image>(
                       sub_heightmap_,
                       *tf_,
                       fixed_frame_id_,
                       tf_queue_size));
    tf_filter_->registerCallback(
      boost::bind(
        &HeightmapTimeAccumulation::accumulate, this, _1));
    srv_reset_ = pnh_->advertiseService("reset", &HeightmapTimeAccumulation::resetCallback, this);
    onInitPostProcess();
  }
  
  void HeightmapTimeAccumulation::subscribe()
  {

  }
  void HeightmapTimeAccumulation::unsubscribe()
  {

  }


  void HeightmapTimeAccumulation::publishHeightmap(
    const cv::Mat& heightmap, const std_msgs::Header& header)
  {
    pub_output_.publish(cv_bridge::CvImage(
                          header,
                          sensor_msgs::image_encodings::TYPE_32FC2,
                          heightmap).toImageMsg());
  }

  cv::Point HeightmapTimeAccumulation::toIndex(
    const PointType& p, const cv::Mat& map)
  {
    if (p.x > max_x_ || p.x < min_x_ ||
        p.y > max_y_ || p.y < min_y_) {
      return cv::Point(-1, -1);
    }
    const float offsetted_x = p.x - min_x_;
    const float offsetted_y = p.y - min_y_;
    const float dx = (max_x_ - min_x_) / map.cols;
    const float dy = (max_y_ - min_y_) / map.rows;
    return cv::Point(std::floor(offsetted_x / dx),
                     std::floor(offsetted_y / dy));
  }

  bool HeightmapTimeAccumulation::isValidIndex(const cv::Point& index, const cv::Mat& map)
  {
    return (index.x >= 0 && index.x < map.cols && 
            index.y >= 0 && index.y < map.rows);
  }

  bool HeightmapTimeAccumulation::isValidCell(const cv::Point& index, const cv::Mat& map)
  {
    float v = map.at<cv::Vec2f>(index.y, index.x)[0];
    return !std::isnan(v) && v != -FLT_MAX;
  }

  void HeightmapTimeAccumulation::accumulate(
    const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!config_) {
      NODELET_ERROR("no ~input/config is yet available");
      return;
    }
    tf::StampedTransform tf_transform;
    try {
      tf_->lookupTransform(fixed_frame_id_, center_frame_id_,
                           msg->header.stamp,
                           tf_transform);
    }
    catch (tf2::TransformException &e) {
        NODELET_ERROR("Transform error: %s", e.what());
        return;
    }
    Eigen::Affine3f from_center_to_fixed;
    tf::transformTFToEigen(tf_transform, from_center_to_fixed);
    cv::Mat new_heightmap = cv_bridge::toCvShare(
      msg, sensor_msgs::image_encodings::TYPE_32FC2)->image;
    // Transform prev_cloud_ to current frame
    Eigen::Affine3f from_prev_to_current
      = prev_from_center_to_fixed_.inverse() * from_center_to_fixed;
    pcl::PointCloud<PointType > transformed_pointcloud;
    pcl::transformPointCloud(prev_cloud_, transformed_pointcloud, from_prev_to_current.inverse());

    mergedAccmulation(transformed_pointcloud, new_heightmap);

    publishHeightmap(new_heightmap, msg->header);
    // prev_from_center_to_fixed_ = from_center_to_fixed;
  }

  void HeightmapTimeAccumulation::overwriteAccmulation(
    pcl::PointCloud<PointType > &transformed_pointcloud, cv::Mat &new_heightmap)
  {
    for (size_t i = 0; i < transformed_pointcloud.points.size(); i++) {
      PointType p = transformed_pointcloud.points[i];
      if (isValidPoint(p)) {
        cv::Point index = toIndex(p, new_heightmap);
        if (isValidIndex(index, new_heightmap)) {
          if (!isValidCell(index, new_heightmap)) {
            // There is not valid data in current heightmap,
            new_heightmap.at<cv::Vec2f>(index.y, index.x)[0] = p.z;
            new_heightmap.at<cv::Vec2f>(index.y, index.x)[1] = (float)(p.intensity);
          } else {
            // There is valid data in current heightmap,
            if (new_heightmap.at<cv::Vec2f>(index.y, index.x)[1] < (float)(p.intensity)) {
              // heightmap has worth quality than prev_pointcloud
              new_heightmap.at<cv::Vec2f>(index.y, index.x)[0] = p.z;
              new_heightmap.at<cv::Vec2f>(index.y, index.x)[1] = (float)(p.intensity);
            }
          }
        }
      }
    }
  }

  void HeightmapTimeAccumulation::mergedAccmulation(
    pcl::PointCloud<PointType > &transformed_pointcloud, cv::Mat &new_heightmap)
  {
    float offset_new_z = 0.0;
    float offset_org_z = 0.0;
    if (use_offset_) { // calc offset
      double diff_z = 0.0;
      long cntr = 0;
      for (size_t i = 0; i < transformed_pointcloud.points.size(); i++) {
        PointType p = transformed_pointcloud.points[i];
        if (isValidPoint(p)) {
          cv::Point index = toIndex(p, new_heightmap);
          if (isValidIndex(index, new_heightmap)) {
            if (isValidCell(index, new_heightmap)) {
              float new_z = new_heightmap.at<cv::Vec2f>(index.y, index.x)[0];
              float new_q = new_heightmap.at<cv::Vec2f>(index.y, index.x)[1];
              float org_z = p.z;
              float org_q = p.intensity;
              float tmp_diff = (new_z - org_z);
              if (std::fabs(tmp_diff) < 0.04) {
                diff_z += tmp_diff;
                cntr++;
              }
            }
          }
        }
      }
      if(cntr > 0) {
        diff_z /= cntr;
        offset_new_z = - diff_z/2.0;
        offset_org_z =   diff_z/2.0;
      }
      // add offset to new heightmap
      for(int yy = 0; yy < new_heightmap.rows; yy++) {
        for(int xx = 0; xx < new_heightmap.cols; xx++) {
          new_heightmap.at<cv::Vec2f>(yy, xx)[0] += offset_new_z;
        }
      }
    }

    //averaging strategy
    for (size_t i = 0; i < transformed_pointcloud.points.size(); i++) {
      PointType p = transformed_pointcloud.points[i];
      if (isValidPoint(p)) {
        cv::Point index = toIndex(p, new_heightmap);
        if (isValidIndex(index, new_heightmap)) {
          p.z += offset_org_z;
          if (!isValidCell(index, new_heightmap)) {
            // There is not valid data in current heightmap,
            new_heightmap.at<cv::Vec2f>(index.y, index.x)[0] = p.z;
            new_heightmap.at<cv::Vec2f>(index.y, index.x)[1] = (float)(p.intensity);
          } else {
            // There is valid data in current heightmap,
            float new_z = new_heightmap.at<cv::Vec2f>(index.y, index.x)[0];
            float new_q = new_heightmap.at<cv::Vec2f>(index.y, index.x)[1];
            float org_z = p.z;
            float org_q = p.intensity;
            new_z = (new_z * new_q + org_z * org_q)/(new_q + org_q);
            new_q = (new_q + org_q)/1.6; // ?? new quality
            if(new_q > 1.0) new_q = 1.0;
            new_heightmap.at<cv::Vec2f>(index.y, index.x)[0] = new_z;
            new_heightmap.at<cv::Vec2f>(index.y, index.x)[1] = new_q;
          }
        }
      }
    }

    if (use_bilateral_) {
      std::vector<cv::Mat> fimages;
      cv::split(new_heightmap, fimages);
      cv::Mat res_image;
      // filter
      cv::bilateralFilter(fimages[0], res_image, bilateral_filter_size_, bilateral_sigma_color_, bilateral_sigma_space_);
      {
        for (size_t j = 0; j < res_image.rows; j++) {
          for (size_t i = 0; i < res_image.cols; i++) {
            float ov = fimages[0].at<float>(j, i);
            if (ov == -FLT_MAX) {
              res_image.at<float>(j, i) = -FLT_MAX;
            }
          }
        }
      }
      // reconstruct images
      std::vector<cv::Mat> ret_images;
      ret_images.push_back(res_image);
      ret_images.push_back(fimages[1]);
      cv::merge(ret_images, new_heightmap);
    }
  }

  void HeightmapTimeAccumulation::prevPointCloud(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    pcl::fromROSMsg(*msg, prev_cloud_);
    // get transform at subscribed timestamp
    tf::StampedTransform tf_transform;
    try {
      tf_->lookupTransform(fixed_frame_id_, center_frame_id_,
                           msg->header.stamp,
                           tf_transform);
    }
    catch (tf2::TransformException &e) {
      NODELET_ERROR("Transform error: %s", e.what());
      return;
    }
    tf::transformTFToEigen(tf_transform, prev_from_center_to_fixed_);
  }

  void HeightmapTimeAccumulation::configTopicCallback(
    const jsk_recognition_msgs::HeightmapConfig::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    config_ = msg;
    min_x_ = msg->min_x;
    max_x_ = msg->max_x;
    min_y_ = msg->min_y;
    max_y_ = msg->max_y;
    pub_config_.publish(msg);
  }

  bool HeightmapTimeAccumulation::resetCallback(std_srvs::Empty::Request& req,
                                                std_srvs::Empty::Response& res)
  {
    boost::mutex::scoped_lock lock(mutex_);
    prev_from_center_to_fixed_ = Eigen::Affine3f::Identity();
    prev_cloud_.points.clear();
    return true;
  }

  void HeightmapTimeAccumulation::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    //min_x_ = config.min_x;
    //max_x_ = config.max_x;
    //min_y_ = config.min_y;
    //max_y_ = config.max_y;
    use_offset_    = config.use_offset;
    use_bilateral_ = config.use_bilateral;
    bilateral_filter_size_ = config.bilateral_filter_size;
    bilateral_sigma_color_ = config.bilateral_sigma_color;
    bilateral_sigma_space_ = config.bilateral_sigma_space;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HeightmapTimeAccumulation, nodelet::Nodelet);
