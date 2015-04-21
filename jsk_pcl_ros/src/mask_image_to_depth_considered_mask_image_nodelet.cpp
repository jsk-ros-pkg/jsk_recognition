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
#include "jsk_pcl_ros/mask_image_to_depth_considered_mask_image.h"
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace jsk_pcl_ros
{
  void MaskImageToDepthConsideredMaskImage::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &MaskImageToDepthConsideredMaskImage::configCallback, this, _1, _2);
    srv_->setCallback (f);
    sub_ = pnh_->subscribe("input/maskregion", 1, &MaskImageToDepthConsideredMaskImage::mask_region_callback, this);
    region_width_ = 0;
    region_height_ = 0;
    region_x_off_ = 0;
    region_y_off_ = 0;
  }

  void MaskImageToDepthConsideredMaskImage::configCallback(Config &config, uint32_t level){
    boost::mutex::scoped_lock lock(mutex_);
    extract_num_ = config.extract_num;
    use_mask_region_ = config.use_mask_region;
    in_the_order_of_depth_ = config.in_the_order_of_depth;
  }


  void MaskImageToDepthConsideredMaskImage::mask_region_callback(const sensor_msgs::Image::ConstPtr& msg){
    boost::mutex::scoped_lock lock(mutex_);
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy
      (msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat mask = cv_ptr->image;
    int tmp_width = 0;
    int tmp_height = 0;
    int tmp_x_off = 0;
    int tmp_y_off = 0;
    bool flag = true;
    for (size_t j = 0; j < mask.rows; j++) {
      for (size_t i = 0; i < mask.cols; i++) {
        if (mask.at<uchar>(j, i) != 0) {
          if (flag == true) {
            tmp_x_off = i;
            tmp_y_off = j;
            flag = false;
          }
          else {
            tmp_width = i-tmp_x_off + 1;
            tmp_height = j-tmp_y_off + 1;
          }}}}
    ROS_INFO("mask resion callback: width:%d height:%d x_off:%d y_off:%d", tmp_width, tmp_height, tmp_x_off, tmp_y_off);
    region_width_ = tmp_width;
    region_height_ = tmp_height;
    region_x_off_ = tmp_x_off;
    region_y_off_ = tmp_y_off;
  }


  void MaskImageToDepthConsideredMaskImage::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_image_.subscribe(*pnh_, "input/image", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_input_, sub_image_);
      async_->registerCallback(boost::bind(&MaskImageToDepthConsideredMaskImage::extractmask, this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_image_);
      sync_->registerCallback(boost::bind(&MaskImageToDepthConsideredMaskImage::extractmask,
					  this, _1, _2));
    }
  }
  
  void MaskImageToDepthConsideredMaskImage::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_image_.unsubscribe();
  }

  void MaskImageToDepthConsideredMaskImage::extractmask
  (
     const sensor_msgs::PointCloud2::ConstPtr& point_cloud2_msg,
     const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (point_cloud2_msg->width == image_msg->width && point_cloud2_msg->height == image_msg->height){
      if (in_the_order_of_depth_ == true){
        vital_checker_->poke();
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*point_cloud2_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr
          edge_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        int width = image_msg->width;
        int height = image_msg->height;
        bool points_exist = false;
        pcl::PointXYZ nan_point;
        nan_point.x = NAN;
        nan_point.y = NAN;
        nan_point.z = NAN;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy
          (image_msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat mask = cv_ptr->image;
        edge_cloud->is_dense = false;
        edge_cloud->points.resize(width * height);
        edge_cloud->width = width;
        edge_cloud->height = height;
        if (use_mask_region_ == false || region_width_ == 0 || region_height_ == 0){
          for (size_t j = 0; j < mask.rows; j++) {
            for (size_t i = 0; i < mask.cols; i++) {
              if (mask.at<uchar>(j, i) != 0) {//if white
                points_exist = true;
                edge_cloud->points[j * width + i] = cloud->points[j * width + i];
              }
              else {
                edge_cloud->points[j * width + i] = nan_point;
              }
            }
          }
        }
        else {
          ROS_INFO("directed region width:%d height:%d", region_width_, region_height_);
          //set nan_point to all points first.
          for (size_t j = 0; j < mask.rows; j++) {
            for (size_t i = 0; i < mask.cols; i++) {
              edge_cloud->points[j * width + i] = nan_point;
            }
          }
          int x_end = region_x_off_+ region_width_;
          int y_end = region_y_off_+ region_height_;
          for (size_t j = region_y_off_; j < y_end; j++) {
            for (size_t i = region_x_off_; i < x_end; i++) {
              if (i < image_msg->width && j <image_msg->height){
                if (mask.at<uchar>(j, i) != 0) {//if white
                  points_exist = true;
                  edge_cloud->points[j * width + i] = cloud->points[j * width + i];
                }
              }
            }
          }
        }

        if (points_exist) {
          cv::Mat mask_image = cv::Mat::zeros(height, width, CV_8UC1);
          pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
          kdtree.setInputCloud(edge_cloud);
          pcl::PointXYZ zero;
          std::vector<int> near_indices;
          std::vector<float> near_distances;
          kdtree.nearestKSearch(zero, extract_num_, near_indices, near_distances);
          ROS_INFO("directed num of extract points:%d   num of nearestKSearch points:%d", extract_num_, ((int) near_indices.size()));
          int ext_num=std::min(extract_num_, ((int) near_indices.size()));
          for (int idx = 0; idx < ext_num; idx++) {
            int x = near_indices.at(idx) % width;
            int y = near_indices.at(idx) / width;
            mask_image.at<uchar>(y, x) = 255;
          }
          cv_bridge::CvImage mask_bridge(point_cloud2_msg->header,
                                         sensor_msgs::image_encodings::MONO8,
                                         mask_image);
          pub_.publish(mask_bridge.toImageMsg());
        }
      }
      else {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy
          (image_msg, sensor_msgs::image_encodings::MONO8);
        cv::Mat mask = cv_ptr->image;
        cv::Mat tmp_mask = cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC1);
        int data_len=image_msg->width * image_msg->height;
        int cnt = 0;
        if (use_mask_region_ ==false || region_width_ == 0 || region_height_ == 0){
          for (size_t j = 0; j < mask.rows; j++) {
            for (size_t i = 0; i < mask.cols; i++) {
              if (mask.at<uchar>(j, i) != 0) {//if white
                cnt++;
                if (std::min(extract_num_ , data_len) > cnt) {
                  tmp_mask.at<uchar>(j, i) = mask.at<uchar>(j, i);
                }
              }
            }
          }
        }
        else {
          ROS_INFO("directed region width:%d height:%d", region_width_, region_height_);
          int x_end = region_x_off_ + region_width_;
          int y_end = region_y_off_ + region_height_;
          for (size_t j = region_y_off_; j < y_end; j++) {
            for (size_t i = region_x_off_; i < x_end; i++) {
              if (i < image_msg->width && j <image_msg->height){
                if (mask.at<uchar>(j, i) != 0) {//if white
                  cnt++;
                  if (std::min(extract_num_ , data_len) > cnt) {
                    tmp_mask.at<uchar>(j, i) = mask.at<uchar>(j, i);
                  }
                }
              }
            }
          }
        }
        cv_bridge::CvImage mask_bridge(point_cloud2_msg->header,
                                       sensor_msgs::image_encodings::MONO8,
                                       tmp_mask);
        pub_.publish(mask_bridge.toImageMsg());
      }
    }
    else {
      ROS_ERROR ("ERROR: Different width and height. Points[width:%d height:%d] Image[width:%d height:%d]", point_cloud2_msg->width, point_cloud2_msg->height, image_msg->width, image_msg->height);
    }
  }

  void MaskImageToDepthConsideredMaskImage::updateDiagnostic(
    diagnostic_updater::DiagnosticStatusWrapper &stat)
  {
    if (vital_checker_->isAlive()) {
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK,
                   "MaskImageToDepthConsideredMaskImage running");
    }
    else {
      jsk_topic_tools::addDiagnosticErrorSummary(
        "MaskImageToDepthConsideredMaskImage", vital_checker_, stat);
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::MaskImageToDepthConsideredMaskImage, nodelet::Nodelet);
