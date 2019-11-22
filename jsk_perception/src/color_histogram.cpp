// -*- mode: C++ -*-
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

#include "jsk_perception/color_histogram.h"
#include <jsk_topic_tools/log_utils.h>

namespace jsk_perception
{
  void ColorHistogram::configCallback(Config &new_config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    b_hist_size_ = new_config.blue_histogram_bin;
    g_hist_size_ = new_config.green_histogram_bin;
    r_hist_size_ = new_config.red_histogram_bin;
    h_hist_size_ = new_config.hue_histogram_bin;
    s_hist_size_ = new_config.saturation_histogram_bin;
    i_hist_size_ = new_config.intensity_histogram_bin;
    onInitPostProcess();
  }
    
  void ColorHistogram::onInit()
  {
    DiagnosticNodelet::onInit();
    nh_ = ros::NodeHandle(getNodeHandle(), "image");
    pnh_->param("use_mask", use_mask_, false);
    b_hist_size_ = r_hist_size_ = g_hist_size_ =
      h_hist_size_ = s_hist_size_ = i_hist_size_ = 512;
    b_hist_pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "blue_histogram", 1);
    g_hist_pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "green_histogram", 1);
    r_hist_pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "red_histogram", 1);
    h_hist_pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "hue_histogram", 1);
    s_hist_pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "saturation_histogram", 1);
    i_hist_pub_ = advertise<jsk_recognition_msgs::ColorHistogram>(
      *pnh_, "intensity_histogram", 1);
    image_pub_ = advertise<sensor_msgs::Image>(
      *pnh_, "input_image", 1);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ColorHistogram::configCallback, this, _1, _2);
    srv_->setCallback (f);
  }

  void ColorHistogram::subscribe()
  {
    ros::V_string names;
    if (!use_mask_) {
      it_.reset(new image_transport::ImageTransport(nh_));
      image_sub_.subscribe(*it_, "", 1);
      rectangle_sub_.subscribe(nh_, "screenrectangle", 1);
      names.push_back("screenrectangle");
      sync_
        = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
      sync_->connectInput(image_sub_, rectangle_sub_);
      sync_->registerCallback(boost::bind(
                                &ColorHistogram::extract, this, _1, _2));
    }
    else {
      it_.reset(new image_transport::ImageTransport(nh_));
      image_sub_.subscribe(*it_, "", 1);
      image_mask_sub_.subscribe(*it_, "mask", 1);
      names.push_back("mask");
      mask_sync_
        = boost::make_shared<message_filters::Synchronizer<
        MaskSyncPolicy> >(100);
      mask_sync_->connectInput(image_sub_, image_mask_sub_);
      mask_sync_->registerCallback(
        boost::bind(
          &ColorHistogram::extractMask, this, _1, _2));
    }
    jsk_topic_tools::warnNoRemap(names);
  }

  void ColorHistogram::unsubscribe()
  {
    if (!use_mask_) {
      image_sub_.unsubscribe();
      rectangle_sub_.unsubscribe();
    }
    else {
      image_sub_.unsubscribe();
      image_mask_sub_.unsubscribe();
    }
  }

  void ColorHistogram::convertHistogramToMsg(const cv::Mat& hist,
                                             int size,
                                             jsk_recognition_msgs::ColorHistogram& msg)
  {
    msg.histogram.clear();
    for (int i = 0; i < size; i++) {
      float val = hist.at<float>(0, i);
      msg.histogram.push_back(val);
    }
  }

  void ColorHistogram::processBGR(const cv::Mat& bgr_image,
                                  const cv::Mat& mask,
                                  const std_msgs::Header& header)
  {
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    cv::MatND b_hist, g_hist, r_hist;
    bool uniform = true; bool accumulate = false;
    std::vector<cv::Mat> bgr_planes;
    split(bgr_image, bgr_planes);
    
    cv::calcHist(&bgr_planes[0], 1, 0, mask, b_hist, 1, &b_hist_size_,
                 &histRange, uniform, accumulate);
    cv::calcHist(&bgr_planes[1], 1, 0, mask, g_hist, 1, &g_hist_size_,
                 &histRange, uniform, accumulate);
    cv::calcHist(&bgr_planes[2], 1, 0, mask, r_hist, 1, &r_hist_size_,
                 &histRange, uniform, accumulate);
      
    jsk_recognition_msgs::ColorHistogram b_histogram;
    b_histogram.header = header;
    convertHistogramToMsg(b_hist, b_hist_size_, b_histogram);
    b_hist_pub_.publish(b_histogram);
      
    jsk_recognition_msgs::ColorHistogram g_histogram;
    g_histogram.header = header;
    convertHistogramToMsg(g_hist, g_hist_size_, g_histogram);
    g_hist_pub_.publish(g_histogram);
      
    jsk_recognition_msgs::ColorHistogram r_histogram;
    r_histogram.header = header;
    convertHistogramToMsg(r_hist, r_hist_size_, r_histogram);
    r_hist_pub_.publish(r_histogram);
  }
  
  void ColorHistogram::processBGR(const cv::Mat& bgr_image,
                                  const std_msgs::Header& header)
  {
    processBGR(bgr_image, cv::Mat(), header);
  }

  void ColorHistogram::processHSI(const cv::Mat& bgr_image,
                                  const std_msgs::Header& header)
  {
    processHSI(bgr_image, cv::Mat(), header);
  }
  
  void ColorHistogram::processHSI(const cv::Mat& bgr_image,
                                  const cv::Mat& mask,
                                  const std_msgs::Header& header)
  {
    cv::Mat hsi_image;
    cv::cvtColor(bgr_image, hsi_image, CV_BGR2HSV);
    
    float range[] = { 0, 256 } ;
    const float* histRange = { range };
    float h_range[] = { 0, 180 } ;
    const float* h_histRange = { h_range };
    cv::MatND h_hist, s_hist, i_hist;
    bool uniform = true; bool accumulate = false;
    std::vector<cv::Mat> hsi_planes;
    split(hsi_image, hsi_planes);
    
    cv::calcHist(&hsi_planes[0], 1, 0, mask, h_hist, 1, &h_hist_size_,
                 &h_histRange, uniform, accumulate);
    cv::calcHist(&hsi_planes[1], 1, 0, mask, s_hist, 1, &s_hist_size_,
                 &histRange, uniform, accumulate);
    cv::calcHist(&hsi_planes[2], 1, 0, mask, i_hist, 1, &i_hist_size_,
                 &histRange, uniform, accumulate);
      
    jsk_recognition_msgs::ColorHistogram h_histogram;
    h_histogram.header = header;
    convertHistogramToMsg(h_hist, h_hist_size_, h_histogram);
    h_hist_pub_.publish(h_histogram);
      
    jsk_recognition_msgs::ColorHistogram s_histogram;
    s_histogram.header = header;
    convertHistogramToMsg(s_hist, s_hist_size_, s_histogram);
    s_hist_pub_.publish(s_histogram);
      
    jsk_recognition_msgs::ColorHistogram i_histogram;
    i_histogram.header = header;
    convertHistogramToMsg(i_hist, i_hist_size_, i_histogram);
    i_hist_pub_.publish(i_histogram);
      
  }
    
  void ColorHistogram::extract(
    const sensor_msgs::Image::ConstPtr& image,
    const geometry_msgs::PolygonStamped::ConstPtr& rectangle)
  {
    vital_checker_->poke();
    boost::mutex::scoped_lock lock(mutex_);
    try
    {
      cv_bridge::CvImagePtr cv_ptr;
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      geometry_msgs::Point32 point0 = rectangle->polygon.points[0];
      geometry_msgs::Point32 point1 = rectangle->polygon.points[1];
      int min_x = std::max(0.0f, std::min(point0.x, point1.x));
      int min_y = std::max(0.0f, std::min(point0.y, point1.y));
      int max_x = std::min(std::max(point0.x, point1.x), (float)image->width);
      int max_y = std::min(std::max(point0.y, point1.y), (float)image->height);
      cv::Rect roi(min_x, min_y, max_x - min_x, max_y - min_y);
      cv::Mat bgr_image, roi_image;
      roi_image = cv_ptr->image(roi);
      if (image->encoding == sensor_msgs::image_encodings::RGB8) {
        cv::cvtColor(roi_image, bgr_image, CV_RGB2BGR);
      }
      else {
        roi_image.copyTo(bgr_image);
      }
      image_pub_.publish(cv_bridge::CvImage(
                           image->header,
                           sensor_msgs::image_encodings::BGR8,
                           bgr_image).toImageMsg());
      processBGR(bgr_image, image->header);
      processHSI(bgr_image, image->header);
    }
    catch (cv_bridge::Exception& e)
    {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }

  void ColorHistogram::extractMask(
    const sensor_msgs::Image::ConstPtr& image,
    const sensor_msgs::Image::ConstPtr& mask_image)
  {
    try {
      cv_bridge::CvImagePtr cv_ptr
        = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);
      cv_bridge::CvImagePtr mask_ptr
        = cv_bridge::toCvCopy(mask_image, sensor_msgs::image_encodings::MONO8);
      cv::Mat bgr_image = cv_ptr->image;
      cv::Mat mask_image = mask_ptr->image;
      cv::Mat masked_image;
      bgr_image.copyTo(masked_image, mask_image);
      sensor_msgs::Image::Ptr ros_masked_image
        = cv_bridge::CvImage(image->header,
                             sensor_msgs::image_encodings::BGR8,
                             masked_image).toImageMsg();
      image_pub_.publish(ros_masked_image);
      
      processBGR(bgr_image, mask_image, image->header);
      processHSI(bgr_image, mask_image, image->header);
      
    }
    catch (cv_bridge::Exception& e)
    {
      NODELET_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::ColorHistogram, nodelet::Nodelet);
