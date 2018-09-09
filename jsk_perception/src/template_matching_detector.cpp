// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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


#include "jsk_perception/template_matching_detector.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void TemplateMatchingDetector::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("queue_size", queue_size_, 100);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&TemplateMatchingDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);

    img_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output/viz", 1);
    rects_pub_ = advertise<jsk_recognition_msgs::RectArray>(*pnh_, "output/rects", 1);

#ifdef USE_CUDA
    if(cv::cuda::getCudaEnabledDeviceCount() > 0){
      use_cuda = true;
      cv::cuda::DeviceInfo info(0);
      ROS_INFO("GPU FOUND : %s", info.name());
    } else {
      use_cuda = false;
      ROS_INFO("NO GPU FOUND : RUN CPU MODE");
    }
#else
      use_cuda = false;
      ROS_INFO("RUN CPU MODE");
#endif

    onInitPostProcess();
  }

  void TemplateMatchingDetector::subscribe()
  {
    sub_ = pnh_->subscribe("input", queue_size_, &TemplateMatchingDetector::apply, this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void TemplateMatchingDetector::unsubscribe()
  {
    sub_.shutdown();
  }

  void TemplateMatchingDetector::sortRects(
    std::vector<cv::Rect> rects)
  {
    switch(config_.sort_direction){
    case jsk_perception::TemplateMatchingDetector_NoOperation:
      break;
    case jsk_perception::TemplateMatchingDetector_Horizontal:
      std::sort(rects.begin(), rects.end(), rect_horizontal<cv::Rect>);
      break;
    case jsk_perception::TemplateMatchingDetector_Vertical:
      std::sort(rects.begin(), rects.end(), rect_vertical<cv::Rect>);
      break;
    }
  }

  void TemplateMatchingDetector::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    config_ = config;
    min_scale = config.min_scale;
    max_scale = config.max_scale;
    resize_template_num = config.resize_template_num;
    target_num = config.target_num;
    update_matching_threshold_ = config.update_matching_threshold;
    check_flipped_image_ = config.check_flipped_image;
    if (need_config_update_) {
      config.matching_threshold = matching_threshold;
      need_config_update_ = false;
    } else {
      matching_threshold = config_.matching_threshold;
    }
    if (config.template_filename != template_filename_) {
      template_filename_ = config.template_filename;
      template_image = cv::imread(template_filename_, 0);
      if (template_image.empty()) {
        ROS_ERROR("Cannot read template image!");
      }
    }
  }


  void TemplateMatchingDetector::apply(
    const sensor_msgs::Image::ConstPtr& image_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    int i, j, k, l;

    if ((image_msg->width == 0) && (image_msg->height == 0)) {
      ROS_WARN("invalid image input");
      return;
    }
    if(template_image.empty()){
      return;
    }

    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image_msg, "mono8");
    cv::Mat search_img = image_ptr->image;
    cv::Mat result_img = search_img.clone();
    cv::cvtColor(result_img, result_img, CV_GRAY2BGR);
    std::vector<cv::Rect> rects;
    std::vector<float> scores;

#ifdef USE_CUDA
    if(use_cuda){
      cv::cuda::GpuMat cuda_search_img(search_img);
      cv::cuda::GpuMat cuda_resized_template;
      cv::cuda::GpuMat cuda_score_img;
      cv::cuda::GpuMat cuda_flipped_template;
      cv::cuda::GpuMat cuda_flip_resized_template;
      cv::cuda::GpuMat cuda_flip_score_img;
      cv::Mat score_img;
      cv::Mat flip_score_img;
      cv::cuda::Stream stream[2];
      stream[0].waitForCompletion(); // normal template
      stream[1].waitForCompletion(); // flipped template

      cuda_template_image.upload(template_image, stream[0]);
      if(check_flipped_image_)
        cv::cuda::flip(cuda_template_image, cuda_flipped_template, 1, stream[0]);

      //template matching
      for(k = 0; k <= resize_template_num; k++){
        double scale = min_scale + (max_scale - min_scale) * (double)k / resize_template_num;
        cv::cuda::resize(cuda_template_image, cuda_resized_template, cv::Size(), scale, scale, CV_INTER_LINEAR, stream[0]);
        if(check_flipped_image_)
          cv::cuda::resize(cuda_flipped_template, cuda_flip_resized_template, cv::Size(), scale, scale, CV_INTER_LINEAR, stream[1]);
        if(cuda_search_img.size().width <= cuda_resized_template.size().width ||
           cuda_search_img.size().height <= cuda_resized_template.size().height){
          break;
        }
        cv::Ptr<cv::cuda::TemplateMatching> alg = cv::cuda::createTemplateMatching(cuda_search_img.type(), CV_TM_CCOEFF_NORMED);
        alg->match(cuda_search_img, cuda_resized_template, cuda_score_img, stream[0]);
        cv::cuda::threshold(cuda_score_img, cuda_score_img, matching_threshold, 1.0, CV_THRESH_TOZERO, stream[0]);
        cuda_score_img.download(score_img, stream[0]);
        if(check_flipped_image_){
          cv::Ptr<cv::cuda::TemplateMatching> alg_flip = cv::cuda::createTemplateMatching(cuda_search_img.type(), CV_TM_CCOEFF_NORMED);
          alg_flip->match(cuda_search_img, cuda_flip_resized_template, cuda_flip_score_img, stream[1]);
          cv::cuda::threshold(cuda_flip_score_img, cuda_flip_score_img, matching_threshold, 1.0, CV_THRESH_TOZERO, stream[1]);
          cuda_flip_score_img.download(flip_score_img, stream[1]);
        }
        for (i = 0; i < score_img.rows; i++) {
          for (j = 0; j < score_img.cols; j++) {
            if (score_img.at<float>(i, j) > 0.0) {
              rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(cuda_resized_template.cols, cuda_resized_template.rows)));
              scores.push_back(score_img.at<float>(i, j));
            }
            if(check_flipped_image_){
              if (flip_score_img.at<float>(i, j) > 0.0) {
                rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(cuda_flip_resized_template.cols, cuda_flip_resized_template.rows)));
                scores.push_back(flip_score_img.at<float>(i, j));
              }
            }
          }
        }
      }
      search_img.release();
    } else {
      cv::Mat score_img;
      cv::Mat resized_template;
      int matching_try_num;

      if(check_flipped_image_){
        matching_try_num = 2;
      } else {
        matching_try_num = 1;
      }

      //template matching
      for(l = 0; l < matching_try_num; l++){
        for(k = 0; k <= resize_template_num; k++){
          double scale = min_scale + (max_scale - min_scale) * (double)k / resize_template_num;
          cv::resize(template_image, resized_template, cv::Size(), scale, scale);
          if(l == 1){
            cv::flip(resized_template, resized_template, 1);
          }
          if(search_img.size().width <= resized_template.size().width ||
             search_img.size().height <= resized_template.size().height){
            break;
          }
          cv::matchTemplate(search_img, resized_template, score_img, CV_TM_CCOEFF_NORMED);

          cv::threshold(score_img, score_img, matching_threshold, 1.0, CV_THRESH_TOZERO);
          for (i = 0; i < score_img.rows; i++) {
            for (j = 0; j < score_img.cols; j++) {
              if (score_img.at<float>(i, j) > 0.0) {
                rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(resized_template.cols, resized_template.rows)));
                scores.push_back(score_img.at<float>(i, j));
              }
            }
          }
        }
      }
      search_img.release();
    }
#else
    cv::Mat score_img;
    cv::Mat resized_template;
    int matching_try_num;

    if(check_flipped_image_){
      matching_try_num = 2;
    } else {
      matching_try_num = 1;
    }

    //template matching
    for(l = 0; l < matching_try_num; l++){
      for(k = 0; k <= resize_template_num; k++){
        double scale = min_scale + (max_scale - min_scale) * (double)k / resize_template_num;
        cv::resize(template_image, resized_template, cv::Size(), scale, scale);
        if(l == 1){
          cv::flip(resized_template, resized_template, 1);
        }
        if(search_img.size().width <= resized_template.size().width ||
           search_img.size().height <= resized_template.size().height){
          break;
        }
        cv::matchTemplate(search_img, resized_template, score_img, CV_TM_CCOEFF_NORMED);

        cv::threshold(score_img, score_img, matching_threshold, 1.0, CV_THRESH_TOZERO);
        for (i = 0; i < score_img.rows; i++) {
          for (j = 0; j < score_img.cols; j++) {
            if (score_img.at<float>(i, j) > 0.0) {
              rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(resized_template.cols, resized_template.rows)));
              scores.push_back(score_img.at<float>(i, j));
            }
          }
        }
      }
    }
    search_img.release();
#endif
    // integrate neighborhood rectangles
    std::vector<int> result_indices;
    cv::dnn::NMSBoxes(rects, scores, matching_threshold, config_.nms_threshold, result_indices);
    std::vector<cv::Rect> result_rects(result_indices.size());
    for (i = 0; i < result_indices.size(); ++i) {
      result_rects[i] = rects[result_indices[i]];
    }
    sortRects(result_rects);

    //update matching threshold
    if(update_matching_threshold_) {
      if (result_rects.size() > target_num) {
        matching_threshold = std::min(matching_threshold + config_.update_step_of_threshold, 1.0);
        need_config_update_ = true;
      }
      else if (result_rects.size() < target_num) {
        matching_threshold = std::max(matching_threshold - config_.update_step_of_threshold, 0.0);
        need_config_update_ = true;
      }
    }

    if (img_pub_.getNumSubscribers() > 0) {
      //draw rectangles on image
      for (i = 0;i < result_rects.size(); i++) {
        cv::rectangle(result_img,
                      result_rects.at(i).tl(),
                      result_rects.at(i).br(),
                      cv::Scalar(0, 0, 255), 2, 8, 0);
      }
      img_pub_.publish(cv_bridge::CvImage(image_msg->header,
                                          sensor_msgs::image_encodings::BGR8,
                                          result_img).toImageMsg());
    }

    if (rects_pub_.getNumSubscribers() > 0) {
      jsk_recognition_msgs::RectArray rects_msg;
      rects_msg.rects.resize(result_rects.size());
      for(i = 0; i < result_rects.size(); i++){
          jsk_recognition_msgs::Rect rect;
          rect.x = result_rects[i].x;
          rect.y = result_rects[i].y;
          rect.width = result_rects[i].width;
          rect.height = result_rects[i].height;
          rects_msg.rects[i] = rect;
      }

      rects_msg.header = image_msg->header;
      rects_pub_.publish(rects_msg);
    }
  }
bool TemplateMatchingDetector::need_config_update_ = false;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::TemplateMatchingDetector, nodelet::Nodelet);
