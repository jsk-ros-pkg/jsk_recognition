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


#include "jsk_perception/template_match_detector.h"
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

namespace jsk_perception
{
  void TemplateMatchDetector::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("queue_size", queue_size_, 100);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&TemplateMatchDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);

    img_pub_ = advertise<sensor_msgs::Image>(*pnh_, "output/viz", 1);
    rects_pub_ = advertise<jsk_recognition_msgs::RectArray>(*pnh_, "output/rect", 1);

    stored_thre = 0.0;
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

  void TemplateMatchDetector::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_info_.subscribe(*pnh_, "input/info", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(sub_image_, sub_info_);
      async_->registerCallback(boost::bind(&TemplateMatchDetector::apply, this, _1, _2));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      sync_->connectInput(sub_image_, sub_info_);
      sync_->registerCallback(boost::bind(&TemplateMatchDetector::apply, this, _1, _2));
    }

    ros::V_string names = boost::assign::list_of("~input")("~input/info");
    jsk_topic_tools::warnNoRemap(names);
  }

  void TemplateMatchDetector::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_info_.unsubscribe();
  }

  bool TemplateMatchDetector::is_overlaped(
   cv::Rect rect1,
   cv::Rect rect2)
  {
    double overlap_thre = 0.5;
    if(
       (double)(rect1 & rect2).area() / rect1.area() > overlap_thre ||
       (double)(rect1 & rect2).area() / rect2.area() > overlap_thre)
      return true;
    else
      return false;
  }

  void TemplateMatchDetector::rectangleIntegration(
   std::vector<cv::Rect> rects,
   std::vector<double> score,
   std::vector<cv::Rect>& result)
  {
    int i, j, k;

    //clustering
    int found_cluster = 0;
    std::vector<std::vector<cv::Rect> > rects_cluster;
    std::vector<std::vector<double> > score_cluster;
    std::vector<cv::Rect> tmp_rects;
    std::vector<double> tmp_scores;
    tmp_rects.push_back(rects.at(0));
    rects_cluster.push_back(tmp_rects);
    rects_cluster.at(0).reserve(rects.size());
    tmp_scores.push_back(score.at(0));
    score_cluster.push_back(tmp_scores);
    score_cluster.at(0).reserve(rects.size());
    for(i = 1; i < rects.size(); i++){
      for(j = 0; j < rects_cluster.size(); j++){
        for(k = 0; k < rects_cluster.at(j).size(); k++){
          if(is_overlaped(rects.at(i), rects_cluster.at(j).at(k))){
            rects_cluster.at(j).push_back(rects.at(i));
            score_cluster.at(j).push_back(score.at(i));
            break;
          }
        }
        if(k < rects_cluster.at(j).size())
          break;
      }
      if(j == rects_cluster.size() &&
         k == rects_cluster.at(rects_cluster.size() - 1).size())
        {
          tmp_rects.clear();
          tmp_rects.push_back(rects.at(i));
          rects_cluster.push_back(tmp_rects);
          rects_cluster.at(rects_cluster.size() - 1).reserve(rects.size());

          tmp_scores.clear();
          tmp_scores.push_back(score.at(i));
          score_cluster.push_back(tmp_scores);
          score_cluster.at(rects_cluster.size() - 1).reserve(rects.size());
        }
    }

    //integrate and sort result
    result.resize(rects_cluster.size());
    for(i = 0; i < result.size(); i++){
      cv::Rect tmp_rect(0.0, 0.0, 0.0, 0.0);
      double score_total = 0.0;
      for(j = 0; j < rects_cluster.at(i).size(); j++){
        tmp_rect += score_cluster.at(i).at(j) * rects_cluster.at(i).at(j).tl();
        tmp_rect += cv::Size(score_cluster.at(i).at(j) * rects_cluster.at(i).at(j).width,
                             score_cluster.at(i).at(j) * rects_cluster.at(i).at(j).height);
        score_total += score_cluster.at(i).at(j);
      }
      tmp_rect.x /= score_total;
      tmp_rect.y /= score_total;
      tmp_rect.width /= score_total;
      tmp_rect.height /= score_total;
      result.at(i) = tmp_rect;
    }
    switch(sort_op){
    case 0:
      std::sort(result.begin(), result.end(), rect_horizontal<cv::Rect>);
      break;
    case 1:
      std::sort(result.begin(), result.end(), rect_vertical<cv::Rect>);
      break;
    default :
      std::cout << "sort_option error!! sort_op should be 0 or 1." << std::endl;
      break;
    }
  }


  void TemplateMatchDetector::configCallback(
    Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    min_scale = config.min_scale;
    max_scale = config.max_scale;
    resize_num = config.resize_num;
    target_num = config.target_num;
    max_detect_num = config.max_detect_num;
    sort_op = config.sort_op;
    debug_ = config.debug;
    specify_target_ = config.specify_target;
    flip_template_ = config.flip_template;
    if(config.template_name != template_name_)
      {
        template_name_ = config.template_name;
        std::cout << "template : " << template_name_ << std::endl;
        tmpl_img = cv::imread(template_name_, 0);
        if(tmpl_img.empty()){
          ROS_WARN("Cannot read template image!");
        }
      }
    if(config.matching_thre != stored_thre)
      {
        stored_thre = config.matching_thre;
        matching_thre = stored_thre;
      }
  }


  void TemplateMatchDetector::apply(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    int i, j, k, l;
    double thre_update_step = 0.05;

    if ((image_msg->width == 0) && (image_msg->height == 0)) {
        ROS_WARN("invalid image input");
        return;
    }
    if(tmpl_img.empty()){
      return;
    }

    cv_bridge::CvImagePtr image_ptr;
    image_ptr = cv_bridge::toCvCopy(image_msg, "mono8");
    cv::Mat search_img = image_ptr->image;
    cv::Mat result_img = search_img.clone();
    cv::cvtColor(result_img, result_img, CV_GRAY2BGR);
    std::vector<cv::Rect> rects;
    std::vector<double> score;

#ifdef USE_CUDA
    if(use_cuda){
      cv::cuda::GpuMat cuda_search_img(search_img);
      cv::cuda::GpuMat cuda_resized_tmpl;
      cv::cuda::GpuMat cuda_score_img;
      cv::cuda::GpuMat cuda_flipped_tmpl;
      cv::cuda::GpuMat cuda_flip_resized_tmpl;
      cv::cuda::GpuMat cuda_flip_score_img;
      cv::Mat score_img;
      cv::Mat flip_score_img;
      cv::cuda::Stream stream[2];
      stream[0].waitForCompletion(); // normal template
      stream[1].waitForCompletion(); // flipped template

      cuda_tmpl_img.upload(tmpl_img, stream[0]);
      if(flip_template_)
        cv::cuda::flip(cuda_tmpl_img, cuda_flipped_tmpl, 1, stream[0]);

      //template matching
      for(k = 0; k <= resize_num; k++){
        double scale = min_scale + (max_scale - min_scale) * (double)k / resize_num;
        if(debug_) std::cout << "scale : " << scale;
        cv::cuda::resize(cuda_tmpl_img, cuda_resized_tmpl, cv::Size(), scale, scale, CV_INTER_LINEAR, stream[0]);
        if(flip_template_)
          cv::cuda::resize(cuda_flipped_tmpl, cuda_flip_resized_tmpl, cv::Size(), scale, scale, CV_INTER_LINEAR, stream[1]);
        if(cuda_search_img.size().width <= cuda_resized_tmpl.size().width ||
           cuda_search_img.size().height <= cuda_resized_tmpl.size().height){
          break;
        }
        cv::Ptr<cv::cuda::TemplateMatching> alg = cv::cuda::createTemplateMatching(cuda_search_img.type(), CV_TM_CCOEFF_NORMED);
        alg->match(cuda_search_img, cuda_resized_tmpl, cuda_score_img, stream[0]);
        cv::cuda::threshold(cuda_score_img, cuda_score_img, matching_thre, 1.0, CV_THRESH_TOZERO, stream[0]);
        cuda_score_img.download(score_img, stream[0]);
        if(flip_template_){
          cv::Ptr<cv::cuda::TemplateMatching> alg_flip = cv::cuda::createTemplateMatching(cuda_search_img.type(), CV_TM_CCOEFF_NORMED);
          alg_flip->match(cuda_search_img, cuda_flip_resized_tmpl, cuda_flip_score_img, stream[1]);
          cv::cuda::threshold(cuda_flip_score_img, cuda_flip_score_img, matching_thre, 1.0, CV_THRESH_TOZERO, stream[1]);
          cuda_flip_score_img.download(flip_score_img, stream[1]);
        }
        int debug_count = 0;
        for (i = 0; i < score_img.rows; i++) {
          for (j = 0; j < score_img.cols; j++) {
            if (score_img.at<float>(i, j) > 0.0) {
              debug_count++;
              rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(cuda_resized_tmpl.cols, cuda_resized_tmpl.rows)));
              score.push_back(score_img.at<float>(i, j));
              if(debug_)
                cv::rectangle(result_img, cv::Point(j, i),
                              cv::Point(j + cuda_resized_tmpl.cols, i + cuda_resized_tmpl.rows), cv::Scalar(0, 0, 0));
            }
            if(flip_template_){
              if (flip_score_img.at<float>(i, j) > 0.0) {
                debug_count++;
                rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(cuda_flip_resized_tmpl.cols, cuda_flip_resized_tmpl.rows)));
                score.push_back(flip_score_img.at<float>(i, j));
                if(debug_)
                  cv::rectangle(result_img, cv::Point(j, i),
                                cv::Point(j + cuda_flip_resized_tmpl.cols, i + cuda_flip_resized_tmpl.rows), cv::Scalar(0, 0, 0));
              }
            }
          }
        }
        if(debug_) std::cout << "  match :" << debug_count << std::endl;
      }
      search_img.release();
    } else {
      cv::Mat score_img;
      cv::Mat resized_tmpl;
      int matching_try_num;

      if(flip_template_){
        matching_try_num = 2;
      } else {
        matching_try_num = 1;
      }

      //template matching
      for(l = 0; l < matching_try_num; l++){
        for(k = 0; k <= resize_num; k++){
          double scale = min_scale + (max_scale - min_scale) * (double)k / resize_num;
          if(debug_) std::cout << "scale : " << scale;
          cv::resize(tmpl_img, resized_tmpl, cv::Size(), scale, scale);
          if(l == 1){
            cv::flip(resized_tmpl, resized_tmpl, 1);
          }
          if(search_img.size().width <= resized_tmpl.size().width ||
             search_img.size().height <= resized_tmpl.size().height){
            break;
          }
          cv::matchTemplate(search_img, resized_tmpl, score_img, CV_TM_CCOEFF_NORMED);

          cv::threshold(score_img, score_img, matching_thre, 1.0, CV_THRESH_TOZERO);
          int debug_count = 0;
          for (i = 0; i < score_img.rows; i++) {
            for (j = 0; j < score_img.cols; j++) {
              if (score_img.at<float>(i, j) > 0.0) {
                debug_count++;
                rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(resized_tmpl.cols, resized_tmpl.rows)));
                score.push_back(score_img.at<float>(i, j));
                if(debug_)
                  cv::rectangle(result_img, cv::Point(j, i), cv::Point(j + resized_tmpl.cols, i + resized_tmpl.rows), cv::Scalar(0, 0, 0));
              }
            }
          }
          if(debug_) std::cout << "  match :" << debug_count << std::endl;
        }
      }
      search_img.release();
    }
#else
    cv::Mat score_img;
    cv::Mat resized_tmpl;
    int matching_try_num;

    if(flip_template_){
      matching_try_num = 2;
    } else {
      matching_try_num = 1;
    }

    //template matching
    for(l = 0; l < matching_try_num; l++){
      for(k = 0; k <= resize_num; k++){
        double scale = min_scale + (max_scale - min_scale) * (double)k / resize_num;
        if(debug_) std::cout << "scale : " << scale;
        cv::resize(tmpl_img, resized_tmpl, cv::Size(), scale, scale);
        if(l == 1){
          cv::flip(resized_tmpl, resized_tmpl, 1);
        }
        if(search_img.size().width <= resized_tmpl.size().width ||
           search_img.size().height <= resized_tmpl.size().height){
          break;
        }
        cv::matchTemplate(search_img, resized_tmpl, score_img, CV_TM_CCOEFF_NORMED);

        cv::threshold(score_img, score_img, matching_thre, 1.0, CV_THRESH_TOZERO);
        int debug_count = 0;
        for (i = 0; i < score_img.rows; i++) {
          for (j = 0; j < score_img.cols; j++) {
            if (score_img.at<float>(i, j) > 0.0) {
              debug_count++;
              rects.push_back(cv::Rect(cv::Point(j, i), cv::Size(resized_tmpl.cols, resized_tmpl.rows)));
              score.push_back(score_img.at<float>(i, j));
              if(debug_)
                cv::rectangle(result_img, cv::Point(j, i), cv::Point(j + resized_tmpl.cols, i + resized_tmpl.rows), cv::Scalar(0, 0, 0));
            }
          }
        }
        if(debug_) std::cout << "  match :" << debug_count << std::endl;
      }
    }
    search_img.release();
#endif

    if(rects.size() == 0){
      if(specify_target_){
        std::cout << "no match!" << std::endl << "Decreasing threshold before : " << matching_thre;
        matching_thre = std::max(matching_thre - thre_update_step, 0.0);
        std::cout << "  after : " << matching_thre << std::endl;
        return;
      } else {
        if(debug_)
          std::cout << std::endl << std::endl;
        jsk_recognition_msgs::RectArray rects_msg;
        rects_msg.header = image_msg->header;
        rects_pub_.publish(rects_msg);
        img_pub_.publish(cv_bridge::CvImage(
                                            image_msg->header,
                                            sensor_msgs::image_encodings::BGR8,
                                            result_img).toImageMsg());
        return;
      }
    }

    if(rects.size() > max_detect_num){
      std::cout << "detected rectangles num is over maxdetected num. detected num : " << rects.size() << "  max num : "
                << max_detect_num << std::endl;
      if(specify_target_){
        std::cout << "Increasing threshold before : " << matching_thre;
        matching_thre = std::min(matching_thre + thre_update_step, 1.0);
        std::cout << "  after : " << matching_thre << std::endl;
        return;
      } else {
        // todo clear rects
        return;
      }
    }

    // integrate neighborhood rectangles
    std::vector<cv::Rect> result_rects;
    TemplateMatchDetector::rectangleIntegration(rects, score, result_rects);
    if(debug_)
      std::cout << "integrated_rect num : " << result_rects.size() << std::endl;

    //update matching threshold
    if(specify_target_ &&
       result_rects.size() != target_num){
      if(result_rects.size() > target_num){
        std::cout << "Too many rectangles detected. detected num : " << result_rects.size() << "  target num : "
                  << target_num << std::endl << "Increasing threshold before : " << matching_thre;
        matching_thre = std::min(matching_thre + thre_update_step, 1.0);
        std::cout << "  after : " << matching_thre << std::endl;
      }
      else {
        std::cout << "Too few rectangles detected. detected num : " << result_rects.size() << "  target num : "
                  << target_num << std::endl << "Decreasing threshold before : " << matching_thre;
        matching_thre = std::max(matching_thre - thre_update_step, 0.0);
        std::cout << "  after : " << matching_thre << std::endl;
      }
      return;
    }
    if(debug_) std::cout << std::endl;

    //draw rectangles on image
    for (i = 0;i < result_rects.size(); i++) {
      cv::rectangle(result_img, result_rects.at(i).tl(), result_rects.at(i).br(), cv::Scalar(0, 0, 255), 2, 8, 0);
    }

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
    rects_msg.rects.resize(result_rects.size());
    rects_pub_.publish(rects_msg);
    img_pub_.publish(cv_bridge::CvImage(
                     image_msg->header,
                     sensor_msgs::image_encodings::BGR8,
                     result_img).toImageMsg());
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::TemplateMatchDetector, nodelet::Nodelet);
