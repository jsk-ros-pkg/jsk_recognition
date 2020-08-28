// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
/*
 * draw_rects.cpp
 * Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#include <jsk_perception/draw_rects.h>
#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/imgproc_c.h>
#endif

namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception
{

  void DrawRects::onInit()
  {
    ConnectionBasedNodelet::onInit();

    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&DrawRects::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pub_image_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void DrawRects::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_rects_.subscribe(*pnh_, "input/rects", 1);

    if (use_classification_result_)
      sub_class_.subscribe(*pnh_, "input/class", 1);
    else
      sub_rects_.registerCallback(boost::bind(&DrawRects::fillEmptyClasses, this, _1));

    if (use_async_)
    {
      async_ = boost::make_shared<message_filters::Synchronizer<AsyncPolicy> >(queue_size_);
      if (use_classification_result_)
        async_->connectInput(sub_image_, sub_rects_, sub_class_);
      else
        async_->connectInput(sub_image_, sub_rects_, null_class_);
      async_->registerCallback(boost::bind(&DrawRects::onMessage, this, _1, _2, _3));
    } else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
      if (use_classification_result_)
        sync_->connectInput(sub_image_, sub_rects_, sub_class_);
      else
        sync_->connectInput(sub_image_, sub_rects_, null_class_);
      sync_->registerCallback(boost::bind(&DrawRects::onMessage, this, _1, _2, _3));
    }
  }

  void DrawRects::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_rects_.unsubscribe();
    if (use_classification_result_)
      sub_class_.unsubscribe();
  }

  void DrawRects::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    bool need_resubscribe = false;
    if (use_async_ != config.approximate_sync ||
        queue_size_ != config.queue_size ||
        use_classification_result_ != config.use_classification_result)
      need_resubscribe = true;

    use_async_ = config.approximate_sync;
    queue_size_ = config.queue_size;

    use_classification_result_ = config.use_classification_result;
    show_proba_ = config.show_proba;

    rect_boldness_ = config.rect_boldness;

    label_size_ = config.label_size;
    label_boldness_ = config.label_boldness;
    label_font_ = config.label_font;
    label_margin_factor_ = config.label_margin_factor;

    resolution_factor_ = config.resolution_factor;
    interpolation_method_ = config.interpolation_method;
#if ROS_VERSION_MINIMUM(1, 11, 4)
    // indigo or later
    if (need_resubscribe && isSubscribed()) {
      unsubscribe();
      subscribe();
    }
#endif
  }

  void DrawRects::fillEmptyClasses(
      const jsk_recognition_msgs::RectArray::ConstPtr& rects)
  {
    jsk_recognition_msgs::ClassificationResult classes;
    classes.header = rects->header;
    null_class_.add(
      boost::make_shared<jsk_recognition_msgs::ClassificationResult>(classes));
  }

  void DrawRects::onMessage(
      const sensor_msgs::Image::ConstPtr& image,
      const jsk_recognition_msgs::RectArray::ConstPtr& rects,
      const jsk_recognition_msgs::ClassificationResult::ConstPtr& classes)
  {
    boost::mutex::scoped_lock lock(mutex_);

    cv_bridge::CvImage::Ptr cv_img;
    try {
      cv_img = cv_bridge::toCvCopy(image, enc::BGR8);
    } catch (cv_bridge::Exception &e) {
      NODELET_ERROR_STREAM("Failed to convert image: " << e.what());
      return;
    }

    cv::Mat img;
    cv::resize(cv_img->image, img, cv::Size(),
               resolution_factor_, resolution_factor_,
               interpolation_method_);

    int label_num = \
      use_classification_result_ ? classes->target_names.size() : rects->rects.size();

    for (size_t i = 0; i < rects->rects.size(); ++i) {
      int label_idx = use_classification_result_ ? classes->labels[i] : i;
      cv::Scalar color;
      randomColor(label_num, label_idx, color);
      drawRect(img, rects->rects[i], color);

      if (use_classification_result_)
      {
        std::ostringstream oss;
        oss << classes->label_names[i];
        if (show_proba_ && classes->label_proba.size() > i) {
          oss << std::fixed << std::setprecision(2);
          oss << " (" << classes->label_proba[i] << ")";
        }
        drawLabel(img, rects->rects[i], color, oss.str());
      }
    }

    pub_image_.publish(*cv_bridge::CvImage(image->header, enc::BGR8, img).toImageMsg());
  }

  void DrawRects::drawRect(
    cv::Mat& img, const jsk_recognition_msgs::Rect& orig_rect, const cv::Scalar& color)
  {
    cv::Rect rect(orig_rect.x * resolution_factor_,
                    orig_rect.y * resolution_factor_,
                    orig_rect.width * resolution_factor_,
                    orig_rect.height * resolution_factor_);
    cv::rectangle(img, rect, color, rect_boldness_, CV_AA);
  }

  void DrawRects::drawLabel(
    cv::Mat& img, const jsk_recognition_msgs::Rect& rect,
    const cv::Scalar& color, const std::string& label)
  {
    int baseline;
    cv::Size label_size = cv::getTextSize(label, label_font_,
                                          label_size_, label_boldness_,
                                          &baseline);
    int text_color = isDarkColor(color) ? 255 : 0;

    double orig_x = rect.x * resolution_factor_;
    double orig_y = rect.y * resolution_factor_;

    cv::rectangle(img,
                  cv::Rect(orig_x,
                           orig_y - label_size.height * label_margin_factor_ * 1.15,
                           label_size.width * label_margin_factor_,
                           label_size.height * label_margin_factor_ * 1.3),
                  color, -1, CV_AA);
    cv::putText(img, label,
                cv::Point(orig_x + label_size.width  * (label_margin_factor_ - 1.0) / 2.0,
                          orig_y - label_size.height * (label_margin_factor_ - 1.0) / 2.0),
                label_font_,
                label_size_,
                cv::Scalar(text_color, text_color, text_color),
                label_boldness_,
                CV_AA);
  }

  void DrawRects::randomColor(const int& label_num, const int& index, cv::Scalar& color)
  {
    static const float colors[6][3] = { {1,0,1}, {0,0,1},{0,1,1},{0,1,0},{1,1,0},{1,0,0} };
    float ratio = ((float)(index * 123457 % label_num) / label_num) * 5;
    int i = std::floor(ratio);
    int j = std::ceil(ratio);
    ratio -= i;
    for (int c = 0; c < 3; ++c)
      color[c] = (int)(((1-ratio) * colors[i][c] + ratio * colors[j][c]) * 255);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::DrawRects, nodelet::Nodelet);
