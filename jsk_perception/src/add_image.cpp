// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) JSK, 2016 Lab
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
 *     disclaimer in the documentation and/or other materials provided
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

#include <jsk_perception/add_image.h>
#include <boost/assign.hpp>
#include <jsk_topic_tools/log_utils.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_utils/cv_utils.h>

namespace jsk_perception {
  void AddImage::onInit() {
    DiagnosticNodelet::onInit();

    ////////////////////////////////////////////////////////
    // Dynamic Reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&AddImage::configCallback, this, _1, _2);
    srv_->setCallback(f);

    pnh_->param("approximate_sync", approximate_sync_, false);
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void AddImage::subscribe() {
    sub_image1_.subscribe(*pnh_, "input/src1", 1);
    sub_image2_.subscribe(*pnh_, "input/src2", 1);
    if (approximate_sync_) {
      async_ =
        boost::make_shared<message_filters::Synchronizer<ApproxSyncPolicy> >(100);
      async_->connectInput(sub_image1_, sub_image2_);
      async_->registerCallback(boost::bind(&AddImage::add, this, _1, _2));
    } else {
      sync_ =
        boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_image1_, sub_image2_);
      sync_->registerCallback(boost::bind(&AddImage::add, this, _1, _2));
    }
    ros::V_string names = boost::assign::list_of("~input/src1")("~input/src2");
    jsk_topic_tools::warnNoRemap(names);
  }

  void AddImage::unsubscribe() {
    sub_image1_.unsubscribe();
    sub_image2_.unsubscribe();
  }

  void AddImage::configCallback(Config& config, uint32_t level) {
    boost::mutex::scoped_lock lock(mutex_);
    alpha_ = config.alpha;
    beta_ = config.beta;
    gamma_ = config.gamma;
  }

  void AddImage::add(const sensor_msgs::Image::ConstPtr& image_msg1,
                     const sensor_msgs::Image::ConstPtr& image_msg2) {
    cv::Mat image1 =
      cv_bridge::toCvShare(image_msg1, image_msg1->encoding)->image;
    cv::Mat image2 =
      cv_bridge::toCvShare(image_msg2, image_msg2->encoding)->image;
    cv::Mat result_image;
    cv::addWeighted(image1, alpha_, image2, beta_, gamma_, result_image);
    pub_.publish(cv_bridge::CvImage(image_msg1->header, image_msg1->encoding,
                                    result_image).toImageMsg());
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::AddImage, nodelet::Nodelet);
