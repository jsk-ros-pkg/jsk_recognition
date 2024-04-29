// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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
/*
 * video_to_scene.cpp
 * Author: Kei Okada <k-okada@jsk.t.u-tokyo.ac.jp>
 */

#include "jsk_perception/video_to_scene.h"
#include <boost/assign.hpp>

namespace jsk_perception{
    void VideoToScene::onInit(){
        DiagnosticNodelet::onInit();

        bgsubtractor = cv::bgsegm::createBackgroundSubtractorGMG();
        pnh_->param("min_percent", min_percent_,  5);
        pnh_->param("max_percent", max_percent_, 20);
        captured_ = false;

        srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> > (*pnh_);
        dynamic_reconfigure::Server<Config>::CallbackType f =
            boost::bind(&VideoToScene::configCallback, this, boost::placeholders::_1, boost::placeholders::_2);
        srv_->setCallback (f);

        //pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
        pub_ = advertiseImage(*pnh_, "output", 1);

        onInitPostProcess();
    }

    void VideoToScene::subscribe(){
        std::string transport;
        pnh_->param("image_transport", transport, std::string("raw"));
        NODELET_INFO_STREAM("Using transport \"" << transport << "\"");
        image_transport::TransportHints hints(transport, ros::TransportHints(), *pnh_);
        //
        it_.reset(new image_transport::ImageTransport(*pnh_));
        sub_ = it_->subscribe(pnh_->resolveName("input"), 1, &VideoToScene::work, this, hints);
        ros::V_string names = boost::assign::list_of("input");
        jsk_topic_tools::warnNoRemap(names);
    }

    void VideoToScene::unsubscribe(){
        sub_.shutdown();
    }

    void VideoToScene::work(const sensor_msgs::Image::ConstPtr& image_msg){
        cv::Mat image;

        vital_checker_ -> poke();
        boost::mutex::scoped_lock lock(mutex_);

        image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::RGB8) -> image;
        cv::resize(image, image, cv::Size(), 300.0/image.cols, 300.0/image.cols);

        cv::Mat bgmask;
        bgsubtractor->apply(image, bgmask);
        cv::erode(bgmask, bgmask, cv::Mat(), cv::Point(-1, -1), 2);
        cv::dilate(bgmask, bgmask, cv::Mat(), cv::Point(-1, -1), 2);

        int p = cv::countNonZero(bgmask) / float(bgmask.cols * bgmask.rows) * 100;
        NODELET_DEBUG_STREAM("p = " << p << ", min_percent = " << min_percent_ << ", max_percent = " << max_percent_ << ", captured = " << captured_);

        if ( p < min_percent_ && !captured_ ) {
          captured_ = true;
          pub_.publish(image_msg);
        } else if ( captured_ && p >= max_percent_ ) {
          captured_ = false;
        }
    }

    void VideoToScene::configCallback(Config &config, uint32_t level){
        boost::mutex::scoped_lock lock(mutex_);
        min_percent_ = config.min_percent;
        max_percent_ = config.max_percent;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::VideoToScene, nodelet::Nodelet);
