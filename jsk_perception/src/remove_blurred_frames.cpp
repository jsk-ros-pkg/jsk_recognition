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
 * remove_blurred_frames.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */

#include "jsk_perception/remove_blurred_frames.h"

namespace jsk_perception{
    void RemoveBlurredFrames::onInit(){
        DiagnosticNodelet::onInit();
        srv_ = boost::make_shared<dynamic_reconfigure::Server<Config> > (*pnh_);
        dynamic_reconfigure::Server<Config>::CallbackType f =
            boost::bind(&RemoveBlurredFrames::configCallback, this, boost::placeholders::_1, boost::placeholders::_2);
        srv_->setCallback (f);
        pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
        pub_masked_ = advertise<sensor_msgs::Image>(*pnh_, "output/mask", 1);
        pub_var_ = advertise<std_msgs::Float64>(*pnh_, "output/var", 1);
        onInitPostProcess();
    }

    void RemoveBlurredFrames::subscribe(){
        sub_ = pnh_->subscribe("input", 1, &RemoveBlurredFrames::work, this);
    }

    void RemoveBlurredFrames::unsubscribe(){
        sub_.shutdown();
    }

    void RemoveBlurredFrames::work(const sensor_msgs::Image::ConstPtr& image_msg){
        cv::Mat image, gray, laplacian_image;
        sensor_msgs::Image::Ptr mask_img_msg;
        cv::Scalar mean, stddev;
        std_msgs::Float64 var_msg;
        double var;
        vital_checker_ -> poke();
        boost::mutex::scoped_lock lock(mutex_);
        image = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::RGB8) -> image;
        cv::cvtColor(image, gray, cv::COLOR_RGB2GRAY);
        cv::Laplacian(gray, laplacian_image, CV_64F);
        cv::meanStdDev(laplacian_image, mean, stddev, cv::Mat());
        var = stddev.val[0] * stddev.val[0];
        NODELET_DEBUG("%s : Checking variance of the Laplacian: actual %f, threshold %f\n", __func__, var, min_laplacian_var_);
        cv::Mat masked_image = cv::Mat::zeros(laplacian_image.rows, laplacian_image.cols, CV_8UC1);
        laplacian_image.convertTo(laplacian_image, CV_8UC1, 1.0);
        cv::threshold(cv::abs(laplacian_image), masked_image, threshold_, 255, cv::THRESH_BINARY);
        mask_img_msg = cv_bridge::CvImage(image_msg->header, sensor_msgs::image_encodings::MONO8, masked_image).toImageMsg();
        var_msg.data = var;
        pub_masked_.publish(mask_img_msg);
        pub_var_.publish(var_msg);
        if(var > min_laplacian_var_){
            pub_.publish(image_msg);
        }
    }

    void RemoveBlurredFrames::configCallback(Config &config, uint32_t level){
        boost::mutex::scoped_lock lock(mutex_);
        min_laplacian_var_ = config.min_laplacian_var;
        threshold_ = config.threshold;
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::RemoveBlurredFrames, nodelet::Nodelet);
