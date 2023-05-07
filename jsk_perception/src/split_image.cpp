// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, JSK Lab
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
 * split_image.cpp
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */


#include "jsk_perception/split_image.h"

namespace jsk_perception{

    void SplitImage::onInit(){
        DiagnosticNodelet::always_subscribe_ = true;
        DiagnosticNodelet::onInit();
        pnh_->param("vertical_parts", vertical_parts_, 1);
        pnh_->param("horizontal_parts", horizontal_parts_, 1);
        for(int i=0; i<vertical_parts_; i++){
            for(int j=0; j<horizontal_parts_; j++){
                std::string pub_name = "output/vertical0"
                    + boost::to_string(i)
                    + "/horizontal0"
                    + boost::to_string(j);
                ros::Publisher pub_ = advertise<sensor_msgs::Image>(*pnh_, pub_name, 1);
                pubs_.push_back(pub_);
            }
        }
        onInitPostProcess();
    }

    SplitImage::~SplitImage(){}

    void SplitImage::subscribe(){
        sub_ = pnh_->subscribe("input", 1, &SplitImage::splitImage, this);
    }

    void SplitImage::unsubscribe(){
        sub_.shutdown();
    }

    void SplitImage::splitImage(const sensor_msgs::Image::ConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }catch(cv_bridge::Exception& e){
            NODELET_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat image = cv_ptr->image;
        int height = image.rows;
        int width = image.cols;
        int vertical_size = height / vertical_parts_;
        int horizontal_size = width / horizontal_parts_;
        for(int i=0; i<vertical_parts_; i++){
            for(int j=0; j<horizontal_parts_; j++){
                cv::Mat part_image = image(cv::Rect(horizontal_size*j, vertical_size*i, horizontal_size, vertical_size));
                cv_bridge::CvImagePtr part_cv_ptr(new cv_bridge::CvImage);
                part_cv_ptr->header = msg->header;
                part_cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
                part_cv_ptr->image = part_image;
                sensor_msgs::ImagePtr part_msg = part_cv_ptr->toImageMsg();
                pubs_.at(i*horizontal_parts_+j).publish(part_msg);
            }
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::SplitImage, nodelet::Nodelet);
