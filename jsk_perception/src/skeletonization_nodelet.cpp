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


#include <jsk_perception/skeletonization.h>

namespace jsk_perception
{
    void Skeletonization::onInit()
    {
       DiagnosticNodelet::onInit();
       pnh_->getParam("num_threads", this->num_threads_);
       this->pub_image_ = advertise<sensor_msgs::Image>(
          *pnh_, "image_output", 1);
      onInitPostProcess();
    }
   
    void Skeletonization::subscribe()
    {
       this->sub_ = pnh_->subscribe(
        "input", 1,
        &Skeletonization::imageCallback, this);
    }

    void Skeletonization::unsubscribe()
    {
       NODELET_DEBUG("Unsubscribing from ROS topic.");
       this->sub_.shutdown();
    }

    void Skeletonization::imageCallback(
       const sensor_msgs::Image::ConstPtr& image_msg)
    {
       boost::mutex::scoped_lock lock(this->mutex_);
       cv_bridge::CvImagePtr cv_ptr;
       try {
          cv_ptr = cv_bridge::toCvCopy(
             image_msg, sensor_msgs::image_encodings::MONO8);
       } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
       }
       cv::Mat image = cv_ptr->image;
       this->skeletonization(image);
       cv_bridge::CvImagePtr out_msg(new cv_bridge::CvImage);
       out_msg->header = cv_ptr->header;
       out_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
       out_msg->image = image.clone();
       this->pub_image_.publish(out_msg->toImageMsg());
    }

    void Skeletonization::skeletonization(
       cv::Mat &image)
    {
       if (image.empty()) {
          ROS_ERROR("--CANNOT THIN EMPTY DATA...");
          return;
       }
       if (image.type() == CV_8UC3) {
          cv::cvtColor(image, image, CV_BGR2GRAY);
       }
       cv::Mat img;
       image.convertTo(img, CV_32F, 1/255.0);
       cv::Mat prev = cv::Mat::zeros(img.size(), CV_32F);
       cv::Mat difference;
       do {
          this->iterativeThinning(img, 0);
          this->iterativeThinning(img, 1);
          cv::absdiff(img, prev, difference);
          img.copyTo(prev);
       } while (cv::countNonZero(difference) > 0);
       image = img.clone();
    }

    void Skeletonization::iterativeThinning(
       cv::Mat& img, int iter)
    {
       if (img.empty()) {
          ROS_ERROR("--CANNOT THIN EMPTY DATA...");
          return;
       }
       cv::Mat marker = cv::Mat::zeros(img.size(), CV_32F);
#ifdef _OPENMP
#pragma omp parallel for collapse(2) num_threads(this->num_threads_)
#endif
       for (int i = 1; i < img.rows-1; i++) {
          for (int j = 1; j < img.cols-1; j++) {
             float val[9] = {};
             int icounter = 0;
             for (int y = -1; y <= 1; y++) {
                for (int x = -1; x <= 1; x++) {
                   val[icounter] = img.at<float>(i + y, j + x);
                   icounter++;
                }
             }
             int A = ((val[1] == 0 && val[2] == 1) ? ODD : EVEN)
                + ((val[2] == 0 && val[5] == 1) ? ODD : EVEN)
                + ((val[5] == 0 && val[8] == 1) ? ODD : EVEN)
                + ((val[8] == 0 && val[7] == 1) ? ODD : EVEN)
                + ((val[7] == 0 && val[6] == 1) ? ODD : EVEN)
                + ((val[6] == 0 && val[3] == 1) ? ODD : EVEN)
                + ((val[3] == 0 && val[0] == 1) ? ODD : EVEN)
                + ((val[0] == 0 && val[1] == 1) ? ODD : EVEN);
             int B  = val[0] + val[1] + val[2] + val[3]
                + val[5] + val[6] + val[7] + val[8];
             int m1 = iter == EVEN ? (val[1] * val[5] * val[7])
                : (val[1] * val[3] * val[5]);
             int m2 = iter == EVEN ? (val[3] * val[5] * val[7])
                : (val[1] * val[3] * val[7]);
             if (A == 1 && (B >= 2 && B <= 6) && !m1 && !m2) {
                marker.at<float>(i, j) = sizeof(char);
             }
          }
       }
       cv::bitwise_not(marker, marker);
       cv::bitwise_and(img, marker, img);
    }
}  // jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::Skeletonization, nodelet::Nodelet);
