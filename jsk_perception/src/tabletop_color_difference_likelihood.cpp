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

#define BOOST_PARAMETER_MAX_ARITY 7
#include <Eigen/Geometry>
#include <boost/assign.hpp>
#include <jsk_recognition_utils/pcl_conversion_util.h>
#include "jsk_perception/tabletop_color_difference_likelihood.h"
#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_utils/sensor_model_utils.h>
#include <jsk_recognition_utils/color_utils.h>
#include <jsk_topic_tools/color_utils.h>
#include <jsk_topic_tools/log_utils.h>
#include <jsk_recognition_utils/cv_utils.h>
#include <sstream>

namespace jsk_perception
{
  void TabletopColorDifferenceLikelihood::onInit()
  {
    DiagnosticNodelet::onInit();
    pnh_->param("tf_queue_size", tf_queue_size_, 10);
    pnh_->param("cyclic_value", cyclic_value_, true);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &TabletopColorDifferenceLikelihood::configCallback, this, _1, _2);
    srv_->setCallback (f);
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pub_ = advertise<sensor_msgs::Image>(*pnh_, "output", 1);
    pub_debug_histogram_image_ = advertise<sensor_msgs::Image>(*pnh_, "debug/histogram_image", 1);
    pub_debug_polygon_ = advertise<sensor_msgs::Image>(*pnh_, "debug/polygon_image", 1);
    onInitPostProcess();
  }

  void TabletopColorDifferenceLikelihood::subscribe()
  {
    sub_info_ = pnh_->subscribe("input/camera_info", 1, 
                                &TabletopColorDifferenceLikelihood::infoCallback, this);
    sub_polygons_ = pnh_->subscribe("input/polygons", 1, 
                                    &TabletopColorDifferenceLikelihood::polygonCallback, this);
    sub_image_.subscribe(*pnh_, "input", 1);
    ros::V_string names = boost::assign::list_of("~input")("~input/camera_info")("~input/polygons");
    jsk_topic_tools::warnNoRemap(names);
  }

  void TabletopColorDifferenceLikelihood::unsubscribe()
  {
    sub_info_.shutdown();
    sub_polygons_.shutdown();
    sub_image_.unsubscribe();
  }

  void TabletopColorDifferenceLikelihood::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_info_msg_ = msg;
  }

  void TabletopColorDifferenceLikelihood::polygonCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    latest_polygon_msg_ = msg;
    if (!tf_filter_) {
      // If tf_filter_ is not initialized yet,
      // we initialize tf_filter_ with frame_id
      // of the message.
      tf_filter_.reset(new tf::MessageFilter<sensor_msgs::Image>(sub_image_,
                                                                 *tf_listener_,
                                                                 msg->header.frame_id,
                                                                 tf_queue_size_));
      tf_filter_->registerCallback(boost::bind(&TabletopColorDifferenceLikelihood::imageCallback, this, _1));
    }
  }

  void TabletopColorDifferenceLikelihood::debugPolygonImage(const jsk_recognition_utils::CameraDepthSensor& model,
                                                            cv::Mat& image,
                                                            jsk_recognition_utils::Polygon::Ptr polygon,
                                                            size_t pi) const
  {
    polygon->drawLineToImage(model, image,
                             jsk_recognition_utils::colorROSToCVRGB(jsk_topic_tools::colorCategory20(pi)));
    if (polygon->centroid()[2] > 0) {
      std::stringstream ss;
      ss << pi;
      cv::putText(image, ss.str(), 
                  jsk_recognition_utils::project3DPointToPixel(model.getPinholeCameraModel(), polygon->centroid()), 
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.5,
                  jsk_recognition_utils::colorROSToCVRGB(jsk_topic_tools::colorCategory20(pi)));
    }
  }

  void TabletopColorDifferenceLikelihood::imageCallback(const sensor_msgs::Image::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    // check camera info and polygon is available
    if (!latest_info_msg_) {
      ROS_WARN("no camera info is available yet");
      return;
    }
    if (!latest_polygon_msg_) {
      ROS_WARN("no polygon is available yet");
      return;
    }

    // try-catch for tf exception
    try 
    {
      cv_bridge::CvImagePtr input_cv_image 
        = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat input_image = input_cv_image->image;
      // Transform PolygonArray to the frame of image
      tf::StampedTransform transform;
      tf_listener_->lookupTransform(msg->header.frame_id,
                                    latest_polygon_msg_->header.frame_id,
                                    msg->header.stamp,
                                    transform);
      Eigen::Affine3f polygon_transform;
      tf::transformTFToEigen(transform, polygon_transform);
      
      // convert to polygon
      std::vector<jsk_recognition_utils::Polygon::Ptr> polygons 
        = jsk_recognition_utils::Polygon::fromROSMsg(*latest_polygon_msg_, polygon_transform);
      jsk_recognition_utils::CameraDepthSensor model;
      model.setCameraInfo(*latest_info_msg_);
      cv::Mat image = model.image(CV_8UC1);
      // debug_polygon_image is an image for debug.
      cv::Mat debug_polygon_image = model.image(CV_8UC3);
      // prepare for histogram image
      cv::Mat histogram_image;
      for (size_t pi = 0; pi < polygons.size(); pi++) {
        jsk_recognition_utils::Polygon::Ptr polygon = polygons[pi];
        cv::Mat mask_image;
        polygon->maskImage(model, mask_image);
        debugPolygonImage(model, debug_polygon_image, polygon, pi);
        // compute histogram of input with mask_image
        cv::MatND hist = jsk_recognition_utils::computeHistogram(input_image, bin_size_,
                                                                 pixel_min_value_, pixel_max_value_,
                                                                 mask_image);
        std::vector<jsk_recognition_msgs::HistogramWithRangeBin> histogram_bins
          = jsk_recognition_utils::cvMatNDToHistogramWithRangeBinArray(hist, pixel_min_value_, pixel_max_value_);
        jsk_recognition_utils::sortHistogramWithRangeBinArray(histogram_bins);
        std::vector<jsk_recognition_msgs::HistogramWithRangeBin> top_n_histogram_bins
          = jsk_recognition_utils::topNHistogramWithRangeBins(histogram_bins, histogram_top_n_ratio_);
        cv::Mat one_histogram_image = cv::Mat(30, 180, CV_8UC3);
        one_histogram_image = cv::Scalar::all(255);
        for (size_t j = 0; j < histogram_bins.size(); j++) {
          jsk_recognition_utils::drawHistogramWithRangeBin(one_histogram_image,
                                                           histogram_bins[j],
                                                           pixel_min_value_,
                                                           pixel_max_value_,
                                                           top_n_histogram_bins[0].count,
                                                           cv::Scalar(0, 0, 0));
        }
        for (size_t j = 0; j < top_n_histogram_bins.size(); j++) {
          jsk_recognition_utils::drawHistogramWithRangeBin(one_histogram_image,
                                                           top_n_histogram_bins[j],
                                                           pixel_min_value_,
                                                           pixel_max_value_,
                                                           top_n_histogram_bins[0].count,
                                                           cv::Scalar(255, 0, 0));
        }
        if (histogram_image.empty()) {
          histogram_image = one_histogram_image;
        }
        else {
          cv::vconcat(histogram_image, one_histogram_image, histogram_image);
        }
        const cv::Scalar center_color = cv::mean(input_image, mask_image);
        for (size_t j = 0; j < model.height(); j++) {
          for (size_t i = 0; i < model.width(); i++) {
            if (mask_image.at<unsigned char>(j, i) != 0) {
              const unsigned char current_value = image.at<unsigned char>(j, i);
              // const unsigned char new_value = computePixelDistance(input_image.at<unsigned char>(j, i),
              //                                                      center_color[0]);
              const unsigned char new_value = computePixelHistogramDistance(input_image.at<unsigned char>(j, i),
                                                                            top_n_histogram_bins);
              if (current_value > new_value || current_value == 0) {
                image.at<unsigned char>(j, i) = new_value;
              }
            }
          }
        }
      }
      pub_.publish(cv_bridge::CvImage(msg->header,
                                      sensor_msgs::image_encodings::MONO8,
                                      image).toImageMsg());
      pub_debug_polygon_.publish(cv_bridge::CvImage(msg->header,
                                                    sensor_msgs::image_encodings::RGB8,
                                                    debug_polygon_image).toImageMsg());
      pub_debug_histogram_image_.publish(cv_bridge::CvImage(msg->header,
                                                            sensor_msgs::image_encodings::RGB8,
                                                            histogram_image).toImageMsg());
    }
    catch (...)
    {
      NODELET_ERROR("Failed to resolve tf");
    }
  }
  
  void TabletopColorDifferenceLikelihood::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bin_size_ = config.bin_size;
    pixel_min_value_ = config.pixel_min_value;
    pixel_max_value_ = config.pixel_max_value;
    histogram_top_n_ratio_ = config.histogram_top_n_ratio;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::TabletopColorDifferenceLikelihood, nodelet::Nodelet);
