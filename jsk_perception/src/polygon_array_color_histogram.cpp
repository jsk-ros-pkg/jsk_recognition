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
#include "jsk_perception/polygon_array_color_histogram.h"
#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/sensor_model/camera_depth_sensor.h>
#include <jsk_recognition_msgs/HistogramWithRangeArray.h>
#include <jsk_recognition_utils/color_utils.h>
#include <jsk_topic_tools/color_utils.h>
#include <jsk_recognition_utils/sensor_model_utils.h>
#include <jsk_recognition_utils/cv_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_utils/pcl_ros_util.h>

namespace jsk_perception
{
  void PolygonArrayColorHistogram::onInit()
  {
    DiagnosticNodelet::onInit();
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pnh_->param("max_queue_size", max_queue_size_, 10);
    pnh_->param("synchronizer_queue_size", sync_queue_size_, 100);
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (
        &PolygonArrayColorHistogram::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<jsk_recognition_msgs::HistogramWithRangeArray>(*pnh_, "output", 1);
    pub_debug_polygon_ = advertise<sensor_msgs::Image>(*pnh_, "debug/polygon_image", 1);
    onInitPostProcess();
  }

  void PolygonArrayColorHistogram::subscribe()
  {
    sub_info_ = pnh_->subscribe("input/info", 1, &PolygonArrayColorHistogram::infoCallback, this);
    sub_polygon_.subscribe(*pnh_, "input", max_queue_size_);
    sub_image_.subscribe(*pnh_, "input/image", max_queue_size_);
    async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(sync_queue_size_);
    async_->connectInput(sub_image_, sub_polygon_);
    async_->registerCallback(
      boost::bind(&PolygonArrayColorHistogram::compute, this, _1, _2));
  }

  void PolygonArrayColorHistogram::unsubscribe()
  {
    sub_info_.shutdown();
    sub_polygon_.unsubscribe();
    sub_image_.unsubscribe();
  }

  void PolygonArrayColorHistogram::infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    info_ = msg;
  }
  
  void PolygonArrayColorHistogram::debugPolygonImage(
    const jsk_recognition_utils::CameraDepthSensor& model,
    cv::Mat& image,
    jsk_recognition_utils::Polygon::Ptr polygon,
    size_t pi) const
  {
    polygon->drawLineToImage(model, image,
                             jsk_recognition_utils::colorROSToCVRGB(jsk_topic_tools::colorCategory20(pi)),
                             debug_line_width_);
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


  void PolygonArrayColorHistogram::compute(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (!info_) {
      NODELET_WARN("No camera_info is available");
      return;
    }
    // check frame_id
    if (!jsk_recognition_utils::isSameFrameId(image_msg->header.frame_id,
                                              polygon_msg->header.frame_id)) {
      NODELET_ERROR("frame_id does not match. image: %s, polygon: %s",
                        image_msg->header.frame_id.c_str(),
                        polygon_msg->header.frame_id.c_str());
      return;
    }
    try 
    {
      tf::StampedTransform transform 
        = jsk_recognition_utils::lookupTransformWithDuration(
          tf_listener_, polygon_msg->header.frame_id, image_msg->header.frame_id,
          image_msg->header.stamp, ros::Duration(1.0));
      Eigen::Affine3f polygon_transform;
      tf::transformTFToEigen(transform, polygon_transform);
      cv_bridge::CvImagePtr input_cv_image 
        = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat input_image = input_cv_image->image;
      jsk_recognition_utils::CameraDepthSensor model;
      model.setCameraInfo(*info_);
      cv::Mat debug_polygon_image = model.image(CV_8UC3);
      std::vector<jsk_recognition_utils::Polygon::Ptr> polygons 
        = jsk_recognition_utils::Polygon::fromROSMsg(*polygon_msg, polygon_transform);
      cv::Mat image = model.image(CV_8UC1);
      jsk_recognition_msgs::HistogramWithRangeArray histogram_array;
      histogram_array.header = polygon_msg->header;
      for (size_t pi = 0; pi < polygons.size(); pi++) {
        jsk_recognition_utils::Polygon::Ptr polygon = polygons[pi];
        debugPolygonImage(model, debug_polygon_image, polygon, pi);
        cv::Mat mask_image;
        polygon->maskImage(model, mask_image);
        cv::MatND hist = jsk_recognition_utils::computeHistogram(input_image, bin_size_,
                                                                 pixel_min_value_, pixel_max_value_,
                                                                 mask_image);
        jsk_recognition_msgs::HistogramWithRange histogram;
        histogram.header = polygon_msg->header;
        histogram.bins
          = jsk_recognition_utils::cvMatNDToHistogramWithRangeBinArray(
            hist, pixel_min_value_, pixel_max_value_);
        histogram_array.histograms.push_back(histogram);
      }
      pub_debug_polygon_.publish(cv_bridge::CvImage(image_msg->header,
                                                    sensor_msgs::image_encodings::RGB8,
                                                    debug_polygon_image).toImageMsg());
      pub_.publish(histogram_array);
    }
    catch (...)
    {
      NODELET_ERROR("Failed to resolve tf");
    }

  }

  void PolygonArrayColorHistogram::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    bin_size_ = config.bin_size;
    pixel_min_value_ = config.pixel_min_value;
    pixel_max_value_ = config.pixel_max_value;
    debug_line_width_ = config.debug_line_width;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::PolygonArrayColorHistogram, nodelet::Nodelet);
