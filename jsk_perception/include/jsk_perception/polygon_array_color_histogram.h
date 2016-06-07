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


#ifndef JSK_PERCEPTION_POLYGON_ARRAY_COLOR_HISTOGRAM_H_
#define JSK_PERCEPTION_POLYGON_ARRAY_COLOR_HISTOGRAM_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>
#include <jsk_recognition_utils/geo/polygon.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_perception/PolygonArrayColorHistogramConfig.h>
#include <dynamic_reconfigure/server.h>

namespace jsk_perception
{
  class PolygonArrayColorHistogram: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    jsk_recognition_msgs::PolygonArray > ApproximateSyncPolicy;
    typedef jsk_perception::PolygonArrayColorHistogramConfig Config;
    PolygonArrayColorHistogram(): DiagnosticNodelet("PolygonArrayColorHistogram") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    virtual void compute(const sensor_msgs::Image::ConstPtr& image_msg,
                         const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg);
    virtual void debugPolygonImage(
      const jsk_recognition_utils::CameraDepthSensor& model,
      cv::Mat& image, jsk_recognition_utils::Polygon::Ptr polygon, size_t pi) const;

    virtual void configCallback(Config &config, uint32_t level);

    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Publisher pub_debug_polygon_;
    ros::Subscriber sub_info_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    sensor_msgs::CameraInfo::ConstPtr info_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    tf::TransformListener* tf_listener_;
    int max_queue_size_;
    int sync_queue_size_;
    int bin_size_;
    int pixel_min_value_, pixel_max_value_;
    int debug_line_width_;
  private:
  };
}

#endif
