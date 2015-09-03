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


#ifndef JSK_PERCEPTION_TABLETOP_COLOR_DIFFERENCE_LIKELIHOOD_H_
#define JSK_PERCEPTION_TABLETOP_COLOR_DIFFERENCE_LIKELIHOOD_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>
#include <jsk_topic_tools/log_utils.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <jsk_recognition_utils/geo_util.h>

namespace jsk_perception
{
  class TabletopColorDifferenceLikelihood: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    TabletopColorDifferenceLikelihood(): DiagnosticNodelet("TabletopColorDifferenceLikelihood") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    virtual void polygonCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

    boost::mutex mutex_;
    sensor_msgs::CameraInfo::ConstPtr latest_info_msg_;
    jsk_recognition_msgs::PolygonArray::ConstPtr latest_polygon_msg_;
    tf::TransformListener* tf_listener_;
    ros::Publisher pub_;
    ros::Subscriber sub_info_;
    ros::Subscriber sub_polygons_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > tf_filter_;
    int tf_queue_size_;
  private:
  };
}

#endif
