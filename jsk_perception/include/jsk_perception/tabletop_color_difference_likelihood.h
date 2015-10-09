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
#include <jsk_recognition_utils/geo/polygon.h>
#include <jsk_perception/TabletopColorDifferenceLikelihoodConfig.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_recognition_msgs/HistogramWithRangeBin.h>
namespace jsk_perception
{
  class TabletopColorDifferenceLikelihood: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef TabletopColorDifferenceLikelihoodConfig Config;
    TabletopColorDifferenceLikelihood(): DiagnosticNodelet("TabletopColorDifferenceLikelihood") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    virtual void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
    virtual void polygonCallback(const jsk_recognition_msgs::PolygonArray::ConstPtr& msg);
    virtual void configCallback(Config& config, uint32_t level);
    
    virtual void debugPolygonImage(const jsk_recognition_utils::CameraDepthSensor& model,
                                   cv::Mat& image,
                                   jsk_recognition_utils::Polygon::Ptr polygon,
                                   size_t pi) const;

    /**
     * @brief
     * compute a distance between pixels.
     * if cyclic_value_ is true, it take into account two direction.
     */
    inline virtual unsigned char computePixelDistance(const unsigned char from, const unsigned char to) const
    {
      if (cyclic_value_) {
        unsigned char diff = (unsigned char)std::abs((int)from - (int)to);
        unsigned char reverse_diff = pixel_max_value_ - diff;
        return (unsigned char)std::min(diff, reverse_diff);
      }
      else {
        return (unsigned char)std::abs((int)from - (int)to);
      }
    }

    /**
     *
     */
    inline virtual unsigned char computePixelHistogramDistance(const unsigned char from,
                                                               const std::vector<jsk_recognition_msgs::HistogramWithRangeBin>& bins)
    {
      unsigned char diff = 255;
      for (size_t i = 0; i < bins.size(); i++) {
        jsk_recognition_msgs::HistogramWithRangeBin bin = bins[i];
        if (bin.min_value < from && bin.max_value > from) {
          return 0;
        }
        else {
          unsigned char min_direction_distance = computePixelDistance(from, (unsigned char)bin.min_value);
          unsigned char max_direction_distance = computePixelDistance(from, (unsigned char)bin.max_value);
          diff = std::min(std::min(min_direction_distance, max_direction_distance),
                          diff);
        }
      }
      return diff;
    }

    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    sensor_msgs::CameraInfo::ConstPtr latest_info_msg_;
    jsk_recognition_msgs::PolygonArray::ConstPtr latest_polygon_msg_;
    tf::TransformListener* tf_listener_;
    ros::Publisher pub_;
    ros::Publisher pub_debug_polygon_;
    ros::Publisher pub_debug_histogram_image_;
    ros::Subscriber sub_info_;
    ros::Subscriber sub_polygons_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > tf_filter_;
    int tf_queue_size_;
    bool cyclic_value_;
    int pixel_max_value_;
    int pixel_min_value_;
    int bin_size_;
    double histogram_top_n_ratio_;
  private:
  };
}

#endif
