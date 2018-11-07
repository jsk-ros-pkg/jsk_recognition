// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/ColorHistogram.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/ColorHistogramConfig.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <jsk_topic_tools/diagnostic_nodelet.h>

namespace jsk_perception
{
  class ColorHistogram: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    geometry_msgs::PolygonStamped > SyncPolicy;
    
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      sensor_msgs::Image> MaskSyncPolicy;
    typedef jsk_perception::ColorHistogramConfig Config;
    ColorHistogram(): DiagnosticNodelet("ColorHistogram") {}
  protected:
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<MaskSyncPolicy> > mask_sync_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    image_transport::SubscriberFilter image_sub_;
    image_transport::SubscriberFilter image_mask_sub_;
    message_filters::Subscriber<geometry_msgs::PolygonStamped> rectangle_sub_;
    ros::NodeHandle nh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::Publisher b_hist_pub_, r_hist_pub_, g_hist_pub_,
      h_hist_pub_, s_hist_pub_, i_hist_pub_;
    ros::Publisher image_pub_;
    int b_hist_size_, r_hist_size_, g_hist_size_,
      h_hist_size_, s_hist_size_, i_hist_size_;
    bool use_mask_;
    boost::mutex mutex_;

    void configCallback(Config &new_config, uint32_t level);
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void convertHistogramToMsg(
      const cv::Mat& hist,
      int size,
      jsk_recognition_msgs::ColorHistogram& msg);
    virtual void processBGR(const cv::Mat& bgr_image,
                            const std_msgs::Header& header);
    virtual void processHSI(const cv::Mat& bgr_image,
                            const std_msgs::Header& header);
    virtual void processBGR(const cv::Mat& bgr_image,
                            const cv::Mat& mask,
                            const std_msgs::Header& header);
    virtual void processHSI(const cv::Mat& bgr_image,
                            const cv::Mat& mask,
                            const std_msgs::Header& header);
    virtual void extract(
      const sensor_msgs::Image::ConstPtr& image,
      const geometry_msgs::PolygonStamped::ConstPtr& rectangle);
    virtual void extractMask(
      const sensor_msgs::Image::ConstPtr& image,
      const sensor_msgs::Image::ConstPtr& mask_image);
  private:
    
  };
}
