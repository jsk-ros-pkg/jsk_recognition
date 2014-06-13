// -*- mode: C++ -*-
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
 *   * Neither the name of the Willow Garage nor the names of its
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
#include <jsk_pcl_ros/ColorHistogram.h>
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

namespace jsk_perception
{
  class ColorHistogram: public nodelet::Nodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::Image,
                                                             geometry_msgs::PolygonStamped > SyncPolicy;
    typedef jsk_perception::ColorHistogramConfig Config;
  protected:
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    image_transport::SubscriberFilter image_sub_;
    message_filters::Subscriber<geometry_msgs::PolygonStamped> rectangle_sub_;
    ros::NodeHandle nh_, pnh_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::Publisher b_hist_pub_, r_hist_pub_, g_hist_pub_,
      h_hist_pub_, s_hist_pub_, i_hist_pub_;
    int b_hist_size_, r_hist_size_, g_hist_size_,
      h_hist_size_, s_hist_size_, i_hist_size_;
    boost::mutex mutex_;
    
    void configCallback(Config &new_config, uint32_t level)
    {
      boost::mutex::scoped_lock(mutex_);
      b_hist_size_ = new_config.blue_histogram_bin;
      g_hist_size_ = new_config.green_histogram_bin;
      r_hist_size_ = new_config.red_histogram_bin;
      h_hist_size_ = new_config.hue_histogram_bin;
      s_hist_size_ = new_config.saturation_histogram_bin;
      i_hist_size_ = new_config.intensity_histogram_bin;
    }
    virtual void onInit()
    {
      nh_ = ros::NodeHandle(getNodeHandle(), "image");
      pnh_ = ros::NodeHandle("~");
      b_hist_size_ = r_hist_size_ = g_hist_size_ = h_hist_size_ = s_hist_size_ = i_hist_size_ = 512;
      b_hist_pub_ = nh_.advertise<jsk_pcl_ros::ColorHistogram>("blue_histogram", 1);
      g_hist_pub_ = nh_.advertise<jsk_pcl_ros::ColorHistogram>("green_histogram", 1);
      r_hist_pub_ = nh_.advertise<jsk_pcl_ros::ColorHistogram>("red_histogram", 1);
      h_hist_pub_ = nh_.advertise<jsk_pcl_ros::ColorHistogram>("hue_histogram", 1);
      s_hist_pub_ = nh_.advertise<jsk_pcl_ros::ColorHistogram>("saturation_histogram", 1);
      i_hist_pub_ = nh_.advertise<jsk_pcl_ros::ColorHistogram>("intensity_histogram", 1);
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&ColorHistogram::configCallback, this, _1, _2);
      srv_->setCallback (f);

      it_.reset(new image_transport::ImageTransport(nh_));
      image_sub_.subscribe(*it_, "", 1);
      rectangle_sub_.subscribe(nh_, "screenrectangle", 1);
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
      sync_->connectInput(image_sub_, rectangle_sub_);
      sync_->registerCallback(boost::bind(&ColorHistogram::extract, this, _1, _2));
    }

    virtual void convertHistogramToMsg(const cv::Mat& hist,
                                       int size,
                                       jsk_pcl_ros::ColorHistogram& msg)
    {
      msg.histogram.clear();
      for (int i = 0; i < size; i++) {
        float val = hist.at<float>(0, i);
        msg.histogram.push_back(val);
      }
    }

    virtual void processBGR(const cv::Mat& bgr_image, const std_msgs::Header& header)
    {

      float range[] = { 0, 256 } ;
      const float* histRange = { range };
      cv::MatND b_hist, g_hist, r_hist;
      bool uniform = true; bool accumulate = false;
      std::vector<cv::Mat> bgr_planes;
      split( bgr_image, bgr_planes );
      
      cv::calcHist( &bgr_planes[0], 1, 0, cv::Mat(), b_hist, 1, &b_hist_size_, &histRange, uniform, accumulate );
      cv::calcHist( &bgr_planes[1], 1, 0, cv::Mat(), g_hist, 1, &g_hist_size_, &histRange, uniform, accumulate );
      cv::calcHist( &bgr_planes[2], 1, 0, cv::Mat(), r_hist, 1, &r_hist_size_, &histRange, uniform, accumulate );
      
      jsk_pcl_ros::ColorHistogram b_histogram;
      b_histogram.header = header;
      convertHistogramToMsg(b_hist, b_hist_size_, b_histogram);
      b_hist_pub_.publish(b_histogram);
      
      jsk_pcl_ros::ColorHistogram g_histogram;
      g_histogram.header = header;
      convertHistogramToMsg(g_hist, g_hist_size_, g_histogram);
      g_hist_pub_.publish(g_histogram);
      
      jsk_pcl_ros::ColorHistogram r_histogram;
      r_histogram.header = header;
      convertHistogramToMsg(r_hist, r_hist_size_, r_histogram);
      r_hist_pub_.publish(r_histogram);
      
    }
    
    virtual void processHSI(const cv::Mat& bgr_image, const std_msgs::Header& header)
    {
      cv::Mat hsi_image;
      cv::cvtColor(bgr_image, hsi_image, CV_BGR2HSV);
      
      float range[] = { 0, 256 } ;
      const float* histRange = { range };
      float h_range[] = { 0, 180 } ;
      const float* h_histRange = { h_range };
      cv::MatND h_hist, s_hist, i_hist;
      bool uniform = true; bool accumulate = false;
      std::vector<cv::Mat> hsi_planes;
      split( hsi_image, hsi_planes );
      
      cv::calcHist( &hsi_planes[0], 1, 0, cv::Mat(), h_hist, 1, &h_hist_size_, &h_histRange, uniform, accumulate );
      cv::calcHist( &hsi_planes[1], 1, 0, cv::Mat(), s_hist, 1, &s_hist_size_, &histRange, uniform, accumulate );
      cv::calcHist( &hsi_planes[2], 1, 0, cv::Mat(), i_hist, 1, &i_hist_size_, &histRange, uniform, accumulate );
      
      jsk_pcl_ros::ColorHistogram h_histogram;
      h_histogram.header = header;
      convertHistogramToMsg(h_hist, h_hist_size_, h_histogram);
      h_hist_pub_.publish(h_histogram);
      
      jsk_pcl_ros::ColorHistogram s_histogram;
      s_histogram.header = header;
      convertHistogramToMsg(s_hist, s_hist_size_, s_histogram);
      s_hist_pub_.publish(s_histogram);
      
      jsk_pcl_ros::ColorHistogram i_histogram;
      i_histogram.header = header;
      convertHistogramToMsg(i_hist, i_hist_size_, i_histogram);
      i_hist_pub_.publish(i_histogram);
      
    }
    
    virtual void extract(const sensor_msgs::Image::ConstPtr& image,
                         const geometry_msgs::PolygonStamped::ConstPtr& rectangle)
    {
      boost::mutex::scoped_lock(mutex_);
      try
      {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

        int larger_x_index = rectangle->polygon.points[0].x > rectangle->polygon.points[1].x? 0: 1;
        int larger_y_index = rectangle->polygon.points[0].y > rectangle->polygon.points[1].y? 0: 1;
        int smaller_x_index = rectangle->polygon.points[0].x < rectangle->polygon.points[1].x? 0: 1;
        int smaller_y_index = rectangle->polygon.points[0].y < rectangle->polygon.points[1].y? 0: 1;
        cv::Rect roi(rectangle->polygon.points[smaller_x_index].x,
                     rectangle->polygon.points[smaller_y_index].y,
                     rectangle->polygon.points[larger_x_index].x - rectangle->polygon.points[smaller_x_index].x,
                     rectangle->polygon.points[larger_y_index].y - rectangle->polygon.points[smaller_y_index].y);
        cv::Mat bgr_image = cv_ptr->image(roi);
        processBGR(bgr_image, image->header);
        processHSI(bgr_image, image->header);
      }
      catch (cv_bridge::Exception& e)
      {
        NODELET_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
    }
  private:
    
  };
}

#include <pluginlib/class_list_macros.h>
typedef jsk_perception::ColorHistogram ColorHistogram;
PLUGINLIB_DECLARE_CLASS (jsk_perception, ColorHistogram, ColorHistogram, nodelet::Nodelet);
