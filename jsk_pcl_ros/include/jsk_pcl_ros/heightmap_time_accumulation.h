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


#ifndef JSK_PCL_ROS_HEIGHTMAP_TIME_ACCUMULATION_H_
#define JSK_PCL_ROS_HEIGHTMAP_TIME_ACCUMULATION_H_

#include <jsk_topic_tools/connection_based_nodelet.h>
#include <jsk_recognition_msgs/HeightmapConfig.h>
#include <jsk_pcl_ros/HeightmapTimeAccumulationConfig.h>
#include <dynamic_reconfigure/server.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include "jsk_pcl_ros/heightmap_utils.h"
#include <std_srvs/Empty.h>

// accumulate
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/stats.hpp>

namespace jsk_pcl_ros
{
  typedef pcl::PointXYZI PointType;

  class HeightmapTimeAccumulation:
    public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef boost::shared_ptr<HeightmapTimeAccumulation> Ptr;
    typedef HeightmapTimeAccumulationConfig Config;
    typedef boost::accumulators::accumulator_set<
      float,
      boost::accumulators::stats<
        boost::accumulators::tag::variance,
        boost::accumulators::tag::count,
        boost::accumulators::tag::mean> >  Accumulator;
    HeightmapTimeAccumulation(): ConnectionBasedNodelet() {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void accumulate(
      const sensor_msgs::Image::ConstPtr& new_heightmap);
    virtual void publishHeightmap(
      const cv::Mat& heightmap, const std_msgs::Header& header);
    virtual cv::Point toIndex(const PointType& p, const cv::Mat& map);
    virtual bool isValidIndex(const cv::Point& index, const cv::Mat& map);
    virtual bool isValidCell(const cv::Point& index, const cv::Mat& map);
    virtual void configTopicCallback(
      const jsk_recognition_msgs::HeightmapConfig::ConstPtr& config);
    virtual void prevPointCloud(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual bool resetCallback(std_srvs::Empty::Request& req,
                               std_srvs::Empty::Response& res);
    virtual void overwriteAccmulation(pcl::PointCloud<PointType > &transformed_pointcloud,
                                      cv::Mat &new_heightmap);
    virtual void mergedAccmulation(pcl::PointCloud<PointType > &transformed_pointcloud,
                                   cv::Mat &new_heightmap);
    virtual void configCallback(Config& config, uint32_t level);

    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    tf::TransformListener* tf_;
    Eigen::Affine3f prev_from_center_to_fixed_;
    std::string center_frame_id_;
    std::string fixed_frame_id_;
    cv::Mat accumulated_heightmap_;
    ros::Publisher pub_output_;
    ros::Publisher pub_config_;
    ros::ServiceServer srv_reset_;
    boost::shared_ptr<tf::MessageFilter<sensor_msgs::Image> > tf_filter_;
    message_filters::Subscriber<sensor_msgs::Image> sub_heightmap_;
    ros::Subscriber sub_previous_pointcloud_;
    ros::Subscriber sub_config_;
    pcl::PointCloud<PointType> prev_cloud_;
    jsk_recognition_msgs::HeightmapConfig::ConstPtr config_;
    double min_x_;
    double min_y_;
    double max_x_;
    double max_y_;
    int tf_queue_size_;
    bool use_offset_;
    int bilateral_filter_size_;
    double bilateral_sigma_color_;
    double bilateral_sigma_space_;
    bool use_bilateral_;
  private:
  };
}

#endif
