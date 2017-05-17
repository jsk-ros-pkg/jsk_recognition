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

#ifndef JSK_PCL_ROS_HEIGHTMAP_CONVERTER_H_
#define JSK_PCL_ROS_HEIGHTMAP_CONVERTER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/HeightmapConverterConfig.h>
#include <dynamic_reconfigure/server.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <jsk_recognition_msgs/HeightmapConfig.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

namespace jsk_pcl_ros
{

  class HeightmapConverter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<HeightmapConverter> Ptr;
    typedef HeightmapConverterConfig Config;
    HeightmapConverter(): DiagnosticNodelet("HeightmapConverter") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);
    virtual void convert(const sensor_msgs::PointCloud2::ConstPtr& msg);
    inline cv::Point toIndex(const pcl::PointXYZ& p) const
    {
      if (p.x > max_x_ || p.x < min_x_ ||
          p.y > max_y_ || p.y < min_y_) {
        return cv::Point(-1, -1);
      }
      const float offsetted_x = p.x - min_x_;
      const float offsetted_y = p.y - min_y_;
      const float dx = (max_x_ - min_x_) / resolution_x_;
      const float dy = (max_y_ - min_y_) / resolution_y_;
      return cv::Point(std::floor(offsetted_x / dx),
                       std::floor(offsetted_y / dy));
    }

    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Publisher pub_config_;
    ros::Subscriber sub_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    double min_x_;
    double max_x_;
    double min_y_;
    double max_y_;
    int resolution_x_;
    int resolution_y_;
    int max_queue_size_;
    std::string fixed_frame_id_;
    std::string center_frame_id_;
    std::string projected_center_frame_id_;
    bool use_projected_center_;
    double initial_probability_;

    tf::TransformListener* tf_;
    tf::TransformBroadcaster tf_broadcaster_;
  };
}

#endif
