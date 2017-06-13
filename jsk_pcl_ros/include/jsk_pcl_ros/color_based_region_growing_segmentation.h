// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Satoshi Otsubo and JSK Lab
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

#ifndef JSK_PCL_ROS_COLOR_BASED_REGION_GROWING_SEGMENTATION_H_
#define JSK_PCL_ROS_COLOR_BASED_REGION_GROWING_SEGMENTATION_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>

#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/ColorBasedRegionGrowingSegmentationConfig.h"
#include <jsk_topic_tools/connection_based_nodelet.h>
namespace jsk_pcl_ros
{
  class ColorBasedRegionGrowingSegmentation:
    public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
  protected:
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int distance_threshold_;
    int point_color_threshold_;
    int region_color_threshold_;
    int min_cluster_size_;
    typedef jsk_pcl_ros::ColorBasedRegionGrowingSegmentationConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void configCallback (Config &config, uint32_t level);
    virtual void subscribe();
    virtual void unsubscribe();
  private:
    virtual void onInit();
  };
}

#endif
