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


#ifndef JSK_PCL_ROS_HEIGHTMAP_MORPHOLOGICAL_FILTERLING_H_
#define JSK_PCL_ROS_HEIGHTMAP_MORPHOLOGICAL_FILTERLING_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <jsk_pcl_ros/HeightmapMorphologicalFilteringConfig.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_recognition_msgs/HeightmapConfig.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>
#include <boost/accumulators/statistics/stats.hpp>

namespace jsk_pcl_ros
{
  class HeightmapMorphologicalFiltering: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<HeightmapMorphologicalFiltering> Ptr;
    typedef HeightmapMorphologicalFilteringConfig Config;
    typedef boost::accumulators::accumulator_set<
      float,
      boost::accumulators::stats<
        boost::accumulators::tag::variance,
        boost::accumulators::tag::count,
        boost::accumulators::tag::mean> >  Accumulator;
    HeightmapMorphologicalFiltering(): DiagnosticNodelet("HeightmapMorphologicalFiltering") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void filter(const sensor_msgs::Image::ConstPtr& msg);
    virtual void configCallback(Config& config, uint32_t level);
    virtual void configTopicCallback(const jsk_recognition_msgs::HeightmapConfig::ConstPtr& msg);
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_;
    ros::Publisher pub_config_;
    ros::Subscriber sub_;
    ros::Subscriber sub_config_;
    int mask_size_;
    double max_variance_;
    int max_queue_size_;
    std::string smooth_method_;
    int bilateral_filter_size_;
    double bilateral_sigma_color_;
    double bilateral_sigma_space_;
    bool use_bilateral_;
  };
}

#endif
