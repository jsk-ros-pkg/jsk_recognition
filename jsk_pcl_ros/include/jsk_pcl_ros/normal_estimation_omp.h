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


#ifndef JSK_PCL_ROS_NORMAL_ESTIMATION_OMP_H_
#define JSK_PCL_ROS_NORMAL_ESTIMATION_OMP_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/FeatureConfig.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_recognition_utils/time_util.h>
#include <std_msgs/Float32.h>

namespace jsk_pcl_ros
{
  class NormalEstimationOMP: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<NormalEstimationOMP> Ptr;
    typedef pcl_ros::FeatureConfig Config;
    NormalEstimationOMP(): DiagnosticNodelet("NormalEstimationOMP"), timer_(10) {}
    
  protected:

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void estimate(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void configCallback(
      Config& config, uint32_t level);
    
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Publisher pub_with_xyz_;
    ros::Publisher pub_latest_time_;
    ros::Publisher pub_average_time_;
    jsk_recognition_utils::WallDurationTimer timer_;
    ros::Subscriber sub_;
    std::string sensor_frame_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    int k_;
    double search_radius_;
    int num_of_threads_;
    
  private:
    
  };
}

#endif
