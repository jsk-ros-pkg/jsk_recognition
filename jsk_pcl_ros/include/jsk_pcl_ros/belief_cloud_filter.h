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


#ifndef JSK_PCL_ROS_BELIEF_CLOUD_FILTER_H_
#define JSK_PCL_ROS_BELIEF_CLOUD_FILTER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

namespace jsk_pcl_ros
{
  namespace belief_cloud_initial_model
  {
    const std::string none = "none";
    const std::string uniform = "uniform";
    const std::string gaussian = "gaussian";
    const std::string pcd = "pcd";
    const std::string topic = "topic";
  }

  class BeliefCloudFilter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    BeliefCloudFilter(): DiagnosticNodelet("BeliefCloudFilter") { }
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual bool setupInitialModel();
    virtual void filter(const sensor_msgs::PointCloud2::ConstPtr& cloud);
    inline double gaussian(double x, double mean, double variance)
    {
      return 1 / sqrt(2 * M_PI * variance * variance) * exp(- (x - mean) * (x - mean) / (2.0 * variance * variance));
    }

    std::string initial_model_;
    double belief_cloud_dx_;
    double belief_cloud_dy_;
    double belief_cloud_dz_;
    double belief_cloud_size_x_;
    double belief_cloud_size_y_;
    double belief_cloud_size_z_;
    
    // for gaussian model
    std::vector<double> gaussian_means_;
    std::vector<double> gaussian_covariances_;
    double cutoff_threshold_;
    ros::Publisher pub_;
    ros::Publisher pub_initial_model_;
    ros::Subscriber sub_;
  private:
    
  };
}

#endif
