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


#ifndef JSK_PCL_ROS_UTILS_POSE_WITH_COVARIANCE_STAMPED_TO_GAUSSIAN_POINTCLOUD_H_
#define JSK_PCL_ROS_UTILS_POSE_WITH_COVARIANCE_STAMPED_TO_GAUSSIAN_POINTCLOUD_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <jsk_pcl_ros_utils/PoseWithCovarianceStampedToGaussianPointCloudConfig.h>
#include <Eigen/Geometry>
#include <dynamic_reconfigure/server.h>
namespace jsk_pcl_ros_utils
{
  class PoseWithCovarianceStampedToGaussianPointCloud: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef PoseWithCovarianceStampedToGaussianPointCloudConfig Config;
    PoseWithCovarianceStampedToGaussianPointCloud(): 
      DiagnosticNodelet("PoseWithCovarianceStampedToGaussianPointCloud") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void convert(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);
    virtual void configCallback(Config& config, uint32_t level);
    virtual float gaussian(const Eigen::Vector2f& input,
                           const Eigen::Vector2f& mean,
                           const Eigen::Matrix2f& S,
                           const Eigen::Matrix2f& S_inv);
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string cut_plane_;
    double threshold_;
    std::string normalize_method_;
    double normalize_value_;
    int sampling_num_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  };
}

#endif
