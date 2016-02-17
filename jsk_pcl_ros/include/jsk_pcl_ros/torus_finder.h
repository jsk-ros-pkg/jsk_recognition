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


#ifndef JSK_PCL_ROS_TORUS_FINDER_H_
#define JSK_PCL_ROS_TORUS_FINDER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/TorusArray.h>
#include <jsk_recognition_msgs/Torus.h>
#include <jsk_pcl_ros/TorusFinderConfig.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Core>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_utils/time_util.h>

namespace jsk_pcl_ros
{
  class TorusFinder: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef TorusFinderConfig Config;
    TorusFinder(): timer_(10), done_initialization_(false), DiagnosticNodelet("TorusFinder") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void segmentFromPoints(const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg);
    virtual void configCallback(Config &config, uint32_t level);
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Subscriber sub_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_torus_;
    ros::Publisher pub_torus_array_;
    ros::Publisher pub_torus_with_failure_;
    ros::Publisher pub_torus_array_with_failure_;
    ros::Publisher pub_inliers_;
    ros::Publisher pub_coefficients_;
    ros::Publisher pub_pose_stamped_;
    ros::Publisher pub_latest_time_;
    ros::Publisher pub_average_time_;
    jsk_recognition_utils::WallDurationTimer timer_;
    boost::mutex mutex_;
    Eigen::Vector3f hint_axis_;

    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    std::string algorithm_;
    double min_radius_;
    double max_radius_;
    double outlier_threshold_;
    double eps_hint_angle_;
    bool use_hint_;
    bool use_normal_;
    int max_iterations_;
    int min_size_;
    bool voxel_grid_sampling_;
    double voxel_size_;
    bool done_initialization_;
  private:
  };
}

#endif
