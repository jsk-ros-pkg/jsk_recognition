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


#ifndef JSK_PCL_ROS_INTERACTIVE_CUBOID_LIKELIHOOD_H_
#define JSK_PCL_ROS_INTERACTIVE_CUBOID_LIKELIHOOD_H_

#include "jsk_pcl_ros/plane_supported_cuboid_estimator.h"
#include <interactive_markers/interactive_marker_server.h>
#include <jsk_pcl_ros/InteractiveCuboidLikelihoodConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Float32.h>

namespace jsk_pcl_ros
{
  class InteractiveCuboidLikelihood: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<InteractiveCuboidLikelihood> Ptr;
    typedef InteractiveCuboidLikelihoodConfig Config;
    typedef pcl::tracking::ParticleCuboid Particle;
    InteractiveCuboidLikelihood(): DiagnosticNodelet("InteractiveCuboidLikelihood") {}
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void likelihood(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void processFeedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);
    virtual void processPlaneFeedback(const visualization_msgs::InteractiveMarkerFeedback::ConstPtr& feedback);
    virtual void configCallback(Config& config, uint32_t level);
    virtual visualization_msgs::Marker particleToMarker(const Particle& p);
    virtual visualization_msgs::InteractiveMarker particleToInteractiveMarker(const Particle& p);
    virtual visualization_msgs::InteractiveMarker planeInteractiveMarker();
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    Eigen::Affine3f plane_pose_;
    std::string frame_id_;
    Particle particle_;
    Config config_;
    tf::TransformListener* tf_;
    Eigen::Vector3f viewpoint_;
    std::string sensor_frame_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    boost::shared_ptr<interactive_markers::InteractiveMarkerServer> plane_server_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
  private:
    
  };
}

#endif
