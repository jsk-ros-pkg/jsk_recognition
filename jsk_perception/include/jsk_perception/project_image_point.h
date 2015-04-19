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


#ifndef JSK_PERCEPTION_PROJECT_IMAGE_POINT_H_
#define JSK_PERCEPTION_PROJECT_IMAGE_POINT_H_

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <image_geometry/pinhole_camera_model.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_perception/ProjectImagePointConfig.h>

namespace jsk_perception
{
  class ProjectImagePoint: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<ProjectImagePoint> Ptr;
    typedef ProjectImagePointConfig Config;
    ProjectImagePoint(): DiagnosticNodelet("ProjectImagePoint") {}
    
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void project(const geometry_msgs::PointStamped::ConstPtr& msg);
    virtual void cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
    virtual void configCallback(Config& config, uint32_t level);
    
    boost::mutex mutex_;
    ros::Subscriber sub_;
    ros::Subscriber sub_camera_info_;
    ros::Publisher pub_;
    ros::Publisher pub_vector_;
    sensor_msgs::CameraInfo::ConstPtr camera_info_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    double z_;
    
  private:
    
  };
}

#endif
