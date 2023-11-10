// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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

#ifndef JSK_PCL_POINTCLOUD_TO_PCD_H_
#define JSK_PCL_POINTCLOUD_TO_PCD_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <jsk_recognition_utils/tf_listener_singleton.h>

#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros_utils/PointCloudToPCDConfig.h>

namespace jsk_pcl_ros_utils
{
  class PointCloudToPCD: public pcl_ros::PCLNodelet
  {
  public:
    virtual ~PointCloudToPCD();
    typedef PointCloudToPCDConfig Config;
  protected:
    virtual void onInit();
    virtual void timerCallback (const ros::TimerEvent& event);
    virtual void configCallback(Config &config, uint32_t level);
    void savePCD();
    bool savePCDCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    boost::mutex mutex_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Timer timer_;
    std::string filename_;
    std::string ext_;
    double duration_;
    std::string prefix_;
    bool binary_;
    bool compressed_;
    std::string fixed_frame_;
    tf::TransformListener* tf_listener_;
    ros::ServiceServer srv_save_pcd_server_;
  private:
  };
}

#endif
