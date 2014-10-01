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
 *   * Neither the name of the Willow Garage nor the names of its
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


#ifndef JSK_PCL_ROS_POINTCLOUD_DATABASE_SERVER_H_
#define JSK_PCL_ROS_POINTCLOUD_DATABASE_SERVER_H_

#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include "jsk_pcl_ros/PointsArray.h"

namespace jsk_pcl_ros
{
  class PointCloudData
  {
  public:
    typedef boost::shared_ptr<PointCloudData> Ptr;
    PointCloudData(const std::string fname);
    virtual ~PointCloudData() { };
    virtual pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    getPointCloud();
    virtual sensor_msgs::PointCloud2 
    getROSPointCloud(ros::Time stamp);

  protected:
    const std::string file_name_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  private:
  };

  class PointcloudDatabaseServer: public pcl_ros::PCLNodelet
  {
  public:
    virtual ~PointcloudDatabaseServer();
  protected:
    virtual void onInit();
    virtual void timerCallback(const ros::TimerEvent& event);
    // virtual void registerPointcloud();
    // virtual void removePointcloud();
    // virtual void listPointcloud();
    ros::Publisher pub_points_;
    ros::Timer timer_;
    std::vector<PointCloudData::Ptr> point_clouds_;

  private:
  };
}

#endif
