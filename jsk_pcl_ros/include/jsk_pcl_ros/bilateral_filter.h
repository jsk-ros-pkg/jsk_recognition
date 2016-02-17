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


#ifndef JSK_PCL_ROS_BILATERAL_FILTER_H_
#define JSK_PCL_ROS_BILATERAL_FILTER_H_

#include <pcl_ros/publisher.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_pcl_ros/BilateralFilterConfig.h>
#include <dynamic_reconfigure/server.h>

namespace jsk_pcl_ros
{
  ////////////////////////////////////////////////////////
  // Copied from pcl/point_types_conversion.h
  // in order to avoid pcl 1.7.1 bug.
  // It should be solved on pcl 1.7.2.
  ////////////////////////////////////////////////////////
  inline void 
  PointXYZRGBtoXYZI (pcl::PointXYZRGB&  in,
                     pcl::PointXYZI&    out)
  {
    out.x = in.x; out.y = in.y; out.z = in.z;
    out.intensity = 0.299f * static_cast <float> (in.r) + 0.587f * static_cast 
      <float> (in.g) + 0.114f * static_cast <float> (in.b);
  }

  inline void 
  PointCloudXYZRGBtoXYZI (pcl::PointCloud<pcl::PointXYZRGB>& in,
                          pcl::PointCloud<pcl::PointXYZI>& out)
  {
    out.width   = in.width;
    out.height  = in.height;
    for (size_t i = 0; i < in.points.size (); i++)
    {
      pcl::PointXYZI p;
      PointXYZRGBtoXYZI (in.points[i], p);
      out.points.push_back (p);
    }
  }

  
  class BilateralFilter: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef boost::shared_ptr<BilateralFilter> Ptr;
    typedef pcl::PointXYZRGB PointT;
    typedef BilateralFilterConfig Config;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();

    virtual void filter(const sensor_msgs::PointCloud2::ConstPtr& msg);

    virtual void subscribe();
    virtual void unsubscribe();

    virtual void configCallback(Config &config, uint32_t level);
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    ////////////////////////////////////////////////////////
    // parameters for bilateral filter
    ////////////////////////////////////////////////////////
    double sigma_s_;
    double sigma_r_;
    
  private:
    
    
  };
}


#endif
