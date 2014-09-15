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


#ifndef JSK_PCL_ROS_MULTI_PLANE_SEGMENTATION_H_
#define JSK_PCL_ROS_MULTI_PLANE_SEGMENTATION_H_

#include <pcl_ros/pcl_nodelet.h>
#include "jsk_pcl_ros/pcl_util.h"
#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <dynamic_reconfigure/server.h>

#include <jsk_pcl_ros/MultiPlaneSACSegmentationConfig.h>

////////////////////////////////////////////////////////
// messages
////////////////////////////////////////////////////////
#include <sensor_msgs/PointCloud2.h>
#include <jsk_pcl_ros/PolygonArray.h>
#include <jsk_pcl_ros/ModelCoefficientsArray.h>
#include <jsk_pcl_ros/ClusterPointIndices.h>

namespace jsk_pcl_ros
{
  class MultiPlaneSACSegmentation: public pcl_ros::PCLNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef jsk_pcl_ros::MultiPlaneSACSegmentationConfig Config;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    
    virtual void applyRecursiveRANSAC(
      const pcl::PointCloud<PointT>::Ptr& input,
      std::vector<pcl::PointIndices::Ptr>& output_inliers,
      std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients,
      std::vector<ConvexPolygon::Ptr>& output_polygons);

    virtual void configCallback (Config &config, uint32_t level);
    
    ////////////////////////////////////////////////////////
    // ROS variabels
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Publisher pub_inliers_, pub_coefficients_, pub_polygons_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    double outlier_threshold_;
    int min_inliers_;
    int min_points_;
    int max_iterations_;
  private:
    
  };
}

#endif
