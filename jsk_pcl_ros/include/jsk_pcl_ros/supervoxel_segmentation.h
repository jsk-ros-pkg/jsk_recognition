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


#ifndef JSK_PCL_ROS_SUPERVOXEL_SEGMENTATION_H_
#define JSK_PCL_ROS_SUPERVOXEL_SEGMENTATION_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/SupervoxelSegmentationConfig.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
namespace jsk_pcl_ros
{
  class SupervoxelSegmentation: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef boost::shared_ptr<SupervoxelSegmentation> Ptr;
    typedef SupervoxelSegmentationConfig Config;
    SupervoxelSegmentation(): DiagnosticNodelet("SupervoxelSegmentation") {}
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void segment(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void configCallback (Config &config, uint32_t level);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;
    ros::Subscriber sub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_indices_;
    ros::Publisher pub_cloud_;

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    double color_importance_;
    double spatial_importance_;
    double normal_importance_;
    double voxel_resolution_;
    double seed_resolution_;
    bool use_transform_;
  private:
    
  };
}

#endif
