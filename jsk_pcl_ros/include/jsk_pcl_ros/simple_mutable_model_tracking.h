// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
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

#ifndef JSK_PCL_ROS_SIMPLE_MUTABLE_MODEL_TRACKING_H_
#define JSK_PCL_ROS_SIMPLE_MUTABLE_MODEL_TRACKING_H_

//-----------------------ROS-----------------------------//
#include <ros/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>

//-----------------------PCL-----------------------------//
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/filters/filter.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/filter.h>

#include <jsk_pcl_ros/SetPointCloud2.h>

namespace jsk_pcl_ros
{
  class SimpleMutableModelTracking: public pcl_ros::PCLNodelet
  {
  protected:
    ros::Subscriber sub_track_model_;
    ros::Subscriber sub_octree_result_;
    ros::Subscriber sub_cloud_;
    boost::mutex mtx_;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr track_model_cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr octree_result_cloud_;

    double resolution_;
    double pass_offset_;

    bool new_track_model_cloud;
    bool new_input_cloud;
    bool new_octree_result_cloud;

    std::string frame_id_;
    int MIN_POINTS;

    void trackModelCb(const sensor_msgs::PointCloud2 &pc);
    void cloudCb(const sensor_msgs::PointCloud2 &pc);
    void octreeResultCb(const sensor_msgs::PointCloud2 &pc);

    void convertROStoPCL(const sensor_msgs::PointCloud2 &pc, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud);
    void updateModel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& renew_model_cloud);
    void filterPassThroughXYZ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud,
                               pcl::PointCloud<pcl::PointXYZRGBA> &result,
                               pcl::PointXYZRGBA &min_pt,pcl::PointXYZRGBA &max_pt,
                               double offset);
  private:
    virtual void onInit();
  };
}

#endif
