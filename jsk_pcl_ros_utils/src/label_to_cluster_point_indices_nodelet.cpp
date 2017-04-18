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
#include "jsk_pcl_ros_utils/label_to_cluster_point_indices.h"
#include "jsk_topic_tools/log_utils.h"
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/assign.hpp>
#include <map>

namespace jsk_pcl_ros_utils
{

  void LabelToClusterPointIndices::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    pub_bg_ = advertise<pcl_msgs::PointIndices>(*pnh_, "output/bg_indices", 1);
    onInitPostProcess();
  }

  void LabelToClusterPointIndices::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1,
                           &LabelToClusterPointIndices::convert,
                           this);
    ros::V_string names = boost::assign::list_of("~input");
    jsk_topic_tools::warnNoRemap(names);
  }

  void LabelToClusterPointIndices::unsubscribe()
  {
    sub_.shutdown();
  }

  void LabelToClusterPointIndices::convert(
    const sensor_msgs::Image::ConstPtr& label_msg)
  {
    vital_checker_->poke();
    cv_bridge::CvImagePtr label_img_ptr = cv_bridge::toCvCopy(
      label_msg, sensor_msgs::image_encodings::TYPE_32SC1);
    // collect indices for each labels
    std::map<int, pcl_msgs::PointIndices> label_to_indices;
    for (size_t j = 0; j < label_img_ptr->image.rows; j++)
    {
      for (size_t i = 0; i < label_img_ptr->image.cols; i++)
      {
        int label = label_img_ptr->image.at<int>(j, i);
        label_to_indices[label].header = label_msg->header;
        label_to_indices[label].indices.push_back(j * label_img_ptr->image.cols + i);
      }
    }
    // convert 'map for label to indices' to 'cluster point indices'
    int bg_label = 0;
    int cluster_counter = 0;
    jsk_recognition_msgs::ClusterPointIndices cluster_indices_msg;
    pcl_msgs::PointIndices bg_indices_msg;
    cluster_indices_msg.header = bg_indices_msg.header = label_msg->header;
    for (std::map<int, pcl_msgs::PointIndices>::iterator it = label_to_indices.begin();
         it != label_to_indices.end(); it++)
    {
      if (it->first == bg_label) {
        bg_indices_msg.indices = it->second.indices;
        cluster_counter++;
        continue;
      }
      while (it->first != cluster_counter) {
        pcl_msgs::PointIndices indices_msg;
        indices_msg.header = label_msg->header;
        cluster_indices_msg.cluster_indices.push_back(indices_msg);
        cluster_counter++;
      }
      cluster_indices_msg.cluster_indices.push_back(it->second);
      cluster_counter++;
    }
    pub_bg_.publish(bg_indices_msg);
    pub_.publish(cluster_indices_msg);
  }

}  // namespace jsk_pcl_ros_utils

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros_utils::LabelToClusterPointIndices, nodelet::Nodelet);
