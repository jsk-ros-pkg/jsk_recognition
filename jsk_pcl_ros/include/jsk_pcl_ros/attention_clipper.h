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


#ifndef JSK_PCL_ROS_ATTENTION_CLIPPER_H_
#define JSK_PCL_ROS_ATTENTION_CLIPPER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/CameraInfo.h>
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <image_geometry/pinhole_camera_model.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/Image.h>

namespace jsk_pcl_ros
{
  class AttentionClipper: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    AttentionClipper(): DiagnosticNodelet("AttentionClipper") { }

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void clip(const sensor_msgs::CameraInfo::ConstPtr& msg);
    virtual void clipPointcloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& pose);
    virtual void boxCallback(const jsk_recognition_msgs::BoundingBox::ConstPtr& box);
    virtual void poseArrayCallback(const geometry_msgs::PoseArray::ConstPtr& pose);
    virtual void boxArrayCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box);
    virtual jsk_recognition_utils::Vertices cubeVertices(Eigen::Vector3f& dimension);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void computeROI(
      const sensor_msgs::CameraInfo::ConstPtr& msg,
      std::vector<cv::Point2d>& points,
      cv::Mat& mask);
    virtual void publishBoundingBox(const std_msgs::Header& header);
    virtual void initializePoseList(size_t num);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_box_;
    ros::Subscriber sub_points_;
    ros::Publisher pub_camera_info_;
    ros::Publisher pub_bounding_box_array_;
    ros::Publisher pub_mask_;
    ros::Publisher pub_indices_;
    ros::Publisher pub_cluster_indices_;
    std::vector<ros::Publisher> multiple_pub_indices_;
    tf::TransformListener* tf_listener_;
    boost::mutex mutex_;

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    // only cube is supported
    jsk_recognition_utils::Vertices vertices_;
    // for multiple attention
    std::vector<Eigen::Affine3f> pose_list_;
    std::vector<Eigen::Affine3f> transformed_pose_list_;
    std::vector<std::string> frame_id_list_;
    jsk_recognition_utils::Vertices dimensions_;
    std::vector<std::string > prefixes_;

    bool use_multiple_attention_;
    bool negative_;
  private:

  };
}

#endif
