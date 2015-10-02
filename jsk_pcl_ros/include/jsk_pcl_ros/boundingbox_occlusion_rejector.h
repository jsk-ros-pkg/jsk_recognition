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


#ifndef JSK_PCL_ROS_BOUNDING_BOX_OCCLUSION_REJECTOR_H_
#define JSK_PCL_ROS_BOUNDING_BOX_OCCLUSION_REJECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"

namespace jsk_pcl_ros
{
  class BoundingBoxOcclusionRejector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<BoundingBoxOcclusionRejector> Ptr;
    BoundingBoxOcclusionRejector(): DiagnosticNodelet("BoundingBoxOcclusionRejector"){}

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void reject(
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& candidate_boxes_msg);
    virtual void infoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual void targetBoxesCallback(
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& target_boxes_msg);
    virtual std::vector<cv::Point2i> projectVertices(const std::vector<cv::Point3d>& vertices,
                                                     const image_geometry::PinholeCameraModel& model);
    virtual std::vector<cv::Point3d> getVertices(const jsk_recognition_msgs::BoundingBox& box);
    virtual std::vector<std::vector<cv::Point2i> > separateIntoFaces(
      const std::vector<cv::Point2i>& vertices);
    boost::mutex mutex_;
    ros::Publisher pub_;
    ros::Publisher pub_target_image_;
    ros::Publisher pub_candidate_image_;
    tf::TransformListener* tf_listener_;
    ros::Subscriber sub_camera_info_;
    ros::Subscriber sub_target_boxes_;
    ros::Subscriber sub_candidate_boxes_;
    sensor_msgs::CameraInfo::ConstPtr latest_info_msg_;
    jsk_recognition_msgs::BoundingBoxArray::ConstPtr latest_target_boxes_msg_;
    
  private:
    
  };
}

#endif
