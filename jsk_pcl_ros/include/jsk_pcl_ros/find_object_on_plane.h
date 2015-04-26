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


#ifndef JSK_PCL_ROS_FIND_OBJECT_ON_PLANE_H_
#define JSK_PCL_ROS_FIND_OBJECT_ON_PLANE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_msgs/ModelCoefficients.h>
#include <image_geometry/pinhole_camera_model.h>
#include <pcl_conversions/pcl_conversions.h>
#include <jsk_pcl_ros/geo_util.h>

namespace jsk_pcl_ros
{
  class FindObjectOnPlane: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image,
    sensor_msgs::CameraInfo,
    pcl_msgs::ModelCoefficients > SyncPolicy;

    FindObjectOnPlane(): DiagnosticNodelet("FindObjectOnPlane") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void find(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg,
      const pcl_msgs::ModelCoefficients::ConstPtr& polygon_3d_coefficient_msg);
    virtual void generateStartPoints(
      const cv::Point2f& point_2d,
      const image_geometry::PinholeCameraModel& model,
      const pcl::ModelCoefficients::Ptr& coefficients,
      std::vector<cv::Point3f>& search_points_3d,
      std::vector<cv::Point2f>& search_points_2d);
    virtual void generateAngles(
      const cv::Mat& blob_image, const cv::Point2f& test_point,
      std::vector<double>& angles,
      std::vector<double>& max_x,
      std::vector<double>& max_y,
      const image_geometry::PinholeCameraModel& model,
      const jsk_pcl_ros::Plane::Ptr& plane);
    virtual double drawAngle(
      cv::Mat& out_image, const cv::Point2f& test_point, const double angle,
      const double max_x, const double max_y,
      const image_geometry::PinholeCameraModel& model,
      const jsk_pcl_ros::Plane::Ptr& plane,
      cv::Scalar color);
    Eigen::Vector3f rayPlaneInteersect(
      const cv::Point3d& ray,
      const jsk_pcl_ros::Plane::Ptr& plane);

    virtual cv::Point2d getUyEnd(
      const cv::Point2d& ux_start,
      const cv::Point2d& ux_end,
      const image_geometry::PinholeCameraModel& model,
      const jsk_pcl_ros::Plane::Ptr& plane);
      
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<pcl_msgs::ModelCoefficients> sub_coefficients_;
    ros::Publisher pub_min_area_rect_image_;
    
  private:
    
  };
}

#endif
