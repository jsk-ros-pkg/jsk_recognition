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


#ifndef JSK_PCL_ROS_INTERMITTENT_IMAGE_ANNOTATOR_H_
#define JSK_PCL_ROS_INTERMITTENT_IMAGE_ANNOTATOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_srvs/Empty.h>
#include <image_geometry/pinhole_camera_model.h>

#include <image_transport/image_transport.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <boost/circular_buffer.hpp>
#include <Eigen/Geometry>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_recognition_msgs/PosedCameraInfo.h>
#include <pcl_ros/transforms.h>


namespace jsk_pcl_ros
{
  class SnapshotInformation
  {
  public:
    typedef boost::shared_ptr<SnapshotInformation> Ptr;
    SnapshotInformation() {};
    virtual ~SnapshotInformation() {};
    
    Eigen::Affine3d camera_pose_;
    cv::Mat image_;
    image_geometry::PinholeCameraModel camera_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  protected:
  private:
    
  };
  
  class IntermittentImageAnnotator: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<IntermittentImageAnnotator> Ptr;
    IntermittentImageAnnotator():
      DiagnosticNodelet("IntermittentImageAnnotator") {}

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void waitForNextImage();
    virtual void cameraCallback(
      const sensor_msgs::Image::ConstPtr& image_msg,
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);
    virtual void rectCallback(
      const geometry_msgs::PolygonStamped::ConstPtr& rect);
    virtual bool shutterCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual bool requestCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual bool clearCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual void publishCroppedPointCloud(
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
      const cv::Point3d& A, const cv::Point3d& B,
      const cv::Point3d& C, const cv::Point3d& D,
      const Eigen::Affine3d& pose);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    tf::TransformListener* listener_;
    double rate_;
    ros::Time last_publish_time_;
    image_transport::Publisher image_pub_;
    image_transport::CameraSubscriber image_sub_;
    boost::mutex mutex_;
    ros::Subscriber rect_sub_;
    ros::Subscriber cloud_sub_;
    ros::ServiceServer shutter_service_;
    ros::ServiceServer clear_service_;
    ros::ServiceServer request_service_;
    ros::Publisher pub_pose_;
    ros::Publisher pub_roi_;
    ros::Publisher pub_marker_;
    ros::Publisher pub_cloud_;
    sensor_msgs::Image::ConstPtr latest_image_msg_;
    sensor_msgs::CameraInfo::ConstPtr latest_camera_info_msg_;
    sensor_msgs::PointCloud2::ConstPtr latest_cloud_msg_;
    bool store_pointcloud_;
    bool keep_organized_;
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    int max_image_buffer_;
    std::string fixed_frame_id_;
    boost::circular_buffer<SnapshotInformation::Ptr> snapshot_buffer_;
    
  private:
    
  };
}

#endif
