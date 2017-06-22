/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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

#define BOOST_PARAMETER_MAX_ARITY 7

#include <boost/thread/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/fill_image.h>
#include <sensor_msgs/image_encodings.h>

#include "jsk_pcl_ros/kinfu.h"

namespace enc = sensor_msgs::image_encodings;

namespace jsk_pcl_ros
{
  void
  Kinfu::onInit()
  {
    ConnectionBasedNodelet::onInit();
    always_subscribe_ = true;  // for mapping

    pnh_->param("device", device_, 0);
    pnh_->param("queue_size", queue_size_, 10);
    pnh_->param("auto_reset", auto_reset_, false);

    is_kinfu_initialized_ = false;

    pub_rendered_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/rendered_image", 1);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    srv_reset_ = pnh_->advertiseService("reset", &Kinfu::resetCallback, this);
    srv_save_mesh_ = pnh_->advertiseService("save_mesh", &Kinfu::saveMeshCallback, this);

    onInitPostProcess();
  }

  void
  Kinfu::initKinfu(const int height, const int width)
  {
    pcl::gpu::setDevice(device_);
    pcl::gpu::printShortCudaDeviceInfo(device_);

    float shift_distance = 10.0f * pcl::device::kinfuLS::DISTANCE_THRESHOLD;
    int snapshot_rate = pcl::device::kinfuLS::SNAPSHOT_RATE;
    Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(pcl::device::kinfuLS::VOLUME_SIZE);  // 3mm

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shift_distance, height, width);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();   // * AngleAxisf( pcl::deg2rad(-30.f), Eigen::Vector3f::UnitX());
    Eigen::Vector3f t = volume_size * 0.5f - Eigen::Vector3f(0, 0, volume_size(2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

    kinfu_->setInitialCameraPose(pose);
    kinfu_->volume().setTsdfTruncDist(0.030f/*meters*/);
    kinfu_->setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.0f)));
    //kinfu_->setDepthTruncationForICP(3.f/*meters*/);
    kinfu_->setCameraMovementThreshold(0.001f);
  }

  void
  Kinfu::subscribe()
  {
    sub_camera_info_.subscribe(*pnh_, "input/camera_info", 1);
    sub_depth_.subscribe(*pnh_, "input/depth", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(queue_size_);
    sync_->connectInput(sub_camera_info_, sub_depth_);
    sync_->registerCallback(boost::bind(&Kinfu::update, this, _1, _2));
  }

  void
  Kinfu::unsubscribe()
  {
    sub_camera_info_.unsubscribe();
    sub_depth_.unsubscribe();
  }

  void
  Kinfu::update(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg, const sensor_msgs::Image::ConstPtr& depth_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if ((depth_msg->height != caminfo_msg->height) || (depth_msg->width != caminfo_msg->width))
    {
      ROS_ERROR("Image size of input depth and camera info must be same. Depth: (%d, %d), Camera Info: (%d, %d)",
                depth_msg->height, depth_msg->width, caminfo_msg->height, caminfo_msg->width);
      return;
    }

    if (!is_kinfu_initialized_)
    {
      initKinfu(caminfo_msg->height, caminfo_msg->width);
      is_kinfu_initialized_ = true;
    }

    // run kinfu
    {
      pcl::gpu::kinfuLS::KinfuTracker::DepthMap depth_device;

      cv::Mat depth;
      if (depth_msg->encoding == enc::TYPE_32FC1)
      {
        cv::Mat depth_32fc1 = cv_bridge::toCvShare(depth_msg, enc::TYPE_32FC1)->image;
        depth_32fc1 *= 1000.;
        depth_32fc1.convertTo(depth, CV_16UC1);
      }
      else if (depth_msg->encoding == enc::TYPE_16UC1)
      {
        depth = cv_bridge::toCvShare(depth_msg, enc::TYPE_16UC1)->image;
      }
      else
      {
        NODELET_FATAL("Unsupported depth image encoding: %s", depth_msg->encoding.c_str());
        return;
      }
      depth_device.upload(&(depth.data[0]), depth.cols * 2, depth.rows, depth.cols);

      kinfu_->setDepthIntrinsics(/*fx=*/caminfo_msg->K[0], /*fy=*/caminfo_msg->K[4],
                                 /*cx=*/caminfo_msg->K[2], /*cy=*/caminfo_msg->K[5]);
      (*kinfu_)(depth_device);
    }

    if (kinfu_->icpIsLost())
    {
      NODELET_FATAL_THROTTLE(10, "Tracking by ICP in kinect fusion is lost. auto_reset: %d", auto_reset_);
      if (auto_reset_)
      {
        kinfu_->reset();
        is_kinfu_initialized_ = false;
      }
      return;
    }

    // publish kinfu origin
    {
      Eigen::Affine3f camera_to_kinfu_origin = kinfu_->getCameraPose().inverse();
      tf::Transform tf_kinfu_origin;
      tf::transformEigenToTF(camera_to_kinfu_origin, tf_kinfu_origin);
      tf_kinfu_origin.setRotation(tf_kinfu_origin.getRotation().normalized());  // for long-term use
      tf_broadcaster_.sendTransform(
        tf::StampedTransform(tf_kinfu_origin, caminfo_msg->header.stamp,
                             caminfo_msg->header.frame_id, "kinfu_origin"));
    }

    // publish rendered image
    {
      pcl::gpu::kinfuLS::KinfuTracker::View view_device;
      std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host_;
      kinfu_->getImage(view_device);

      int cols;
      view_device.download(view_host_, cols);

      sensor_msgs::Image rendered_image_msg;
      sensor_msgs::fillImage(rendered_image_msg,
                             sensor_msgs::image_encodings::RGB8,
                             view_device.rows(),
                             view_device.cols(),
                             view_device.cols() * 3,
                             reinterpret_cast<unsigned char*>(&view_host_[0]));
      pub_rendered_image_.publish(rendered_image_msg);
    }

    // publish cloud
    {
      pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device_;

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = kinfu_->volume().fetchCloud(cloud_buffer_device_);
      extracted.download(cloud->points);
      cloud->width = static_cast<int>(cloud->points.size());
      cloud->height = 1;

      sensor_msgs::PointCloud2 output_cloud_msg;
      pcl::toROSMsg(*cloud, output_cloud_msg);
      output_cloud_msg.header.stamp = depth_msg->header.stamp;
      output_cloud_msg.header.frame_id = "kinfu_origin";
      pub_cloud_.publish(output_cloud_msg);
    }
  }

  bool
  Kinfu::resetCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    NODELET_INFO("Resetting kinect fusion was requested, so resetting.");
    kinfu_->reset();
    is_kinfu_initialized_ = false;
    return true;
  }

  bool
  Kinfu::saveMeshCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    if (!marching_cubes_)
    {
      marching_cubes_ = pcl::gpu::kinfuLS::MarchingCubes::Ptr(new pcl::gpu::kinfuLS::MarchingCubes());
    }

    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device;
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_device =
      marching_cubes_->run(kinfu_->volume(), triangles_buffer_device);
    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr(new pcl::PolygonMesh());
    mesh_ptr = convertToMesh(triangles_device);

    NODELET_INFO("Saving mesh to: %s.", "mesh.ply");
    pcl::io::savePLYFile("mesh.ply", *mesh_ptr);
    return true;
  }

  boost::shared_ptr<pcl::PolygonMesh>
  Kinfu::convertToMesh(const pcl::gpu::DeviceArray<pcl::PointXYZ>& triangles)
  {
    if (triangles.empty())
    {
      return boost::shared_ptr<pcl::PolygonMesh>();
    }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    cloud.width = static_cast<int>(triangles.size());
    cloud.height = 1;
    triangles.download(cloud.points);
    boost::shared_ptr<pcl::PolygonMesh> mesh_ptr(new pcl::PolygonMesh());
    pcl::toPCLPointCloud2(cloud, mesh_ptr->cloud);
    mesh_ptr->polygons.resize(triangles.size() / 3);
    for (size_t i = 0; i < mesh_ptr->polygons.size(); ++i)
    {
      pcl::Vertices v;
      v.vertices.push_back(i*3+0);
      v.vertices.push_back(i*3+2);
      v.vertices.push_back(i*3+1);
      mesh_ptr->polygons[i] = v;
    }
    return mesh_ptr;
  }
}  // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::Kinfu, nodelet::Nodelet);
