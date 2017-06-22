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

// TODO(wkentaro): Move to upstream.
namespace pcl
{
  void
  concatenateFields(PointCloud<PointXYZ>& cloud_xyz, PointCloud<RGB>& cloud_rgb, PointCloud<PointXYZRGB>& cloud)
  {
    if (cloud_xyz.points.size() != cloud_rgb.points.size())
    {
      std::cout << "Clouds being concatenated must have same size." << std::endl;
      return;
    }
    cloud.points.resize(cloud_xyz.points.size());
    for (size_t i = 0; i < cloud_xyz.points.size(); i++)
    {
      cloud.points[i].x = cloud_xyz.points[i].x;
      cloud.points[i].y = cloud_xyz.points[i].y;
      cloud.points[i].z = cloud_xyz.points[i].z;
      cloud.points[i].r = cloud_rgb.points[i].r;
      cloud.points[i].g = cloud_rgb.points[i].g;
      cloud.points[i].b = cloud_rgb.points[i].b;
    }
    cloud.width = cloud_xyz.width;
    cloud.height = cloud_xyz.height;
  }
}

namespace jsk_pcl_ros
{
  void
  Kinfu::onInit()
  {
    ConnectionBasedNodelet::onInit();
    always_subscribe_ = true;  // for mapping

    pnh_->param("device", device_, 0);
    pnh_->param("auto_reset", auto_reset_, false);
    pnh_->param("integrate_color", integrate_color_, false);

    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_generated_depth_ = advertise<sensor_msgs::Image>(*pnh_, "output/generated_depth", 1);
    pub_rendered_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/rendered_image", 1);

    srv_reset_ = pnh_->advertiseService("reset", &Kinfu::resetCallback, this);
    srv_save_mesh_ = pnh_->advertiseService("save_mesh", &Kinfu::saveMeshCallback, this);

    onInitPostProcess();
  }

  void
  Kinfu::initKinfu(const int height, const int width)
  {
    pcl::gpu::setDevice(device_);
    pcl::gpu::printShortCudaDeviceInfo(device_);

    /* below are copied from pcl/gpu/kinfu_large_scale/src/kinfuLS_app.cpp */

    float vsz = pcl::device::kinfuLS::VOLUME_SIZE;
    float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
    Eigen::Vector3f volume_size = Eigen::Vector3f::Constant(vsz/*meters*/);
    if (shift_distance > 2.5 * vsz)
    {
      NODELET_WARN("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).",
                   shift_distance, vsz);
    }

    kinfu_ = new pcl::gpu::kinfuLS::KinfuTracker(volume_size, shift_distance, height, width);

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = volume_size * 0.5f - Eigen::Vector3f(0, 0, volume_size(2) / 2 * 1.2f);

    Eigen::Affine3f pose = Eigen::Translation3f(t) * Eigen::AngleAxisf(R);

    kinfu_->setInitialCameraPose(pose);
    kinfu_->volume().setTsdfTruncDist(0.030f/*meters*/);
    kinfu_->setIcpCorespFilteringParams(0.1f/*meters*/, sin(pcl::deg2rad(20.f)));
    //kinfu_->setDepthTruncationForICP(3.0f/*meters*/);
    kinfu_->setCameraMovementThreshold(0.001f);

    if (integrate_color_)
    {
      const int max_color_integration_weight = 2;
      kinfu_->initColorIntegration(max_color_integration_weight);
    }
  }

  void
  Kinfu::subscribe()
  {
    int queue_size;
    pnh_->param("queue_size", queue_size, 10);

    sub_camera_info_.subscribe(*pnh_, "input/camera_info", 1);
    sub_depth_.subscribe(*pnh_, "input/depth", 1);
    if (integrate_color_)
    {
      sub_color_.subscribe(*pnh_, "input/color", 1);
      sync_with_color_.reset(new message_filters::Synchronizer<SyncPolicyWithColor>(queue_size));
      sync_with_color_->connectInput(sub_camera_info_, sub_depth_, sub_color_);
      sync_with_color_->registerCallback(boost::bind(&Kinfu::update, this, _1, _2, _3));
    }
    else
    {
      sync_.reset(new message_filters::Synchronizer<SyncPolicy>(queue_size));
      sync_->connectInput(sub_camera_info_, sub_depth_);
      sync_->registerCallback(boost::bind(&Kinfu::update, this, _1, _2));
    }
  }

  void
  Kinfu::unsubscribe()
  {
  }

  void
  Kinfu::update(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg,
                const sensor_msgs::Image::ConstPtr& depth_msg)
  {
    update(caminfo_msg, depth_msg, sensor_msgs::ImageConstPtr());
  }

  void
  Kinfu::update(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg,
                const sensor_msgs::Image::ConstPtr& depth_msg,
                const sensor_msgs::Image::ConstPtr& color_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if ((depth_msg->height != caminfo_msg->height) || (depth_msg->width != caminfo_msg->width))
    {
      ROS_ERROR("Image size of input depth and camera info must be same. Depth: (%d, %d), Camera Info: (%d, %d)",
                depth_msg->height, depth_msg->width, caminfo_msg->height, caminfo_msg->width);
      return;
    }
    if (integrate_color_ && ((color_msg->height != caminfo_msg->height) || (color_msg->width != color_msg->width)))
    {
      ROS_ERROR("Image size of input color image and camera info must be same. Color: (%d, %d), Camera Info: (%d, %d)",
                color_msg->height, color_msg->width, caminfo_msg->height, caminfo_msg->width);
      return;
    }

    if (!is_kinfu_initialized_)
    {
      initKinfu(caminfo_msg->height, caminfo_msg->width);
      is_kinfu_initialized_ = true;
    }

    // run kinfu
    {
      kinfu_->setDepthIntrinsics(/*fx=*/caminfo_msg->K[0], /*fy=*/caminfo_msg->K[4],
                                 /*cx=*/caminfo_msg->K[2], /*cy=*/caminfo_msg->K[5]);

      // depth: 32fc1 -> 16uc1
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

      // depth: cpu -> gpu
      pcl::gpu::kinfuLS::KinfuTracker::DepthMap depth_device;
      depth_device.upload(&(depth.data[0]), depth.cols * 2, depth.rows, depth.cols);


      if (integrate_color_)
      {
        // color: cpu -> gpu
        colors_device_.upload(&(color_msg->data[0]), color_msg->step, color_msg->height, color_msg->width);

        (*kinfu_)(depth_device, colors_device_);
      }
      else
      {
        (*kinfu_)(depth_device);
      }
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

    // publish generated depth image
    {
      pcl::gpu::kinfuLS::KinfuTracker::DepthMap generated_depth;
      Eigen::Affine3f camera_pose = kinfu_->getCameraPose();
      if (!raycaster_)
      {
        raycaster_ = pcl::gpu::kinfuLS::RayCaster::Ptr(
          new pcl::gpu::kinfuLS::RayCaster(
            /*height=*/kinfu_->rows(), /*width=*/kinfu_->cols(),
            /*fx=*/caminfo_msg->K[0], /*fy=*/caminfo_msg->K[4],
            /*cx=*/caminfo_msg->K[2], /*cy=*/caminfo_msg->K[5]));
      }
      raycaster_->run(kinfu_->volume(), camera_pose, kinfu_->getCyclicalBufferStructure());
      raycaster_->generateDepthImage(generated_depth);

      int cols;
      std::vector<unsigned short> data;
      generated_depth.download(data, cols);

      sensor_msgs::Image generated_depth_msg;
      sensor_msgs::fillImage(generated_depth_msg,
                             enc::TYPE_16UC1,
                             generated_depth.rows(),
                             generated_depth.cols(),
                             generated_depth.cols() * 2,
                             reinterpret_cast<unsigned short*>(&data[0]));
      generated_depth_msg.header = caminfo_msg->header;
      pub_generated_depth_.publish(generated_depth_msg);
    }

    // publish rendered image
    {
      pcl::gpu::kinfuLS::KinfuTracker::View view_device;
      std::vector<pcl::gpu::kinfuLS::PixelRGB> view_host;
      kinfu_->getImage(view_device);

      if (integrate_color_)
      {
        pcl::gpu::kinfuLS::paint3DView(colors_device_, view_device);
      }

      int cols;
      view_device.download(view_host, cols);

      sensor_msgs::Image rendered_image_msg;
      sensor_msgs::fillImage(rendered_image_msg,
                             enc::RGB8,
                             view_device.rows(),
                             view_device.cols(),
                             view_device.cols() * 3,
                             reinterpret_cast<unsigned char*>(&view_host[0]));
      rendered_image_msg.header = caminfo_msg->header;
      pub_rendered_image_.publish(rendered_image_msg);
    }

    // publish cloud
    {
      pcl::gpu::DeviceArray<pcl::PointXYZ> cloud_buffer_device;
      pcl::gpu::DeviceArray<pcl::PointXYZ> extracted = kinfu_->volume().fetchCloud(cloud_buffer_device);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
      extracted.download(cloud_xyz->points);
      cloud_xyz->width = static_cast<int>(cloud_xyz->points.size());
      cloud_xyz->height = 1;

      sensor_msgs::PointCloud2 output_cloud_msg;
      if (integrate_color_)
      {
        pcl::gpu::DeviceArray<pcl::RGB> point_colors_device;
        kinfu_->colorVolume().fetchColors(extracted, point_colors_device);

        pcl::PointCloud<pcl::RGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::RGB>());
        point_colors_device.download(cloud_rgb->points);
        cloud_rgb->width = static_cast<int>(cloud_rgb->points.size());
        cloud_rgb->height = 1;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        cloud->points.resize(cloud_xyz->width);
        pcl::concatenateFields(*cloud_xyz, *cloud_rgb, *cloud);
        pcl::toROSMsg(*cloud, output_cloud_msg);
      }
      else
      {
        pcl::toROSMsg(*cloud_xyz, output_cloud_msg);
      }
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
