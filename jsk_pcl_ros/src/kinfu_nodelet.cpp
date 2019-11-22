/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Kentaro Wada and JSK Lab
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
 *   * Neither the name of Kentaro Wada and JSK Lab nor the names of its
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
#include <boost/filesystem.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/io/obj_io.h>

#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_msgs/TrackingStatus.h>
#include <jsk_recognition_msgs/SaveMesh.h>
#include <jsk_recognition_utils/pcl_ros_util.h>
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
    pnh_->param("auto_reset", auto_reset_, true);
    pnh_->param("integrate_color", integrate_color_, false);
    pnh_->param("slam", slam_, false);
    pnh_->param<std::string>("fixed_frame_id", fixed_frame_id_, "odom_init");
    pnh_->param("n_textures", n_textures_, -1);
    pnh_->param("volume_size", volume_size_, pcl::device::kinfuLS::VOLUME_SIZE);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> >(*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f = boost::bind(&Kinfu::configCallback, this, _1, _2);
    srv_->setCallback(f);

    tf_listener_.reset(new tf::TransformListener());

    pub_camera_pose_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output", 1);
    pub_cloud_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output/cloud", 1);
    pub_depth_ = advertise<sensor_msgs::Image>(*pnh_, "output/depth", 1);
    pub_rendered_image_ = advertise<sensor_msgs::Image>(*pnh_, "output/rendered_image", 1);
    pub_status_ = advertise<jsk_recognition_msgs::TrackingStatus>(*pnh_, "output/status", 1);

    srv_reset_ = pnh_->advertiseService("reset", &Kinfu::resetCallback, this);
    srv_save_mesh_ = pnh_->advertiseService("save_mesh", &Kinfu::saveMeshCallback, this);
    srv_save_mesh_with_context_ = pnh_->advertiseService(
      "save_mesh_with_context", &Kinfu::saveMeshWithContextCallback, this);

    onInitPostProcess();
  }

  void
  Kinfu::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    save_dir_ = config.save_dir;
  }

  void
  Kinfu::initKinfu(const sensor_msgs::CameraInfo::ConstPtr& caminfo_msg)
  {
    pcl::gpu::setDevice(device_);
    pcl::gpu::printShortCudaDeviceInfo(device_);

    /* below are copied from pcl/gpu/kinfu_large_scale/src/kinfuLS_app.cpp */

    float shift_distance = pcl::device::kinfuLS::DISTANCE_THRESHOLD;
    Eigen::Vector3f volume_size_vector = Eigen::Vector3f::Constant(volume_size_/*meters*/);
    if (shift_distance > 2.5 * volume_size_)
    {
      NODELET_WARN("WARNING Shifting distance (%.2f) is very large compared to the volume size (%.2f).",
                   shift_distance, volume_size_);
    }

    kinfu_.reset(new pcl::gpu::kinfuLS::KinfuTracker(
      volume_size_vector, shift_distance, caminfo_msg->height, caminfo_msg->width));

    Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
    Eigen::Vector3f t = volume_size_vector * 0.5f - Eigen::Vector3f(0, 0, volume_size_vector(2) / 2 * 1.2f);

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

    if (!kinfu_)
    {
      initKinfu(caminfo_msg);
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
        if (color_msg->encoding == enc::BGR8)
        {
          cv_bridge::CvImagePtr tmp_image_ptr_ = cv_bridge::toCvCopy(color_msg, enc::RGB8);
          colors_device_.upload(&(tmp_image_ptr_->toImageMsg()->data[0]),
                                color_msg->step, color_msg->height, color_msg->width);
        }
        else
        {
          colors_device_.upload(&(color_msg->data[0]), color_msg->step, color_msg->height, color_msg->width);
        }

        (*kinfu_)(depth_device, colors_device_);
      }
      else
      {
        (*kinfu_)(depth_device);
      }
      frame_idx_++;
    }

    jsk_recognition_msgs::TrackingStatus status;
    status.header = caminfo_msg->header;
    if (kinfu_->icpIsLost())
    {
      NODELET_FATAL_THROTTLE(10, "Tracking by ICP in kinect fusion is lost. auto_reset: %d", auto_reset_);
      if (auto_reset_)
      {
        kinfu_.reset();
        frame_idx_ = 0;
        cameras_.clear();
      }
      status.is_lost = true;
      pub_status_.publish(status);
      return;
    }
    status.is_lost = false;
    pub_status_.publish(status);

    // save texture
    if (integrate_color_ && (frame_idx_ % pcl::device::kinfuLS::SNAPSHOT_RATE == 1))
    {
      cv::Mat texture = cv_bridge::toCvCopy(color_msg, color_msg->encoding)->image;
      if (color_msg->encoding == enc::RGB8)
      {
        cv::cvtColor(texture, texture, cv::COLOR_RGB2BGR);
      }
      textures_.push_back(texture);

      pcl::TextureMapping<pcl::PointXYZ>::Camera camera;
      camera.pose = kinfu_->getCameraPose();
      camera.focal_length = caminfo_msg->K[0];  // must be equal to caminfo_msg->K[4]
      camera.height = caminfo_msg->height;
      camera.width = caminfo_msg->width;
      cameras_.push_back(camera);
    }

    // publish kinfu origin and slam
    {
      Eigen::Affine3f camera_pose = kinfu_->getCameraPose();

      // publish camera pose
      if (pub_camera_pose_.getNumSubscribers() > 0)
      {
        geometry_msgs::PoseStamped camera_pose_msg;
        tf::poseEigenToMsg(camera_pose, camera_pose_msg.pose);
        camera_pose_msg.header.stamp = caminfo_msg->header.stamp;
        camera_pose_msg.header.frame_id = "kinfu_origin";
        pub_camera_pose_.publish(camera_pose_msg);
      }

      Eigen::Affine3f camera_to_kinfu_origin = camera_pose.inverse();
      tf::Transform tf_camera_to_kinfu_origin;
      tf::transformEigenToTF(camera_to_kinfu_origin, tf_camera_to_kinfu_origin);
      tf_camera_to_kinfu_origin.setRotation(tf_camera_to_kinfu_origin.getRotation().normalized());
      tf_broadcaster_.sendTransform(
        tf::StampedTransform(tf_camera_to_kinfu_origin, caminfo_msg->header.stamp,
                             caminfo_msg->header.frame_id, "kinfu_origin"));

      if (slam_)
      {
        // use kinfu as slam, and publishes tf: map -> fixed_frame_id_ (usually odom_init)
        try
        {
          tf::StampedTransform tf_odom_to_camera;
          tf_listener_->lookupTransform(
            fixed_frame_id_, caminfo_msg->header.frame_id, ros::Time(0), tf_odom_to_camera);
          Eigen::Affine3f odom_to_camera;
          tf::transformTFToEigen(tf_odom_to_camera, odom_to_camera);

          if (frame_idx_ == 1)
          {
            odom_init_to_kinfu_origin_ = odom_to_camera * camera_to_kinfu_origin;
          }

          Eigen::Affine3f map_to_odom;
          // map_to_odom * odom_to_camera * camera_to_kinfu_origin == odom_init_to_kinfu_origin_
          map_to_odom = odom_init_to_kinfu_origin_ * (odom_to_camera * camera_to_kinfu_origin).inverse();
          tf::StampedTransform tf_map_to_odom;
          tf::transformEigenToTF(map_to_odom, tf_map_to_odom);
          tf_map_to_odom.setRotation(tf_map_to_odom.getRotation().normalized());
          tf_broadcaster_.sendTransform(
            tf::StampedTransform(tf_map_to_odom, caminfo_msg->header.stamp, "map", fixed_frame_id_));
        }
        catch (tf::TransformException e)
        {
          NODELET_FATAL("%s", e.what());
        }
      }
    }

    // publish depth image
    if (pub_depth_.getNumSubscribers() > 0)
    {
      pcl::gpu::kinfuLS::KinfuTracker::DepthMap depth_gpu;
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
      raycaster_->generateDepthImage(depth_gpu);

      int cols;
      std::vector<unsigned short> data;
      depth_gpu.download(data, cols);

      sensor_msgs::Image output_depth_msg;
      sensor_msgs::fillImage(output_depth_msg,
                             enc::TYPE_16UC1,
                             depth_gpu.rows(),
                             depth_gpu.cols(),
                             depth_gpu.cols() * 2,
                             reinterpret_cast<unsigned short*>(&data[0]));
      output_depth_msg.header = caminfo_msg->header;
      pub_depth_.publish(output_depth_msg);
    }

    // publish rendered image
    if (pub_rendered_image_.getNumSubscribers() > 0)
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
    if (pub_cloud_.getNumSubscribers() > 0)
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
    boost::mutex::scoped_lock lock(mutex_);
    kinfu_.reset();
    textures_.clear();
    cameras_.clear();
    NODELET_INFO("Reset kinect fusion by request.");
    return true;
  }

  bool
  Kinfu::saveMeshWithContextCallback(
    jsk_recognition_msgs::SaveMesh::Request& req, jsk_recognition_msgs::SaveMesh::Response& res)
  {
    pcl::PolygonMesh polygon_mesh = createPolygonMesh(req.box, req.ground_frame_id);

    boost::filesystem::path dir(integrate_color_ ? save_dir_ + "/textures" : save_dir_);
    if (boost::filesystem::create_directories(dir))
    {
      NODELET_INFO("Created save_dir: %s", save_dir_.c_str());
    }

    std::string out_file = save_dir_ + "/mesh.obj";
    if (integrate_color_ && n_textures_ != 0)
    {
      pcl::TextureMesh texture_mesh;
      if (n_textures_ > 0)
      {
        std::vector<cv::Mat> textures(textures_.end() - n_textures_, textures_.end());
        pcl::texture_mapping::CameraVector cameras(cameras_.end() - n_textures_, cameras_.end());
        texture_mesh = convertToTextureMesh(polygon_mesh, textures, cameras);
      }
      else
      {
        texture_mesh = convertToTextureMesh(polygon_mesh, textures_, cameras_);
      }
      pcl::io::saveOBJFile(out_file, texture_mesh, 5);
    }
    else
    {
      pcl::io::saveOBJFile(out_file, polygon_mesh);
    }
    NODELET_INFO("Saved mesh file: %s", out_file.c_str());
    return true;
  }

  bool
  Kinfu::saveMeshCallback(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
  {
    pcl::PolygonMesh polygon_mesh = createPolygonMesh();

    boost::filesystem::path dir(integrate_color_ ? save_dir_ + "/textures" : save_dir_);
    if (boost::filesystem::create_directories(dir))
    {
      NODELET_INFO("Created save_dir: %s", save_dir_.c_str());
    }

    std::string out_file = save_dir_ + "/mesh.obj";
    if (integrate_color_ && n_textures_ != 0)
    {
      pcl::TextureMesh texture_mesh;
      if (n_textures_ > 0)
      {
        std::vector<cv::Mat> textures(textures_.end() - n_textures_, textures_.end());
        pcl::texture_mapping::CameraVector cameras(cameras_.end() - n_textures_, cameras_.end());
        texture_mesh = convertToTextureMesh(polygon_mesh, textures, cameras);
      }
      else
      {
        texture_mesh = convertToTextureMesh(polygon_mesh, textures_, cameras_);
      }
      pcl::io::saveOBJFile(out_file, texture_mesh, 5);
    }
    else
    {
      pcl::io::saveOBJFile(out_file, polygon_mesh);
    }
    NODELET_INFO("Saved mesh file: %s", out_file.c_str());
    return true;
  }

  pcl::TextureMesh
  Kinfu::convertToTextureMesh(const pcl::PolygonMesh& triangles,
                              const std::vector<cv::Mat> textures,
                              pcl::texture_mapping::CameraVector cameras)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(triangles.cloud, *cloud);

    // Create the texturemesh object that will contain our UV-mapped mesh
    pcl::TextureMesh mesh;
    mesh.cloud = triangles.cloud;
    std::vector< pcl::Vertices> polygon_1;

    // push faces into the texturemesh object
    polygon_1.resize (triangles.polygons.size ());
    for(size_t i =0; i < triangles.polygons.size (); ++i)
    {
      polygon_1[i] = triangles.polygons[i];
    }
    mesh.tex_polygons.push_back(polygon_1);
    NODELET_INFO("Input mesh contains %lu faces, %lu vertices and %lu textures.",
                 mesh.tex_polygons[0].size(), cloud->points.size(), cameras.size());

    // Create materials for each texture (and one extra for occluded faces)
    mesh.tex_materials.resize(cameras.size () + 1);
    for(int i = 0 ; i <= cameras.size() ; ++i)
    {
      pcl::TexMaterial mesh_material;
      mesh_material.tex_Ka.r = 0.2f;
      mesh_material.tex_Ka.g = 0.2f;
      mesh_material.tex_Ka.b = 0.2f;

      mesh_material.tex_Kd.r = 0.8f;
      mesh_material.tex_Kd.g = 0.8f;
      mesh_material.tex_Kd.b = 0.8f;

      mesh_material.tex_Ks.r = 1.0f;
      mesh_material.tex_Ks.g = 1.0f;
      mesh_material.tex_Ks.b = 1.0f;

      mesh_material.tex_d = 1.0f;
      mesh_material.tex_Ns = 75.0f;
      mesh_material.tex_illum = 2;

      std::stringstream tex_name;
      tex_name << "material_" << i;
      tex_name >> mesh_material.tex_name;

      if (i < cameras.size ())
      {
        std::stringstream ss;
        ss << "textures/" << i << ".jpg";
        std::string texture_file = ss.str();
        cv::imwrite(save_dir_ + "/" + texture_file, textures[i]);
        cameras[i].texture_file = texture_file;
        mesh_material.tex_file = texture_file;
      }
      else
      {
        std::string texture_file = "textures/occluded.jpg";
        cv::imwrite(save_dir_ + "/" + texture_file,
                    cv::Mat::zeros(textures[0].rows, textures[0].cols, CV_8UC1));
        mesh_material.tex_file = texture_file;
      }

      mesh.tex_materials[i] = mesh_material;
    }

    // sort faces
    pcl::TextureMapping<pcl::PointXYZ> tm;  // TextureMapping object that will perform the sort
    tm.textureMeshwithMultipleCameras(mesh, cameras);

    // compute normals for the mesh
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    n.setInputCloud(cloud);
    n.setSearchMethod(tree);
    n.setKSearch(20);
    n.compute(*normals);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    pcl::toPCLPointCloud2(*cloud_with_normals, mesh.cloud);

    return mesh;
  }

  pcl::PolygonMesh
  Kinfu::createPolygonMesh()
  {
    // create triangles
    if (!marching_cubes_)
    {
      marching_cubes_ = pcl::gpu::kinfuLS::MarchingCubes::Ptr(new pcl::gpu::kinfuLS::MarchingCubes());
    }
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device;
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_device =
      marching_cubes_->run(kinfu_->volume(), triangles_buffer_device);

    if (triangles_device.empty())
    {
      return pcl::PolygonMesh();
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    cloud->width = static_cast<int>(triangles_device.size());
    cloud->height = 1;
    cloud->header.frame_id = "kinfu_origin";
    triangles_device.download(cloud->points);

    pcl::PolygonMesh mesh;
    pcl::toPCLPointCloud2(*cloud, mesh.cloud);
    mesh.polygons.resize(triangles_device.size() / 3);
    for (size_t i = 0; i < mesh.polygons.size(); ++i)
    {
      pcl::Vertices v;
      v.vertices.push_back(i*3+0);
      v.vertices.push_back(i*3+2);
      v.vertices.push_back(i*3+1);
      mesh.polygons[i] = v;
    }
    return mesh;
  }

  pcl::PolygonMesh
  Kinfu::createPolygonMesh(const jsk_recognition_msgs::BoundingBox& box_msg, const std::string& ground_frame_id)
  {
    // create triangles
    if (!marching_cubes_)
    {
      marching_cubes_ = pcl::gpu::kinfuLS::MarchingCubes::Ptr(new pcl::gpu::kinfuLS::MarchingCubes());
    }
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_buffer_device;
    pcl::gpu::DeviceArray<pcl::PointXYZ> triangles_device =
      marching_cubes_->run(kinfu_->volume(), triangles_buffer_device);

    if (triangles_device.empty())
    {
      return pcl::PolygonMesh();
    }

    // get original polygon mesh
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    cloud->width = static_cast<int>(triangles_device.size());
    cloud->height = 1;
    cloud->header.frame_id = "kinfu_origin";
    triangles_device.download(cloud->points);

    // get tf
    Eigen::Affine3f kinfu_origin_to_box = Eigen::Affine3f::Identity();
    if (box_msg.header.frame_id != "kinfu_origin")
    {
      tf::StampedTransform tf_kinfu_origin_to_box;
      tf_listener_->lookupTransform("kinfu_origin", box_msg.header.frame_id, ros::Time(0), tf_kinfu_origin_to_box);
      tf::transformTFToEigen(tf_kinfu_origin_to_box, kinfu_origin_to_box);
    }

    // transform bounding box to kinfu frame
    jsk_recognition_msgs::BoundingBox transformed_box_msg;
    transformed_box_msg.dimensions = box_msg.dimensions;
    Eigen::Affine3f box_pose;
    tf::poseMsgToEigen(box_msg.pose, box_pose);
    box_pose = kinfu_origin_to_box * box_pose;
    tf::poseEigenToMsg(box_pose, transformed_box_msg.pose);
    transformed_box_msg.header.frame_id = "kinfu_origin";

    // crop cloud
    std::vector<int> indices;
    jsk_recognition_utils::cropPointCloud<pcl::PointXYZ>(cloud, transformed_box_msg, &indices);

    // generate filtered polygon mesh
    pcl::PolygonMesh mesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_box(new pcl::PointCloud<pcl::PointXYZ>());
    cloud_in_box->width = static_cast<int>(indices.size());
    cloud_in_box->height = 1;
    cloud_in_box->points.resize(indices.size());
    for (size_t i = 0; i < indices.size(); i++)
    {
      cloud_in_box->points[i] = cloud->points[indices[i]];
      if (indices[i] % 3 == 0 && (indices[i] + 1) == indices[i  + 1] && (indices[i] + 2) == indices[i + 2])
      {
        pcl::Vertices v;
        v.vertices.push_back(i + 0);
        v.vertices.push_back(i + 2);
        v.vertices.push_back(i + 1);
        mesh.polygons.push_back(v);
      }
    }

    // fill face occluded by putting object on plane (ex. tabletop)
    if (!ground_frame_id.empty())
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ground_frame(new pcl::PointCloud<pcl::PointXYZ>());
      tf::StampedTransform tf_transform;
      tf_listener_->lookupTransform(ground_frame_id, "kinfu_origin", ros::Time(0), tf_transform);
      Eigen::Affine3f transform;
      tf::transformTFToEigen(tf_transform, transform);
      pcl::transformPointCloud(*cloud_in_box, *cloud_ground_frame, transform);

      pcl::PointXYZ min_pt, max_pt;
      pcl::getMinMax3D(*cloud_ground_frame, min_pt, max_pt);

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_occluded(new pcl::PointCloud<pcl::PointXYZ>());
      for (size_t i = 0; i < cloud_ground_frame->points.size(); i++)
      {
        pcl::PointXYZ pt_visible = cloud_ground_frame->points[i];
        pcl::PointXYZ pt_occluded(pt_visible.x, pt_visible.y, min_pt.z);
        cloud_occluded->points.push_back(pt_occluded);
        if (i >= 3)
        {
          pcl::Vertices v;
          v.vertices.push_back(cloud_in_box->width + i - 2);
          v.vertices.push_back(cloud_in_box->width + i - 1);
          v.vertices.push_back(cloud_in_box->width + i - 0);
          mesh.polygons.push_back(v);
        }
      }
      cloud_occluded->width = cloud_occluded->points.size();
      cloud_occluded->height = 1;
      pcl::transformPointCloud(*cloud_occluded, *cloud_occluded, transform.inverse());
      *cloud_in_box = *cloud_in_box + *cloud_occluded;
    }

    pcl::toPCLPointCloud2(*cloud_in_box, mesh.cloud);
    return mesh;
  }
}  // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::Kinfu, nodelet::Nodelet);
