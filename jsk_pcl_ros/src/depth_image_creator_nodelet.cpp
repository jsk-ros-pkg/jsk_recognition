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

#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_utils/pcl_ros_util.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include "jsk_pcl_ros/depth_image_creator.h"

void jsk_pcl_ros::DepthImageCreator::onInit () {
  NODELET_INFO("[%s::onInit]", getName().c_str());
  ConnectionBasedNodelet::onInit();
  tf_listener_ = TfListenerSingleton::getInstance();
  // scale_depth
  pnh_->param("scale_depth", scale_depth, 1.0);
  ROS_INFO("scale_depth : %f", scale_depth);

  // use fixed transform
  pnh_->param("use_fixed_transform", use_fixed_transform, false);
  ROS_INFO("use_fixed_transform : %d", use_fixed_transform);

  pnh_->param("use_service", use_service, false);
  ROS_INFO("use_service : %d", use_service);

  pnh_->param("use_asynchronous", use_asynchronous, false);
  ROS_INFO("use_asynchronous : %d", use_asynchronous);

  pnh_->param("use_approximate", use_approximate, false);
  ROS_INFO("use_approximate : %d", use_approximate);

  pnh_->param("fill_value", fill_value, std::numeric_limits<float>::quiet_NaN());
  pnh_->param("organize_cloud", organize_cloud_, false);

  pnh_->param("info_throttle", info_throttle_, 0);
  info_counter_ = 0;
  pnh_->param("max_queue_size", max_queue_size_, 3);
  // maybe below queue_size can always be 1,
  // but we set max_queue_size_ for backward compatibility.
  pnh_->param("max_pub_queue_size", max_pub_queue_size_, max_queue_size_);
  pnh_->param("max_sub_queue_size", max_sub_queue_size_, max_queue_size_);
  // set transformation
  std::vector<double> trans_pos(3, 0);
  std::vector<double> trans_quat(4, 0); trans_quat[3] = 1.0;
  if (pnh_->hasParam("translation")) {
    jsk_topic_tools::readVectorParameter(*pnh_, "translation", trans_pos);
  }
  if (pnh_->hasParam("rotation")) {
    jsk_topic_tools::readVectorParameter(*pnh_, "rotation", trans_quat);
  }
  tf::Quaternion btq(trans_quat[0], trans_quat[1], trans_quat[2], trans_quat[3]);
  tf::Vector3 btp(trans_pos[0], trans_pos[1], trans_pos[2]);
  fixed_transform.setOrigin(btp);
  fixed_transform.setRotation(btq);
  pnh_->param("tf_duration", tf_duration_, 0.001);

  pub_depth_ = advertise<sensor_msgs::Image> (*pnh_, "output", max_pub_queue_size_);
  pub_image_ = advertise<sensor_msgs::Image> (*pnh_, "output_image", max_pub_queue_size_);
  pub_cloud_ = advertise<PointCloud>(*pnh_, "output_cloud", max_pub_queue_size_);
  pub_disp_image_ = advertise<stereo_msgs::DisparityImage> (*pnh_, "output_disp", max_pub_queue_size_);
  
  if (use_service) {
    service_ = pnh_->advertiseService("set_point_cloud",
                                      &DepthImageCreator::service_cb, this);
  }
  onInitPostProcess();
}

void jsk_pcl_ros::DepthImageCreator::subscribe() {
  if (!use_service) {
    if (use_asynchronous) {
      sub_as_info_ = pnh_->subscribe<sensor_msgs::CameraInfo> ("info", max_sub_queue_size_,
                                                               &DepthImageCreator::callback_info, this);
      sub_as_cloud_ = pnh_->subscribe<sensor_msgs::PointCloud2> ("input", max_sub_queue_size_,
                                                                 &DepthImageCreator::callback_cloud, this);
    } else {
      sub_info_.subscribe(*pnh_, "info", max_sub_queue_size_);
      sub_cloud_.subscribe(*pnh_, "input", max_sub_queue_size_);

      if (use_approximate) {
        sync_inputs_a_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > (max_queue_size_);
        sync_inputs_a_->connectInput (sub_info_, sub_cloud_);
        sync_inputs_a_->registerCallback (bind (&DepthImageCreator::callback_sync, this, _1, _2));
      } else {
        sync_inputs_e_ = boost::make_shared <message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > (max_queue_size_);
        sync_inputs_e_->connectInput (sub_info_, sub_cloud_);
        sync_inputs_e_->registerCallback (bind (&DepthImageCreator::callback_sync, this, _1, _2));
      }
    }
  } else {
    // not continuous
    sub_as_info_ = pnh_->subscribe<sensor_msgs::CameraInfo> ("info", max_sub_queue_size_,
                                                             &DepthImageCreator::callback_info, this);
   
  }
}

void jsk_pcl_ros::DepthImageCreator::unsubscribe() {
  if (!use_service) {
    if (use_asynchronous) {
      sub_as_info_.shutdown();
      sub_as_cloud_.shutdown();
    }
    else {
      sub_info_.unsubscribe();
      sub_cloud_.unsubscribe();
    }
  } else {
    // not continuous
    sub_as_info_.shutdown();
  }
}

bool jsk_pcl_ros::DepthImageCreator::service_cb (std_srvs::Empty::Request &req,
                                                 std_srvs::Empty::Response &res) {
  return true;
}

void jsk_pcl_ros::DepthImageCreator::callback_sync(const sensor_msgs::CameraInfoConstPtr& info,
                                                   const sensor_msgs::PointCloud2ConstPtr& pcloud2) {
  ROS_DEBUG("DepthImageCreator::callback_sync");
  publish_points(info, pcloud2);
}

void jsk_pcl_ros::DepthImageCreator::callback_cloud(const sensor_msgs::PointCloud2ConstPtr& pcloud2) {
  ROS_DEBUG("DepthImageCreator::callback_cloud");
  boost::mutex::scoped_lock lock(this->mutex_points);
  points_ptr_ = pcloud2;
}

void jsk_pcl_ros::DepthImageCreator::callback_info(const sensor_msgs::CameraInfoConstPtr& info) {
  ROS_DEBUG("DepthImageCreator::callback_info");
  boost::mutex::scoped_lock lock(this->mutex_points);
  if( info_counter_++ >= info_throttle_ ) {
    info_counter_ = 0;
  } else {
    return;
  }
  if (points_ptr_) {
    publish_points(info, points_ptr_);
  }
}

void jsk_pcl_ros::DepthImageCreator::publish_points(const sensor_msgs::CameraInfoConstPtr& info,
                                                    const sensor_msgs::PointCloud2ConstPtr& pcloud2) {
  sensor_msgs::PointCloud2Ptr pcloud2_ptr(new sensor_msgs::PointCloud2(*pcloud2));
  if (!jsk_recognition_utils::hasField("rgb", *pcloud2_ptr)) {
    sensor_msgs::PointCloud2Modifier pc_mod(*pcloud2_ptr);
    pc_mod.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                   "y", 1, sensor_msgs::PointField::FLOAT32,
                                   "z", 1, sensor_msgs::PointField::FLOAT32,
                                   "rgb", 1, sensor_msgs::PointField::FLOAT32);
  }

  ROS_DEBUG("DepthImageCreator::publish_points");
  if (!pcloud2_ptr)  return;
  bool proc_cloud = true, proc_depth = true, proc_image = true, proc_disp = true;
  if ( pub_cloud_.getNumSubscribers()==0 ) {
    proc_cloud = false;
  }
  if ( pub_depth_.getNumSubscribers()==0 ) {
    proc_depth = false;
  }
  if ( pub_image_.getNumSubscribers()==0 ) {
    proc_image = false;
  }
  if ( pub_disp_image_.getNumSubscribers()==0 ) {
    proc_disp = false;
  }
  if( !proc_cloud && !proc_depth && !proc_image && !proc_disp) return;

  int width = info->width;
  int height = info->height;
  float fx = info->P[0];
  float cx = info->P[2];
  float tx = info->P[3];
  float fy = info->P[5];
  float cy = info->P[6];

  Eigen::Affine3f sensorPose;
  {
    tf::StampedTransform transform;
    if(use_fixed_transform) {
      transform = fixed_transform;
    } else {
      try {
        tf_listener_->waitForTransform(pcloud2_ptr->header.frame_id,
                                       info->header.frame_id,
                                       info->header.stamp,
                                       ros::Duration(tf_duration_));
        tf_listener_->lookupTransform(pcloud2_ptr->header.frame_id,
                                      info->header.frame_id,
                                      info->header.stamp, transform);
      }
      catch ( std::runtime_error e ) {
        ROS_ERROR("%s",e.what());
        return;
      }
    }
    tf::Vector3 p = transform.getOrigin();
    tf::Quaternion q = transform.getRotation();
    sensorPose = (Eigen::Affine3f)Eigen::Translation3f(p.getX(), p.getY(), p.getZ());
    Eigen::Quaternion<float> rot(q.getW(), q.getX(), q.getY(), q.getZ());
    sensorPose = sensorPose * rot;

    if (tx != 0.0) {
      Eigen::Affine3f trans = (Eigen::Affine3f)Eigen::Translation3f(-tx/fx , 0, 0);
      sensorPose = sensorPose * trans;
    }
#if 0 // debug print
    ROS_INFO("%f %f %f %f %f %f %f %f %f, %f %f %f",
             sensorPose(0,0), sensorPose(0,1), sensorPose(0,2),
             sensorPose(1,0), sensorPose(1,1), sensorPose(1,2),
             sensorPose(2,0), sensorPose(2,1), sensorPose(2,2),
             sensorPose(0,3), sensorPose(1,3), sensorPose(2,3));
#endif
  }

  PointCloud pointCloud;
  pcl::RangeImagePlanar rangeImageP;
  {
    // code here is dirty, some bag is in RangeImagePlanar
    PointCloud tpc;
    pcl::fromROSMsg(*pcloud2_ptr, tpc);

    Eigen::Affine3f inv;
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
    inv = sensorPose.inverse();
    pcl::transformPointCloud< Point > (tpc, pointCloud, inv);
#else
    pcl::getInverse(sensorPose, inv);
    pcl::getTransformedPointCloud<PointCloud> (tpc, inv, pointCloud);
#endif

    Eigen::Affine3f dummytrans;
    dummytrans.setIdentity();
    rangeImageP.createFromPointCloudWithFixedSize (pointCloud,
                                                   width/scale_depth, height/scale_depth,
                                                   cx/scale_depth, cy/scale_depth,
                                                   fx/scale_depth, fy/scale_depth,
                                                   dummytrans); //sensorPose);
  }

  // Create color and depth image from point cloud
  cv::Mat color_mat = cv::Mat::zeros(rangeImageP.height, rangeImageP.width, CV_8UC3);
  cv::Mat depth_mat(rangeImageP.height, rangeImageP.width, CV_32FC1);
  depth_mat.setTo(fill_value);
  for (size_t i=0; i<pointCloud.size(); i++) {
    Point pt = pointCloud[i];

    int image_x;
    int image_y;
    rangeImageP.getImagePoint(pt.x, pt.y, pt.z, image_x, image_y);

    if (!rangeImageP.isInImage(image_x, image_y)) {
      continue;
    }

    pcl::PointWithRange pt_with_range = rangeImageP.getPoint(image_x, image_y);
    depth_mat.at<float>(image_y, image_x) = pt_with_range.z;

    if (!std::isnan(pt.x) && !std::isnan(pt.y) && !std::isnan(pt.z)) {
      color_mat.at<cv::Vec3b>(image_y, image_x) = cv::Vec3b(pt.r, pt.g, pt.b);
    }
  }

  if (scale_depth != 1.0) {
    // LINEAR
    cv::resize(color_mat, color_mat, cv::Size(info->width, info->height));
    cv::resize(depth_mat, depth_mat, cv::Size(info->width, info->height));
  }

  if (proc_image) {
    pub_image_.publish(cv_bridge::CvImage(info->header,
                                          sensor_msgs::image_encodings::RGB8,
                                          color_mat).toImageMsg());
  }
  if (proc_depth) {
    pub_depth_.publish(cv_bridge::CvImage(info->header,
                                          sensor_msgs::image_encodings::TYPE_32FC1,
                                          depth_mat).toImageMsg());
  }

  if(proc_cloud || proc_disp) {
    // publish point cloud
    pcl::RangeImagePlanar rangeImagePP;
    rangeImagePP.setDepthImage ((float *)depth_mat.ptr(),
                                width, height,
                                cx, cy, fx, fy);
#if PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 7
    rangeImagePP.header = pcl_conversions::toPCL(info->header);
#else
    rangeImagePP.header = info->header;
#endif
    if (proc_cloud) {
      PointCloud cloud_out;
      cloud_out.header = rangeImagePP.header;
      if (organize_cloud_) {
        cloud_out.width = rangeImagePP.width;
        cloud_out.height = rangeImagePP.height;
      } else {
        cloud_out.width = rangeImagePP.width * rangeImagePP.height;
        cloud_out.height = 1;
      }
      cloud_out.resize(cloud_out.width * cloud_out.height);
      cloud_out.is_dense = true;
      for (int y = 0; y < (int)rangeImagePP.height; y++ ) {
        for (int x = 0; x < (int)rangeImagePP.width; x++ ) {
          pcl::PointWithRange pt_from = rangeImagePP.points[rangeImagePP.width * y + x];
          cv::Vec3b rgb = color_mat.at<cv::Vec3b>(y, x);
          Point pt_to;
          pt_to.x = pt_from.x;
          pt_to.y = pt_from.y;
          pt_to.z = pt_from.z;
          pt_to.r = rgb[0];
          pt_to.g = rgb[1];
          pt_to.b = rgb[2];
          cloud_out.points[rangeImagePP.width * y + x] = pt_to;
          if (std::isnan(pt_to.x) || std::isnan(pt_to.y) || std::isnan(pt_to.z)) {
            cloud_out.is_dense = false;
          }
        }
      }
      pub_cloud_.publish(boost::make_shared<PointCloud>(cloud_out));
    }

    if(proc_disp) {
      stereo_msgs::DisparityImage disp;
#if PCL_MAJOR_VERSION == 1 && PCL_MINOR_VERSION >= 7
      disp.header = pcl_conversions::fromPCL(rangeImagePP.header);
#else
      disp.header = rangeImagePP.header;
#endif
      disp.image.encoding  = sensor_msgs::image_encodings::TYPE_32FC1;
      disp.image.height    = rangeImagePP.height;
      disp.image.width     = rangeImagePP.width;
      disp.image.step      = disp.image.width * sizeof(float);
      disp.f = fx; disp.T = 0.075;
      disp.min_disparity = 0;
      disp.max_disparity = disp.T * disp.f / 0.3;
      disp.delta_d = 0.125;

      disp.image.data.resize (disp.image.height * disp.image.step);
      float *data = reinterpret_cast<float*> (&disp.image.data[0]);

      float normalization_factor = disp.f * disp.T;
      for (int y = 0; y < (int)rangeImagePP.height; y++ ) {
        for (int x = 0; x < (int)rangeImagePP.width; x++ ) {
          pcl::PointWithRange p = rangeImagePP.getPoint(x,y);
          data[y*disp.image.width+x] = normalization_factor / p.z;
        }
      }
      pub_disp_image_.publish(boost::make_shared<stereo_msgs::DisparityImage> (disp));
    }
  }
}  // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::DepthImageCreator, nodelet::Nodelet);
