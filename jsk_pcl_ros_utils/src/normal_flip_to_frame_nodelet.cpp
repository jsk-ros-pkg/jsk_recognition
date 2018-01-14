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

#define BOOST_PARAMETER_MAX_ARITY 7

#include "jsk_pcl_ros_utils/normal_flip_to_frame.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"


namespace jsk_pcl_ros_utils
{
  void NormalFlipToFrame::onInit()
  {
    DiagnosticNodelet::onInit();
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    if (!pnh_->getParam("frame_id", frame_id_)) {
      NODELET_FATAL("[%s] no ~frame_id is specified", __PRETTY_FUNCTION__);
    }
    pnh_->param("strict_tf", strict_tf_, false);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void NormalFlipToFrame::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &NormalFlipToFrame::flip, this);
  }

  void NormalFlipToFrame::unsubscribe()
  {
    sub_.shutdown();
  }

  void NormalFlipToFrame::flip(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    vital_checker_->poke();
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud
      (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    // check tf is available
    try {
      ros::Time stamp;
      if (strict_tf_) {
        stamp = cloud_msg->header.stamp;
      }
      else {
        stamp = ros::Time(0);
      }
      tf::StampedTransform sensor_transform_tf
        = jsk_recognition_utils::lookupTransformWithDuration(
          tf_listener_, frame_id_,
          cloud_msg->header.frame_id,
          stamp,
          ros::Duration(1.0));
      Eigen::Affine3f sensor_transform;
      tf::transformTFToEigen(sensor_transform_tf, sensor_transform);
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr flipped_cloud
        (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      flipped_cloud->points.resize(cloud->points.size());
      Eigen::Vector3f s(sensor_transform.translation());
      for (size_t i = 0; i < cloud->points.size(); i++) {
        // Check point is not nan
        if (pcl_isfinite(cloud->points[i].x) &&
            pcl_isfinite(cloud->points[i].y) &&
            pcl_isfinite(cloud->points[i].z) &&
            pcl_isfinite(cloud->points[i].normal_x) &&
            pcl_isfinite(cloud->points[i].normal_y) &&
            pcl_isfinite(cloud->points[i].normal_z)) {
          Eigen::Vector3f p = cloud->points[i].getVector3fMap();
          Eigen::Vector3f n(cloud->points[i].normal_x,
                            cloud->points[i].normal_y,
                            cloud->points[i].normal_z);
          if ((s - p).dot(n) < 0) {
            pcl::PointXYZRGBNormal new_p;
            jsk_recognition_utils::pointFromVectorToXYZ<Eigen::Vector3f, pcl::PointXYZRGBNormal>(
              p, new_p);
            new_p.rgb = cloud->points[i].rgb;
            new_p.normal_x = - n[0];
            new_p.normal_y = - n[1];
            new_p.normal_z = - n[2];
            flipped_cloud->points[i] = new_p;
          }
          else {
            flipped_cloud->points[i] = cloud->points[i];
          }
        }
        else {
          flipped_cloud->points[i] = cloud->points[i];
        }
        
      }
      sensor_msgs::PointCloud2 ros_cloud;
      pcl::toROSMsg(*flipped_cloud, ros_cloud);
      ros_cloud.header = cloud_msg->header;
      pub_.publish(ros_cloud);
    }
    catch (tf2::ConnectivityException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::InvalidArgumentException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
    catch (tf2::TransformException &e)
    {
      NODELET_ERROR("[%s] Transform error: %s", __PRETTY_FUNCTION__, e.what());
    }
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::NormalFlipToFrame, nodelet::Nodelet);
