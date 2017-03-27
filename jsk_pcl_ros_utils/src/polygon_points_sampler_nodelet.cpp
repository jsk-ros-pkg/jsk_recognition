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
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_pcl_ros_utils/polygon_points_sampler.h"

namespace jsk_pcl_ros_utils
{
  void PolygonPointsSampler::onInit()
  {
    DiagnosticNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&PolygonPointsSampler::configCallback, this, _1, _2);
    srv_->setCallback (f);

    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    pub_xyz_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_xyz", 1);

    onInitPostProcess();
  }

  void PolygonPointsSampler::subscribe()
  {
    sub_polygons_.subscribe(*pnh_, "input/polygons", 1);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_polygons_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&PolygonPointsSampler::sample, this, _1, _2));
  }

  void PolygonPointsSampler::unsubscribe()
  {
    sub_polygons_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }

  bool PolygonPointsSampler::isValidMessage(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    if (polygon_msg->polygons.size() == 0) {
      NODELET_DEBUG("empty polygons");
      return false;
    }
    if (coefficients_msg->coefficients.size() != polygon_msg->polygons.size()) {
      NODELET_ERROR("The size of coefficients and polygons are not same");
      return false;
    }

    std::string frame_id = polygon_msg->header.frame_id;
    for (size_t i = 0; i < polygon_msg->polygons.size(); i++) {
      if (frame_id != polygon_msg->polygons[i].header.frame_id) {
        NODELET_ERROR("Frame id of polygon is not same: %s, %s",
                      frame_id.c_str(),
                      polygon_msg->polygons[i].header.frame_id.c_str());
        return false;
      }
    }
    for (size_t i = 0; i < coefficients_msg->coefficients.size(); i++) {
      if (frame_id != coefficients_msg->coefficients[i].header.frame_id) {
        NODELET_ERROR("Frame id of coefficient is not same: %s, %s",
                      frame_id.c_str(),
                      coefficients_msg->coefficients[i].header.frame_id.c_str());
        return false;
      }
    }
    return true;
  }
  
  void PolygonPointsSampler::sample(
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
    const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    vital_checker_->poke();
    // check frame_ids
    if (!isValidMessage(polygon_msg, coefficients_msg)) {
      return;
    }
    // Sample points... 
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < polygon_msg->polygons.size(); i++) {
      jsk_recognition_utils::Polygon polygon
        = jsk_recognition_utils::Polygon::fromROSMsg(polygon_msg->polygons[i].polygon);
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr one_cloud
        = polygon.samplePoints<pcl::PointXYZRGBNormal>(grid_size_);
      pcl::PointCloud<pcl::PointXYZ> one_xyz_cloud;
      one_xyz_cloud.points.resize(one_cloud->points.size());
      for (size_t i = 0; i < one_cloud->points.size(); i++) {
        pcl::PointXYZ p;
        p.getVector3fMap() = one_cloud->points[i].getVector3fMap();
        one_xyz_cloud.points[i] = p;
      }
      *xyz_cloud = *xyz_cloud + one_xyz_cloud;
      *cloud = *cloud + *one_cloud;
    }
    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*cloud, ros_cloud);
    ros_cloud.header = polygon_msg->header;
    pub_.publish(ros_cloud);
    sensor_msgs::PointCloud2 ros_xyz_cloud;
    pcl::toROSMsg(*xyz_cloud, ros_xyz_cloud);
    ros_xyz_cloud.header = polygon_msg->header;
    pub_xyz_.publish(ros_xyz_cloud);
  }

  
  void PolygonPointsSampler::configCallback(Config& config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    grid_size_ = config.grid_size;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PolygonPointsSampler, nodelet::Nodelet);
