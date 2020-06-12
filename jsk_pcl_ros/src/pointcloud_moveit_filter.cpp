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
#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros/pointcloud_moveit_filter.h"

namespace jsk_pcl_ros
{
  PointCloudMoveitFilter::PointCloudMoveitFilter():
    OccupancyMapUpdater("PointCloudMoveitFilter"),
    private_nh_("~"),
    scale_(1.0),
    padding_(0.0),
    max_range_(std::numeric_limits<double>::infinity()),
    point_subsample_(1),
    point_cloud_subscriber_(NULL),
    point_cloud_filter_(NULL),
    use_color_(false),
    keep_organized_(false)
  {
  }

  PointCloudMoveitFilter::~PointCloudMoveitFilter()
  {
  }

  bool PointCloudMoveitFilter::setParams(XmlRpc::XmlRpcValue &params)
  {
    try
    {
      if (!params.hasMember("point_cloud_topic"))
        return false;
      point_cloud_topic_ = static_cast<const std::string&>(params["point_cloud_topic"]);
      
      readXmlParam(params, "max_range", &max_range_);
      readXmlParam(params, "padding_offset", &padding_);
      readXmlParam(params, "padding_scale", &scale_);
      readXmlParam(params, "point_subsample", &point_subsample_);
      if (!params.hasMember("filtered_cloud_topic")) {
        ROS_ERROR("filtered_cloud_topic is required");
        return false;
      }
      else {
        filtered_cloud_topic_ = static_cast<const std::string&>(params["filtered_cloud_topic"]);
      }
      if (params.hasMember("filtered_cloud_use_color")) {
        use_color_ = (bool)params["filtered_cloud_use_color"];
      }
      if (params.hasMember("filtered_cloud_keep_organized")) {
        keep_organized_ = (bool)params["filtered_cloud_keep_organized"];
      }
    }
    catch (XmlRpc::XmlRpcException& ex)
    {
      ROS_ERROR("XmlRpc Exception: %s", ex.getMessage().c_str());
      return false;
    }
    
    return true;
  }


  
  bool PointCloudMoveitFilter::initialize()
  {
    tf_ = monitor_->getTFClient();
    shape_mask_.reset(new point_containment_filter::ShapeMask());
    shape_mask_->setTransformCallback(
      boost::bind(&PointCloudMoveitFilter::getShapeTransform, this, _1, _2));
    filtered_cloud_publisher_ = private_nh_.advertise<sensor_msgs::PointCloud2>(
      filtered_cloud_topic_, 10, false);
    return true;
  }

#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic or MoveIt 1.0
  bool PointCloudMoveitFilter::getShapeTransform(ShapeHandle h, Eigen::Isometry3d &transform) const
#else
  bool PointCloudMoveitFilter::getShapeTransform(ShapeHandle h, Eigen::Affine3d &transform) const
#endif
  {
    ShapeTransformCache::const_iterator it = transform_cache_.find(h);
    if (it == transform_cache_.end())
    {
      ROS_ERROR("Internal error. Shape filter handle %u not found", h);
      return false;
    }
    transform = it->second;
    return true;
  }

  ShapeHandle PointCloudMoveitFilter::excludeShape(const shapes::ShapeConstPtr &shape)
  {
    ShapeHandle h = 0;
    if (shape_mask_)
      h = shape_mask_->addShape(shape, scale_, padding_);
    else
      ROS_ERROR("Shape filter not yet initialized!");
    return h;
  }

  
  void PointCloudMoveitFilter::forgetShape(ShapeHandle handle)
  {
    if (shape_mask_)
      shape_mask_->removeShape(handle);
  }

  
  void PointCloudMoveitFilter::start()
  {
    if (point_cloud_subscriber_)
      return;
    /* subscribe to point cloud topic using tf filter*/
    point_cloud_subscriber_
      = new message_filters::Subscriber<sensor_msgs::PointCloud2>(
        root_nh_, point_cloud_topic_, 5);
    if (tf_ && !monitor_->getMapFrame().empty())
    {
      point_cloud_filter_
#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic or MoveIt 1.0
        = new tf2_ros::MessageFilter<sensor_msgs::PointCloud2>(
          *point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 5, nullptr);
#else
        = new tf::MessageFilter<sensor_msgs::PointCloud2>(
          *point_cloud_subscriber_, *tf_, monitor_->getMapFrame(), 5);
#endif
      if (use_color_) {
        point_cloud_filter_->registerCallback(
          boost::bind(
            &PointCloudMoveitFilter::cloudMsgCallback<pcl::PointXYZRGB>,
            this, _1));
      }
      else {
        point_cloud_filter_->registerCallback(
          boost::bind(
            &PointCloudMoveitFilter::cloudMsgCallback<pcl::PointXYZ>,
            this, _1));
      }
      ROS_INFO("Listening to '%s' using message filter with target frame '%s'",
               point_cloud_topic_.c_str(),
               point_cloud_filter_->getTargetFramesString().c_str());
    }
    else
    {
      if (use_color_) {
        point_cloud_subscriber_->registerCallback(
          boost::bind(
            &PointCloudMoveitFilter::cloudMsgCallback<pcl::PointXYZRGB>,
            this, _1));
      }
      else {
        point_cloud_subscriber_->registerCallback(
          boost::bind(&PointCloudMoveitFilter::cloudMsgCallback<pcl::PointXYZ>,
                      this, _1));
      }
      ROS_INFO("Listening to '%s'", point_cloud_topic_.c_str());
    }
  }

  void PointCloudMoveitFilter::stopHelper()
  {
    delete point_cloud_filter_;
    delete point_cloud_subscriber_;
  }
  
  void PointCloudMoveitFilter::stop()
  {
    stopHelper();
    point_cloud_filter_ = NULL;
    point_cloud_subscriber_ = NULL;
  }
  
}

#include <class_loader/class_loader.h>
CLASS_LOADER_REGISTER_CLASS(jsk_pcl_ros::PointCloudMoveitFilter, occupancy_map_monitor::OccupancyMapUpdater)
