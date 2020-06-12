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


#ifndef JSK_PCL_ROS_POINTCLOUD_MOVEIT_FILTER_H_
#define JSK_PCL_ROS_POINTCLOUD_MOVEIT_FILTER_H_

#include <ros/ros.h>
#include <tf/tf.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit/version.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/point_containment_filter/shape_mask.h>
#include <moveit/occupancy_map_monitor/occupancy_map_monitor.h>
#include <XmlRpcException.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>

#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic or MoveIt 1.0
#include <tf2_ros/message_filter.h>
#else
#include <tf/message_filter.h>
#endif

namespace jsk_pcl_ros
{
  typedef occupancy_map_monitor::ShapeHandle ShapeHandle;
  typedef occupancy_map_monitor::ShapeTransformCache ShapeTransformCache;
  class PointCloudMoveitFilter:
    public occupancy_map_monitor::OccupancyMapUpdater
  {
  public:
    PointCloudMoveitFilter();
    virtual ~PointCloudMoveitFilter();

    virtual bool setParams(XmlRpc::XmlRpcValue &params);
    virtual bool initialize();
    virtual void start();
    virtual void stop();
    virtual ShapeHandle excludeShape(const shapes::ShapeConstPtr &shape);
    virtual void forgetShape(ShapeHandle handle);
    
  protected:
    virtual void stopHelper();
    virtual bool getShapeTransform(ShapeHandle h,
#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic or MoveIt 1.0
                                   Eigen::Isometry3d &transform) const;
#else
                                   Eigen::Affine3d &transform) const;
#endif
    template <typename PointT>
    void cloudMsgCallback(
      const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
      {
        if (monitor_->getMapFrame().empty())
          monitor_->setMapFrame(cloud_msg->header.frame_id);
        tf::StampedTransform map_H_sensor;
        if (monitor_->getMapFrame() == cloud_msg->header.frame_id) {
          map_H_sensor.setIdentity();
        }
        else {
          if (tf_) {
            try
            {
#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic or MoveIt 1.0
              tf::transformStampedMsgToTF(tf_->lookupTransform(monitor_->getMapFrame(),
                                          cloud_msg->header.frame_id,
                                          cloud_msg->header.stamp),
                                          map_H_sensor);
#else
              tf_->lookupTransform(monitor_->getMapFrame(),
                                   cloud_msg->header.frame_id,
                                   cloud_msg->header.stamp,
                                   map_H_sensor);
#endif
            }
            catch (tf::TransformException& ex)
            {
              ROS_ERROR_STREAM("Transform error of sensor data: "
                               << ex.what() << "; quitting callback");
              return;
            }
          }
          else {
            return;
          }
        }
        /* convert cloud message to pcl cloud object */
        //typename pcl::PointCloud<PointT>::Ptr cloud(pcl::PointCloud<PointT>);
        typename pcl::PointCloud<PointT>::Ptr cloud;
        cloud.reset(new pcl::PointCloud<PointT>());
        pcl::PointCloud<pcl::PointXYZ> xyz_cloud;
        pcl::fromROSMsg(*cloud_msg, *cloud);
        pcl::fromROSMsg(*cloud_msg, xyz_cloud);
        const tf::Vector3 &sensor_origin_tf = map_H_sensor.getOrigin();
        octomap::point3d sensor_origin(
          sensor_origin_tf.getX(),
          sensor_origin_tf.getY(),
          sensor_origin_tf.getZ());
        Eigen::Vector3d sensor_origin_eigen(
          sensor_origin_tf.getX(),
          sensor_origin_tf.getY(),
          sensor_origin_tf.getZ());
        if (!updateTransformCache(cloud_msg->header.frame_id,
                                  cloud_msg->header.stamp))
        {
          ROS_ERROR_THROTTLE(
            1, "Transform cache was not updated. Self-filtering may fail.");
          return;
        }
        /* mask out points on the robot */
#if (MOVEIT_VERSION_MAJOR == 0 and MOVEIT_VERSION_MINOR < 6)
        shape_mask_->maskContainment(xyz_cloud, sensor_origin_eigen, 0.0,
                                     max_range_, mask_);
#else   // from moveit 0.6 (indigo), PCL dependency is removed
        shape_mask_->maskContainment(*cloud_msg, sensor_origin_eigen, 0.0,
                                     max_range_, mask_);
#endif
        typename pcl::PointCloud<PointT>::Ptr
          filtered_cloud (new pcl::PointCloud<PointT>);
        pcl::PointIndices::Ptr indices (new pcl::PointIndices);
        // convert mask into indices
        for (size_t i = 0; i < mask_.size(); i++) {
          if (mask_[i] == point_containment_filter::ShapeMask::OUTSIDE) {
            indices->indices.push_back(i);
          }
        }
        pcl::ExtractIndices<PointT> ex;
        ex.setInputCloud(cloud);
        ex.setIndices(indices);
        ex.setKeepOrganized(keep_organized_);
        ex.filter(*filtered_cloud);
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*filtered_cloud, ros_cloud);
        ros_cloud.header = cloud_msg->header;
        filtered_cloud_publisher_.publish(ros_cloud);
      }

    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::NodeHandle root_nh_;
    ros::NodeHandle private_nh_;
#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic and MoveIt 1.0
    std::shared_ptr<tf2_ros::Buffer> tf_;
#else
    boost::shared_ptr<tf::Transformer> tf_;
#endif
    std::string point_cloud_topic_;
    double scale_;
    double padding_;
    double max_range_;
    unsigned int point_subsample_;
    std::string filtered_cloud_topic_;
    ros::Publisher filtered_cloud_publisher_;

    message_filters::Subscriber<sensor_msgs::PointCloud2> *point_cloud_subscriber_;
#if (ROS_VERSION_MINIMUM(1,14,0) || MOVEIT_VERSION_MAJOR >= 1) // melodic or MoveIt 1.0
    tf2_ros::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;
#else
    tf::MessageFilter<sensor_msgs::PointCloud2> *point_cloud_filter_;
#endif

    boost::scoped_ptr<point_containment_filter::ShapeMask> shape_mask_;
    std::vector<int> mask_;

    bool use_color_;
    bool keep_organized_;
    
  private:
    
  };
}

#endif
