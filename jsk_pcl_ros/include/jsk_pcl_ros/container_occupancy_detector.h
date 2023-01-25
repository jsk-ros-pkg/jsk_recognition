// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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
/*
 * container_occupancy_detector.h
 * Author: Yoshiki Obinata <obinata@jsk.imi.i.u-tokyo.ac.jp>
 */


#ifndef CONTAINER_OCCUPANCY_DETECTOR_H_
#define CONTAINER_OCCUPANCY_DETECTOR_H_

#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/transforms.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.h>

#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/crop_box.h>

namespace jsk_pcl_ros{

      class ContainerOccupancyDetector: public jsk_topic_tools::DiagnosticNodelet{
            public:
                  typedef message_filters::sync_policies::ExactTime<
                  jsk_recognition_msgs::BoundingBoxArray,
                  sensor_msgs::PointCloud2,
                  jsk_recognition_msgs::ClusterPointIndices
                  > SyncPolicy;
                  typedef message_filters::sync_policies::ApproximateTime<
                  jsk_recognition_msgs::BoundingBoxArray,
                  sensor_msgs::PointCloud2,
                  jsk_recognition_msgs::ClusterPointIndices
                  > ApproximateSyncPolicy;
                  ContainerOccupancyDetector() : DiagnosticNodelet("ContainerOccupancyDetector") {}
                  virtual ~ContainerOccupancyDetector();

            protected:
                  ////////////////////////////////////////////////////////
                  // methods
                  ////////////////////////////////////////////////////////
                  virtual void onInit();
                  virtual void calculate(
                        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& boxes_msg,
                        const sensor_msgs::PointCloud2::ConstPtr& points_msg,
                        const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& point_indices_msg);
                  virtual void updateDiagnostic(
                        diagnostic_updater::DiagnosticStatusWrapper &stat);
                  virtual void subscribe();
                  virtual void unsubscribe();
                  virtual bool pointsTransform(
                        const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& boxes_msg,
                        const sensor_msgs::PointCloud2::ConstPtr& points_msg);

                  ////////////////////////////////////////////////////////
                  // ROS varariables
                  ////////////////////////////////////////////////////////
                  message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_boxes_;
                  message_filters::Subscriber<sensor_msgs::PointCloud2> sub_points_;
                  message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_point_indices_;
                  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
                  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >ap_sync_;
                  ros::Publisher boxes_occupancy_pub_;
                  boost::mutex mutex_;
                  tf2_ros::Buffer tf_buffer_;
                  tf2_ros::TransformListener* tf_listener_;
                  sensor_msgs::PointCloud2::Ptr transformed_points_msg_ =
                        boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2);
                  pcl::PCLPointCloud2Ptr pcl_pc2_ptr_ =
                        boost::shared_ptr<pcl::PCLPointCloud2>(new pcl::PCLPointCloud2);
                  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_xyz_ptr_ =
                        boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>(new pcl::PointCloud<pcl::PointXYZ>);

                  ////////////////////////////////////////////////////////
                  // Diagnostics Variables
                  ////////////////////////////////////////////////////////
                  jsk_recognition_utils::Counter remove_counter_;
                  jsk_recognition_utils::Counter pass_counter_;

                  ////////////////////////////////////////////////////////
                  // Parameters
                  ////////////////////////////////////////////////////////
                  int queue_size_;
                  bool approximate_sync_;

            private:

      };
}

#endif // CONTAINER_OCCUPANCY_DETECTOR_H_
