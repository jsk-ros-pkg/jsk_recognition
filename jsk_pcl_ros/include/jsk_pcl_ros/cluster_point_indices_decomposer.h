// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

#ifndef JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_
#define JSK_PCL_ROS_CLUSTER_POINT_INDICES_DECOMPOSER_H_

#include <ros/ros.h>
#include <ros/names.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"

#include "sensor_msgs/PointCloud2.h"
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <tf/transform_broadcaster.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include "jsk_pcl_ros/pcl_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include "jsk_topic_tools/diagnostic_nodelet.h"

namespace jsk_pcl_ros
{
  class ClusterPointIndicesDecomposer: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    ClusterPointIndicesDecomposer(): DiagnosticNodelet("ClusterPointIndicesDecomposer") { }
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices > SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::ModelCoefficientsArray> SyncAlignPolicy;
    virtual void onInit();
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &point,
                         const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices,
                         const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
                         const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients);
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &point,
                         const jsk_recognition_msgs::ClusterPointIndicesConstPtr &indices);
    virtual void sortIndicesOrder(pcl::PointCloud<pcl::PointXYZ>::Ptr input,
                                  std::vector<pcl::IndicesPtr> indices_array,
                                  std::vector<pcl::IndicesPtr> &output_array);
  protected:
    void addToDebugPointCloud
    (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
     size_t i,
     pcl::PointCloud<pcl::PointXYZRGB>& debug_output);
    
    virtual void computeBoundingBox
    (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud,
     const std_msgs::Header header,
     const Eigen::Vector4f center,
     const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
     const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients,
     jsk_recognition_msgs::BoundingBox& bounding_box);

    
    virtual int findNearestPlane(const Eigen::Vector4f& center,
                                 const jsk_recognition_msgs::PolygonArrayConstPtr& planes,
                                 const jsk_recognition_msgs::ModelCoefficientsArrayConstPtr& coefficients);

    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void allocatePublishers(size_t num);

    virtual void subscribe();
    virtual void unsubscribe();
    
    static uint32_t colorRGBAToUInt32(std_msgs::ColorRGBA c)
    {
        uint8_t r, g, b;
        r = (uint8_t)(c.r * 255);
        g = (uint8_t)(c.g * 255);
        b = (uint8_t)(c.b * 255);
        return ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);
    }

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_target_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<SyncAlignPolicy> >sync_align_;
    std::vector<ros::Publisher> publishers_;
    ros::Publisher pc_pub_, box_pub_;
    tf::TransformBroadcaster br_;
    std::string tf_prefix_;
    
    bool force_to_flip_z_axis_;
    bool publish_clouds_;
    bool publish_tf_;
    bool align_boxes_;
    bool use_pca_;

    Counter cluster_counter_;
    
  };

}

#endif
