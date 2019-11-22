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

////////////////////////////////////////////////////////
// the implementation is based on
// Line segment-based fast 3D plane extraction using nodding 2D laser rangender
//   Su-Yong An, Lae-Kyoung Lee and Se-Young Oh,
//   ,Robotica / FirstView Article / October 2014, pp 1 - 24
////////////////////////////////////////////////////////
#ifndef JSK_PCL_ROS_LINE_SEGMENT_COLLECTOR_H_
#define JSK_PCL_ROS_LINE_SEGMENT_COLLECTOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <sensor_msgs/JointState.h>
#include "jsk_pcl_ros/line_segment_detector.h"
#include <jsk_topic_tools/time_accumulator.h>
#include <jsk_pcl_ros/LineSegmentCollectorConfig.h>
#include <dynamic_reconfigure/server.h>
#include "jsk_recognition_utils/geo_util.h"
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/TimeRange.h>

namespace jsk_pcl_ros
{

  class LineSegmentCluster
  {
  public:
    typedef boost::shared_ptr<LineSegmentCluster> Ptr;
    LineSegmentCluster();
    virtual ~LineSegmentCluster() { };
    
    ////////////////////////////////////////////////////////
    // update delta_ by EWMA(exponentially weighted moving agerage)
    ////////////////////////////////////////////////////////
    virtual void addLineSegmentEWMA(LineSegment::Ptr segment, const double tau);
    virtual Eigen::Vector3f getDelta() { return delta_; }
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getPoints();
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr getRawPoints();
    virtual void removeBefore(const ros::Time& stamp);
    virtual bool isEmpty();
  protected:
    Eigen::Vector3f delta_;
    std::vector<LineSegment::Ptr> segments_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points_;
  private:
    
  };
  
  template <class T>
  class TimeStampedVector: public std::vector<T>
  {
  public:
    typedef typename std::vector<T>::iterator iterator;
    void removeBefore(const ros::Time& stamp)
    {
      for (iterator it = std::vector<T>::begin();
           it != std::vector<T>::end();) {
        if (((*it)->header.stamp - stamp) < ros::Duration(0.0)) {
          it = this->erase(it);
        }
        else {
          ++it;
        }
      }
    }
  protected:
  private:
  };
  
  class LineSegmentCollector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    LineSegmentCollector(): DiagnosticNodelet("LineSegmentCollector") { }
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::ModelCoefficientsArray> SyncPolicy;
    enum RotateType {
      ROTATION_SPINDLE, ROTATION_TILT, ROTATION_TILT_TWO_WAY
    };
    typedef jsk_pcl_ros::LineSegmentCollectorConfig Config;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void collectFromBuffers(const std_msgs::Header& header,
                                    std::vector<LineSegment::Ptr> new_segments);
    virtual void collect(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients_msg);
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void triggerCallback(
      const jsk_recognition_msgs::TimeRange::ConstPtr& trigger);
    virtual void cleanupBuffers(
      const ros::Time& stamp);
    virtual void publishBeforePlaneSegmentation(
      const std_msgs::Header& header,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      const std::vector<pcl::PointIndices::Ptr>& connected_indices);
    virtual LineSegmentCluster::Ptr lookupNearestSegment(
      LineSegment::Ptr segment);
    virtual void publishResult(
      const std_msgs::Header& header,
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      std::vector<pcl::ModelCoefficients::Ptr> all_coefficients,
      std::vector<pcl::PointIndices::Ptr> all_indices);
    virtual void configCallback(Config &config, uint32_t level);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    ros::Publisher pub_point_cloud_;
    ros::Publisher pub_inliers_;
    ros::Publisher pub_coefficients_;
    ros::Publisher pub_polygons_;
    ros::Publisher debug_pub_inliers_before_plane_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    jsk_topic_tools::TimeAccumulator connect_ac_;
    ros::Subscriber sub_trigger_;
    
    ////////////////////////////////////////////////////////
    // parameters to collect pointclouds
    ////////////////////////////////////////////////////////
    std::string fixed_frame_id_;
    RotateType rotate_type_;
    TimeStampedVector<sensor_msgs::PointCloud2::ConstPtr> pointclouds_buffer_;
    TimeStampedVector<jsk_recognition_msgs::ClusterPointIndices::ConstPtr> indices_buffer_;
    TimeStampedVector<jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr> coefficients_buffer_;
    TimeStampedVector<LineSegment::Ptr> segments_buffer_;
    std::vector<LineSegmentCluster::Ptr> segment_clusters_;
    double segment_connect_normal_threshold_;
    double ewma_tau_;
    jsk_recognition_msgs::TimeRange::ConstPtr time_range_;
    ////////////////////////////////////////////////////////
    // plane estimation
    ////////////////////////////////////////////////////////
    double outlier_threshold_;
    
  private:
    
  };
}

#endif
