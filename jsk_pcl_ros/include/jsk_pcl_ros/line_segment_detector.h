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


#ifndef JSK_PCL_ROS_LINE_SEGMENT_DETECTOR_H_
#define JSK_PCL_ROS_LINE_SEGMENT_DETECTOR_H_

#include <pcl/segmentation/sac_segmentation.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_pcl_ros/LineSegmentDetectorConfig.h>
#include <dynamic_reconfigure/server.h>
#include <visualization_msgs/Marker.h>
#include "jsk_recognition_utils/geo_util.h"

namespace jsk_pcl_ros
{

  class LineSegment
  {
  public:
    typedef boost::shared_ptr<LineSegment> Ptr;
    LineSegment(const std_msgs::Header& input_header,
                pcl::PointIndices::Ptr indices,
                pcl::ModelCoefficients::Ptr coefficients,
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    LineSegment(pcl::PointIndices::Ptr indices,
                pcl::ModelCoefficients::Ptr coefficients);
    virtual ~LineSegment();
    virtual bool addMarkerLine(
      visualization_msgs::Marker& marker,
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      double minimum_line_length);
    //virtual Segment::Ptr toSegment();
    virtual jsk_recognition_utils::Line::Ptr toSegment();
    pcl::PointIndices::Ptr getIndices() { return indices_; }
    pcl::ModelCoefficients::Ptr getCoefficients() { return coefficients_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getPoints() { return points_; }
    pcl::PointCloud<pcl::PointXYZ>::Ptr getRawPoints() { return raw_points_; }
    std_msgs::Header header;
  protected:
    pcl::PointIndices::Ptr indices_;
    pcl::ModelCoefficients::Ptr coefficients_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_points_;
  private:
    
  };
  
  class LineSegmentDetector: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    LineSegmentDetector(): DiagnosticNodelet("LineSegmentDetector")
    {
    }
    ~LineSegmentDetector()
    {
      sync_.reset();
      srv_.reset();
    }
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices> ApproximateSyncPolicy;
    typedef pcl::PointXYZ PointT;
    typedef jsk_pcl_ros::LineSegmentDetectorConfig Config;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void onInit();
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                         const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_msg);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void segmentLines(
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const pcl::PointIndices::Ptr& indices,
      std::vector<pcl::PointIndices::Ptr>& line_indices,
      std::vector<pcl::ModelCoefficients::Ptr>& line_coefficients);
    virtual void publishResult(
      const std_msgs::Header& header,
      const pcl::PointCloud<PointT>::Ptr& cloud,
      const std::vector<LineSegment::Ptr>& segments);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Publisher pub_line_marker_;
    ros::Publisher pub_indices_;
    ros::Publisher pub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
    boost::recursive_mutex config_mutex_;

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    bool approximate_sync_;
    double outlier_threshold_;
    int max_iterations_;
    int min_indices_;
    double min_length_;
    double line_width_;

    pcl::SACSegmentation<PointT> seg_;

  private:
    
  };
}

#endif
