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
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_pcl_ros/line_segment_detector.h"
#include <visualization_msgs/Marker.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <jsk_topic_tools/color_utils.h>

namespace jsk_pcl_ros
{

  LineSegment::LineSegment(
    pcl::PointIndices::Ptr indices,
    pcl::ModelCoefficients::Ptr coefficients):
    indices_(indices), coefficients_(coefficients)
  {
  }
  
  LineSegment::LineSegment(
    const std_msgs::Header& input_header,
    pcl::PointIndices::Ptr indices,
    pcl::ModelCoefficients::Ptr coefficients,
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud):
    header(input_header),
    indices_(indices), coefficients_(coefficients),
    points_(new pcl::PointCloud<pcl::PointXYZ>),
    raw_points_(new pcl::PointCloud<pcl::PointXYZ>)
  {
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setInputCloud(cloud);
    proj.setIndices(indices);
    proj.setModelType(pcl::SACMODEL_LINE);
    proj.setModelCoefficients(coefficients);
    proj.filter(*points_);
    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(cloud);
    ex.setIndices(indices);
    ex.filter(*raw_points_);
  }

  LineSegment::~LineSegment()
  {
  }

  jsk_recognition_utils::Line::Ptr LineSegment::toSegment()
  {
    // we suppose the first and the last point should be the end points
    // return Segment::Ptr(
    //   new Segment(
    //     points_->points[0].getVector3fMap(),
    //     points_->points[points_->points.size() - 1].getVector3fMap()));
    Eigen::Vector3f direction;
    direction[0] = coefficients_->values[3];
    direction[1] = coefficients_->values[4];
    direction[2] = coefficients_->values[5];
    return jsk_recognition_utils::Line::Ptr(new jsk_recognition_utils::Line(direction,
                                                                            points_->points[0].getVector3fMap()));
    // return Segment::Ptr(
    //   new Segment(
    //     points_->points[0].getVector3fMap(),
    //     points_->points[points_->points.size() - 1].getVector3fMap()));
  }
  

  bool LineSegment::addMarkerLine(
    visualization_msgs::Marker& marker,
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    const double minimum_line_length)
  {
    // lookup minimum and max index
    int min_index = INT_MAX;
    int max_index = - INT_MAX;
    for (size_t i = 0; i < indices_->indices.size(); i++) {
      int index = indices_->indices[i];
      if (min_index > index) {
        min_index = index;
      }
      if (max_index < index) {
        max_index = index;
      }
    }
    geometry_msgs::Point a, b;
    jsk_recognition_utils::pointFromXYZToXYZ<pcl::PointXYZ, geometry_msgs::Point>(
      cloud->points[min_index], a);
    jsk_recognition_utils::pointFromXYZToXYZ<pcl::PointXYZ, geometry_msgs::Point>(
      cloud->points[max_index], b);
    if (std::sqrt((a.x - b.x) * (a.x - b.x) +
                  (a.y - b.y) * (a.y - b.y) +
                  (a.z - b.z) * (a.z - b.z)) < minimum_line_length) {
      return false;
    }
    marker.points.push_back(a);
    marker.points.push_back(b);
    return true;
  }
  
  void LineSegmentDetector::onInit()
  {
    DiagnosticNodelet::onInit();

    pnh_->param("approximate_sync", approximate_sync_, false);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (config_mutex_, *pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&LineSegmentDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);

    ////////////////////////////////////////////////////////
    // setup publishers
    ////////////////////////////////////////////////////////
    pub_line_marker_ = advertise<visualization_msgs::Marker>(
      *pnh_, "debug/line_marker", 1);
    pub_indices_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(
      *pnh_, "output/inliers", 1);
    pub_coefficients_ = advertise<jsk_recognition_msgs::ModelCoefficientsArray>(
      *pnh_, "output/coefficients", 1);

    onInitPostProcess();
  }

  void LineSegmentDetector::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    outlier_threshold_ = config.outlier_threshold;
    max_iterations_ = config.max_iterations;
    min_indices_ = config.min_indices;
    min_length_ = config.min_length;
    line_width_ = config.line_width;


    // update segmentation parameters
    seg_.setOptimizeCoefficients (true);
    seg_.setModelType(pcl::SACMODEL_LINE);
    int segmentation_method;
    {
      boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
      segmentation_method = config.method_type;
    }
    seg_.setMethodType(segmentation_method);
    seg_.setDistanceThreshold (outlier_threshold_);
    seg_.setMaxIterations (max_iterations_);
  }
  
  void LineSegmentDetector::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "input_indices", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
      async_->connectInput(sub_input_, sub_indices_);
      async_->registerCallback(boost::bind(&LineSegmentDetector::segment,
                                           this, _1, _2));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
      sync_->connectInput(sub_input_, sub_indices_);
      sync_->registerCallback(boost::bind(&LineSegmentDetector::segment,
                                          this, _1, _2));
    }
  }

  void LineSegmentDetector::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_indices_.unsubscribe();
  }

  void LineSegmentDetector::publishResult(
    const std_msgs::Header& header,
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<LineSegment::Ptr>& segments)
  {
    std::vector<pcl::PointIndices::Ptr> indices;
    std::vector<pcl::ModelCoefficients::Ptr> coefficients;
    visualization_msgs::Marker marker;
    marker.header = header;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = line_width_;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    for (size_t i = 0; i < segments.size(); i++) {
      if (segments[i]->addMarkerLine(marker, cloud, min_length_) == false)
        continue;
      indices.push_back(segments[i]->getIndices());
      coefficients.push_back(segments[i]->getCoefficients());
      std_msgs::ColorRGBA color = jsk_topic_tools::colorCategory20(i);
      color.a = 1.0;
      marker.colors.push_back(color);
    }

    jsk_recognition_msgs::ModelCoefficientsArray ros_coefficients;
    jsk_recognition_msgs::ClusterPointIndices ros_indices;
    ros_coefficients.header = header;
    ros_indices.header = header;
    ros_coefficients.coefficients
      = pcl_conversions::convertToROSModelCoefficients(
        coefficients, header);
    ros_indices.cluster_indices
      = pcl_conversions::convertToROSPointIndices(
        indices, header);
    pub_indices_.publish(ros_indices);
    pub_coefficients_.publish(ros_coefficients);
    pub_line_marker_.publish(marker);
  }
  
  void LineSegmentDetector::segmentLines(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const pcl::PointIndices::Ptr& indices,
    std::vector<pcl::PointIndices::Ptr>& line_indices,
    std::vector<pcl::ModelCoefficients::Ptr>& line_coefficients)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::PointIndices::Ptr rest_indices (new pcl::PointIndices);
    rest_indices->indices = indices->indices;
    // use RANSAC to segment lines
    seg_.setInputCloud(cloud);
    while (true) {
      if (rest_indices->indices.size() > min_indices_) {
        pcl::PointIndices::Ptr
          result_indices (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr
          result_coefficients (new pcl::ModelCoefficients);
        seg_.setIndices(rest_indices);
        seg_.segment(*result_indices, *result_coefficients);
        if (result_indices->indices.size() > min_indices_) {
          line_indices.push_back(result_indices);
          line_coefficients.push_back(result_coefficients);
          rest_indices = jsk_recognition_utils::subIndices(*rest_indices, *result_indices);
        }
        else {
          break;
        }
      }
      else {
        break;
      }
    }
    
  }
  
  void LineSegmentDetector::segment(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_msg)
  {
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*cloud_msg, *cloud);
    std::vector<LineSegment::Ptr> segments;
    std::vector<pcl::PointIndices::Ptr> input_indices
      = pcl_conversions::convertToPCLPointIndices(cluster_msg->cluster_indices);
    // for each cluster
    for (size_t i = 0; i < cluster_msg->cluster_indices.size(); i++) {
      std::vector<pcl::PointIndices::Ptr> line_indices;
      std::vector<pcl::ModelCoefficients::Ptr> line_coefficients;
      segmentLines(cloud, input_indices[i],
                   line_indices, line_coefficients);
      if (line_indices.size() > 0) {
        // update lines
        for (size_t j = 0; j < line_indices.size(); j++) {
          segments.push_back(
            LineSegment::Ptr(new LineSegment(line_indices[j],
                                             line_coefficients[j])));
        }
      }
    }
    // publish result
    publishResult(cloud_msg->header, cloud, segments);
  }
  
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::LineSegmentDetector,
                        nodelet::Nodelet);
