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

#include "jsk_pcl_ros/organized_edge_detector.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <pcl/features/organized_edge_detection.h>
#include <pcl/features/integral_image_normal.h>
#include "jsk_recognition_utils/pcl_util.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/ColorRGBA.h>
#include <jsk_topic_tools/color_utils.h>

#include <opencv2/opencv.hpp>
#include <jsk_recognition_msgs/ClusterPointIndices.h>

namespace jsk_pcl_ros
{
  void OrganizedEdgeDetector::onInit()
  {
    ConnectionBasedNodelet::onInit();

    ////////////////////////////////////////////////////////
    // setup dynamic reconfigure
    ////////////////////////////////////////////////////////
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&OrganizedEdgeDetector::configCallback, this, _1, _2);
    srv_->setCallback (f);
    ////////////////////////////////////////////////////////
    // indices publishers
    ////////////////////////////////////////////////////////
    pub_nan_boundary_edges_indices_
      = advertise<PCLIndicesMsg>(*pnh_, "output_nan_boundary_edge_indices", 1);
    pub_occluding_edges_indices_
      = advertise<PCLIndicesMsg>(*pnh_, "output_occluding_edge_indices", 1);
    pub_occluded_edges_indices_
      = advertise<PCLIndicesMsg>(*pnh_, "output_occluded_edge_indices", 1);
    pub_curvature_edges_indices_
      = advertise<PCLIndicesMsg>(*pnh_, "output_curvature_edge_indices", 1);
    pub_rgb_edges_indices_
      = advertise<PCLIndicesMsg>(*pnh_, "output_rgb_edge_indices", 1);
    pub_all_edges_indices_
      = advertise<PCLIndicesMsg>(*pnh_, "output_indices", 1);
    pub_straight_edges_indices_
      = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, 
        "output_straight_edges_indices", 1);
    ////////////////////////////////////////////////////////
    // pointcloud publishers
    ////////////////////////////////////////////////////////
    pub_normal_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_normal", 1);
    pub_nan_boundary_edges_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_nan_boundary_edge", 1);
    pub_occluding_edges_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_occluding_edge", 1);
    pub_occluded_edges_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_occluded_edge", 1);
    pub_curvature_edges_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_curvature_edge", 1);
    pub_rgb_edges_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output_rgb_edge", 1);
    pub_all_edges_
      = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    ////////////////////////////////////////////////////////
    // image publishers
    ////////////////////////////////////////////////////////
    image_transport::ImageTransport it(*pnh_);
    pub_edge_image_ = it.advertise("edge_image", 1);
    pub_hough_image_ = it.advertise("hough_image", 1);
    onInitPostProcess();
  }
  
  void OrganizedEdgeDetector::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &OrganizedEdgeDetector::estimate, this);
  }

  void OrganizedEdgeDetector::unsubscribe()
  {
    sub_.shutdown();
  }
  
  void OrganizedEdgeDetector::estimateNormal(
    const pcl::PointCloud<PointT>::Ptr& input,
    pcl::PointCloud<pcl::Normal>::Ptr output,
    const std_msgs::Header& header)
  {
    pcl::IntegralImageNormalEstimation<PointT, pcl::Normal> ne;
    if (estimation_method_ == 0) {
      ne.setNormalEstimationMethod (ne.AVERAGE_3D_GRADIENT);
    }
    else if (estimation_method_ == 1) {
     ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    }
    else if (estimation_method_ == 2) {
      ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    }
    else {
      NODELET_FATAL("unknown estimation method: %d", estimation_method_);
      return;
    }

    if (border_policy_ignore_) {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_IGNORE);
    }
    else {
      ne.setBorderPolicy(pcl::IntegralImageNormalEstimation<PointT, pcl::Normal>::BORDER_POLICY_MIRROR);
    }

    ne.setMaxDepthChangeFactor(max_depth_change_factor_);
    ne.setNormalSmoothingSize(normal_smoothing_size_);
    ne.setDepthDependentSmoothing(depth_dependent_smoothing_);
    ne.setInputCloud(input);
    ne.compute(*output);
    if (publish_normal_) {
      sensor_msgs::PointCloud2 ros_output;
      pcl::toROSMsg(*output, ros_output);
      ros_output.header = header;
      pub_normal_.publish(ros_output);
    }
  }

  void OrganizedEdgeDetector::configCallback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);
    max_depth_change_factor_ = config.max_depth_change_factor;
    normal_smoothing_size_ = config.normal_smoothing_size;
    depth_dependent_smoothing_ = config.depth_dependent_smoothing;
    estimation_method_ = config.estimation_method;
    border_policy_ignore_ = config.border_policy_ignore;
    max_search_neighbors_ = config.max_search_neighbors;
    depth_discontinuation_threshold_ = config.depth_discontinuation_threshold;
    publish_normal_ = config.publish_normal;
    use_nan_boundary_ = config.use_nan_boundary;
    use_occluding_ = config.use_occluding;
    use_occluded_ = config.use_occluded;
    use_curvature_ = config.use_curvature;
    use_rgb_ = config.use_rgb;
    use_straightline_detection_ = config.use_straightline_detection;
    rho_ = config.rho;
    theta_ = config.theta;
    straightline_threshold_ = config.straightline_threshold;
    min_line_length_ = config.min_line_length;
    max_line_gap_ = config.max_line_gap;
    publish_debug_image_ = config.publish_debug_image;
  }
  
  void OrganizedEdgeDetector::estimateEdge(
    const pcl::PointCloud<PointT>::Ptr& input,
    const pcl::PointCloud<pcl::Normal>::Ptr& normal,
    pcl::PointCloud<pcl::Label>::Ptr& output,
    std::vector<pcl::PointIndices>& label_indices)
  {
    pcl::OrganizedEdgeFromRGBNormals<PointT, pcl::Normal, pcl::Label> oed;
    oed.setDepthDisconThreshold (depth_discontinuation_threshold_);
    oed.setMaxSearchNeighbors (max_search_neighbors_);
    int flags = 0;
    if (use_nan_boundary_) {
      flags |= oed.EDGELABEL_NAN_BOUNDARY;
    }
    if (use_occluding_) {
      flags |= oed.EDGELABEL_OCCLUDING;
    }
    if (use_occluded_) {
      flags |= oed.EDGELABEL_OCCLUDED;
    }
    if (use_curvature_) {
      flags |= oed.EDGELABEL_HIGH_CURVATURE;
    }
    if (use_rgb_) {
      flags |= oed.EDGELABEL_RGB_CANNY;
    }
    oed.setEdgeType (flags);
    oed.setInputNormals(normal);
    oed.setInputCloud(input);
    oed.compute(*output, label_indices);
  }

  void OrganizedEdgeDetector::publishIndices(
    ros::Publisher& pub,
    ros::Publisher& pub_indices,
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& indices,
    const std_msgs::Header& header)
  {
    ////////////////////////////////////////////////////////
    // publish indices
    ////////////////////////////////////////////////////////
    PCLIndicesMsg msg;
    msg.header = header;
    msg.indices = indices;
    pub_indices.publish(msg);

    ////////////////////////////////////////////////////////
    // publish cloud
    ////////////////////////////////////////////////////////
    pcl::PointCloud<PointT>::Ptr output(new pcl::PointCloud<PointT>);
    pcl::copyPointCloud(*cloud, indices, *output);
    sensor_msgs::PointCloud2 ros_output;
    pcl::toROSMsg(*output, ros_output);
    ros_output.header = header;
    pub.publish(ros_output);
  }

  void OrganizedEdgeDetector::publishStraightEdges(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std_msgs::Header& header,
    const std::vector<std::vector<int> > indices)
  {
    // output as cluster indices
    jsk_recognition_msgs::ClusterPointIndices ros_msg;
    ros_msg.header = header;
    ros_msg.cluster_indices.resize(indices.size());
    for (size_t i = 0; i < indices.size(); i++) {
      PCLIndicesMsg ros_indices;
      ros_indices.header = header;
      ros_indices.indices = indices[i];
      ros_msg.cluster_indices[i] = ros_indices;
    }
    pub_straight_edges_indices_.publish(ros_msg);
  }
  
  void OrganizedEdgeDetector::estimateStraightEdges(
    const pcl::PointCloud<PointT>::Ptr& cloud,
    const std::vector<int>& indices,
    const std_msgs::Header& header,
    std::vector<std::vector<int> >& output_indices)
  {
    // initialize all the value by 0
    cv::Mat mat = cv::Mat(cloud->height, cloud->width, CV_8UC1) * 0;
    for (size_t i = 0; i < indices.size(); i++) {
      int index = indices[i];
      int index_height = index / cloud->width;
      int index_width = index % cloud->width;
      mat.data[index_height * mat.step + index_width * mat.elemSize()] = 255;
    }
    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(mat, lines, rho_, theta_ * CV_PI/180,
                    straightline_threshold_, min_line_length_, max_line_gap_);
    output_indices.resize(lines.size());
    std::set<int> all_indices_set(indices.begin(), indices.end());
    for (size_t i_line = 0; i_line < lines.size(); i_line++) {
      std::vector<int> pixels;
      cv::LineIterator it(mat,
                          cv::Point(lines[i_line][0], lines[i_line][1]),
                          cv::Point(lines[i_line][2], lines[i_line][3]), 4);
      for(int i_pixel = 0; i_pixel < it.count; i_pixel++, ++it) {
        cv::Point point = it.pos();
        int flatten_index = point.x + point.y * cloud->width;
        // check if flatten_index is included in indices or not
        if (all_indices_set.find(flatten_index) != all_indices_set.end()) {
          pixels.push_back(flatten_index);
        }
      }
      output_indices[i_line] = pixels;
    }
    if (publish_debug_image_) {
      cv::Mat color_dst;
      cv::cvtColor(mat, color_dst, CV_GRAY2BGR );
      for( size_t i = 0; i < lines.size(); i++ )
      {
        std_msgs::ColorRGBA c = jsk_topic_tools::colorCategory20(i);
        cv::line(color_dst,
                 cv::Point(lines[i][0], lines[i][1]),
                 cv::Point(lines[i][2], lines[i][3]),
                 cv::Scalar((uint8_t)(c.b * 255), (uint8_t)(c.g * 255), (uint8_t)(c.r * 255)), 3, 8);
      }
      sensor_msgs::Image::Ptr ros_edge_image
        = cv_bridge::CvImage(header,
                             sensor_msgs::image_encodings::MONO8,
                             mat).toImageMsg();
      pub_edge_image_.publish(ros_edge_image);
      sensor_msgs::Image::Ptr ros_hough_image
        = cv_bridge::CvImage(header,
                             sensor_msgs::image_encodings::BGR8,
                             color_dst).toImageMsg();
      pub_hough_image_.publish(ros_hough_image);
    }
  }
  
  void OrganizedEdgeDetector::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    if (msg->height == 1) {
      NODELET_ERROR("[OrganizedEdgeDetector] organized pointcloud is required");
      return;
    }
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::Label>::Ptr label(new pcl::PointCloud<pcl::Label>);
    std::vector<pcl::PointIndices> label_indices;
    pcl::fromROSMsg(*msg, *cloud);
    
    estimateNormal(cloud, normal, msg->header);
    estimateEdge(cloud, normal, label, label_indices);
    ////////////////////////////////////////////////////////
    // build indices includes all the indices
    ////////////////////////////////////////////////////////
    std::vector<int> tmp1 = jsk_recognition_utils::addIndices(label_indices[0].indices,
                                       label_indices[1].indices);
    std::vector<int> tmp2 = jsk_recognition_utils::addIndices(tmp1,
                                       label_indices[2].indices);
    std::vector<int> tmp3 = jsk_recognition_utils::addIndices(tmp2,
                                       label_indices[3].indices);
    std::vector<int> all = jsk_recognition_utils::addIndices(tmp3,
                                       label_indices[4].indices);
    if (use_straightline_detection_) {
      std::vector<std::vector<int> > straightline_indices;
      estimateStraightEdges(cloud, all, msg->header, straightline_indices);
      // publish the result
      publishStraightEdges(cloud, msg->header, straightline_indices);
    }
    ////////////////////////////////////////////////////////
    // publish result
    ////////////////////////////////////////////////////////
    publishIndices(pub_nan_boundary_edges_, pub_nan_boundary_edges_indices_,
                   cloud,
                   label_indices[0].indices, msg->header);
    publishIndices(pub_occluding_edges_, pub_occluding_edges_indices_,
                   cloud,
                   label_indices[1].indices, msg->header);
    publishIndices(pub_occluded_edges_, pub_occluded_edges_indices_,
                   cloud,
                   label_indices[2].indices, msg->header);
    publishIndices(pub_curvature_edges_, pub_curvature_edges_indices_,
                   cloud,
                   label_indices[3].indices, msg->header);
    publishIndices(pub_rgb_edges_, pub_rgb_edges_indices_,
                   cloud,
                   label_indices[4].indices, msg->header);
    publishIndices(pub_all_edges_, pub_all_edges_indices_,
                   cloud,
                   all, msg->header);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::OrganizedEdgeDetector, nodelet::Nodelet);
