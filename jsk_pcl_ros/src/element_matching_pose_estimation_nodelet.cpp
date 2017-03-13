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
#include <ctime>
#include <algorithm>
#include <boost/thread/thread.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <tf/transform_broadcaster.h>
#include <jsk_recognition_msgs/EdgeArray.h>
#include <jsk_topic_tools/color_utils.h>
#include <jsk_recognition_msgs/ElementCorrespondence.h>
#include <jsk_recognition_msgs/ResetPose.h>

#include "jsk_pcl_ros/element_matching_pose_estimation.h"


namespace jsk_pcl_ros
{
  void ElementMatchingPoseEstimation::onInit()
  {
    ConnectionBasedNodelet::onInit();

    estimated_pose_.pose.orientation.w = 1.0;
    estimated_pose_.header.frame_id = std::string("");

    pnh_->param("frame_id", frame_id_, std::string(""));
    pnh_->param("max_queue_size", max_queue_size_, 10);
    pnh_->param("approximate_sync", approximate_sync_, false);
    pnh_->param("debug_viewer", debug_viewer_, false);
    pnh_->param("publish_tf", publish_tf_, false);
    double tf_rate;
    pnh_->param("tf_rate", tf_rate, 20.0);
    pnh_->param("output_frame_id", output_frame_id_, std::string("track_result"));
    pnh_->param("publish_model_element", publish_model_element_, true);
    pnh_->param("max_edge_indices", max_edge_indices_, 100);
    pnh_->param("max_polygon_indices", max_polygon_indices_, 100);
    {
      double tolerance;
      if (pnh_->hasParam("tolerance")) {
        pnh_->param("tolerance", tolerance, 0.0);
        trans_est_.tolerance_line = tolerance;
        trans_est_.tolerance_edge = tolerance;
        trans_est_.tolerance_plane = tolerance;
        trans_est_.tolerance_polygon = tolerance;
        trans_est_.tolerance_point = tolerance;
      }
      if (pnh_->hasParam("tolerance_line")) {
        pnh_->param("tolerance_line", tolerance, 0.0);
        trans_est_.tolerance_line = tolerance;
      }
      if (pnh_->hasParam("tolerance_edge")) {
        pnh_->param("tolerance_edge", tolerance, 0.0);
        trans_est_.tolerance_edge = tolerance;
      }
      if (pnh_->hasParam("tolerance_plane")) {
        pnh_->param("tolerance_plane", tolerance, 0.0);
        trans_est_.tolerance_plane = tolerance;
      }
      if (pnh_->hasParam("tolerance_polygon")) {
        pnh_->param("tolerance_polygon", tolerance, 0.0);
        trans_est_.tolerance_polygon = tolerance;
      }
      if (pnh_->hasParam("tolerance_point")) {
        pnh_->param("tolerance_point", tolerance, 0.0);
        trans_est_.tolerance_point = tolerance;
      }
    }
    if (pnh_->hasParam("fixed_weight")) {
      double fixed_weight;
      pnh_->param("fixed_weight", fixed_weight, 0.0);
      trans_est_.fixed_weight = fixed_weight;
    }

    tf_listener_ = jsk_recognition_utils::TfListenerSingleton::getInstance();
    pub_ = advertise<geometry_msgs::PoseStamped>(*pnh_, "output_pose", 1);
    pub_edges_ = advertise<jsk_recognition_msgs::EdgeArray>(*pnh_, "output_model_edges", 1);
    pub_polys_ = advertise<jsk_recognition_msgs::PolygonArray>(*pnh_, "output_model_polygons", 1);
    pub_poses_ = advertise<jsk_recognition_msgs::PoseLabeledArray>(*pnh_, "output_model_poses", 1);
    cli_element_corresp_ = nh_->serviceClient<jsk_recognition_msgs::ElementCorrespondence>("get_element_correspondence");
    srv_reset_pose_ = pnh_->advertiseService("reset_estimated_pose", &ElementMatchingPoseEstimation::reset_estimated_pose_cb, this);

    if (publish_tf_) {
      tf_timer_ = pnh_->createTimer(ros::Duration(1.0 / tf_rate),
                                    boost::bind(&ElementMatchingPoseEstimation::tfTimerCallback, this, _1));
    }

    onInitPostProcess();
  }

  void ElementMatchingPoseEstimation::subscribe()
  {
    sub_cloud_.subscribe(*pnh_, "input", 1);
    sub_edge_indices_.subscribe(*pnh_, "edge_indices", 1);
    sub_edges_.subscribe(*pnh_, "edges", 1);
    sub_poly_indices_.subscribe(*pnh_, "polygon_indices", 1);
    sub_polys_.subscribe(*pnh_, "polygons", 1);
    sub_poses_.subscribe(*pnh_, "poses", 1);
    if (approximate_sync_) {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(max_queue_size_);
      async_->connectInput(sub_cloud_, sub_edge_indices_, sub_edges_, sub_poly_indices_, sub_polys_, sub_poses_);
      async_->registerCallback(boost::bind(&ElementMatchingPoseEstimation::estimate,
                                           this, _1, _2, _3, _4, _5, _6));
    }
    else {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(max_queue_size_);
      sync_->connectInput(sub_cloud_, sub_edge_indices_, sub_edges_, sub_poly_indices_, sub_polys_, sub_poses_);
      sync_->registerCallback(boost::bind(&ElementMatchingPoseEstimation::estimate,
                                          this, _1, _2, _3, _4, _5, _6));
    }
  }

  void ElementMatchingPoseEstimation::unsubscribe()
  {
    sub_cloud_.unsubscribe();
    sub_edge_indices_.unsubscribe();
    sub_edges_.unsubscribe();
    sub_poly_indices_.unsubscribe();
    sub_polys_.unsubscribe();
  }

  void ElementMatchingPoseEstimation::estimate(
    const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& edge_indices_msg,
    const jsk_recognition_msgs::EdgeArray::ConstPtr& edges_msg,
    const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& poly_indices_msg,
    const jsk_recognition_msgs::PolygonArray::ConstPtr& polys_msg,
    const jsk_recognition_msgs::PoseLabeledArray::ConstPtr& poses_msg)
  {
    NODELET_DEBUG("estimate function is called.");

    // check size
    {
      int edge_indices_size = edge_indices_msg->cluster_indices.size();
      int edges_size = edges_msg->edges.size();
      if(edge_indices_size != edges_size) {
        NODELET_ERROR("size of edge_indices(%d) and edges(%d) are not same.", edge_indices_size, edges_size);
        return;
      }
      int poly_indices_size = poly_indices_msg->cluster_indices.size();
      int polys_size = polys_msg->polygons.size();
      if(poly_indices_size != polys_size) {
        NODELET_ERROR("size of polygon_indices(%d) and polygons(%d) are not same.", poly_indices_size, polys_size);
        return;
      }
    }

    // check frame_id
    {
      std::string cloud_frame_id = cloud_msg->header.frame_id;
      std::string edge_indices_frame_id = edge_indices_msg->header.frame_id;
      std::string edges_frame_id = edges_msg->header.frame_id;
      std::string poses_frame_id = poses_msg->header.frame_id;
      if (! ((cloud_frame_id == edge_indices_frame_id) && (cloud_frame_id == edges_frame_id) && (cloud_frame_id == poses_frame_id))) {
        NODELET_ERROR("frame_id of cloud(%s), indices(%s), edges(%s), and poses(%s) are not same.",
                      cloud_frame_id.c_str(), edge_indices_frame_id.c_str(), edges_frame_id.c_str(), poses_frame_id.c_str());
        return;
      }
    }

    // update frame_id of estimated pose
    {
      std::string pose_frame_id = estimated_pose_.header.frame_id;
      std::string cloud_frame_id = cloud_msg->header.frame_id;
      if (pose_frame_id == std::string("")) {
        estimated_pose_.header.frame_id = cloud_frame_id;
      } else if (tf_listener_->waitForTransform(pose_frame_id, cloud_frame_id,
                                                cloud_msg->header.stamp, ros::Duration(1.0))) {
        boost::mutex::scoped_lock lock(tf_mutex_);
        tf_listener_->transformPose(cloud_frame_id, estimated_pose_, estimated_pose_);
      } else {
        NODELET_ERROR("No tf transformation from %s to %s is available",
                      pose_frame_id.c_str(), cloud_frame_id.c_str());
        return;
      }
    }

    geometry_msgs::PoseStamped current_pose = estimated_pose_;
    current_pose.header.stamp = cloud_msg->header.stamp;

    // get element correspondence
    jsk_recognition_msgs::EdgeArray model_edges;
    jsk_recognition_msgs::PolygonArray model_polys;
    jsk_recognition_msgs::PoseLabeledArray model_poses;
    {
      std::clock_t begin = clock();
      jsk_recognition_msgs::ElementCorrespondence element_corresp_srv;
      element_corresp_srv.request.input_edges = *edges_msg;
      element_corresp_srv.request.input_polygons = *polys_msg;
      element_corresp_srv.request.input_poses = *poses_msg;
      element_corresp_srv.request.input_pose = current_pose;
      if (! cli_element_corresp_.call(element_corresp_srv)) {
        NODELET_ERROR("failed to call service for element correspondence.");
        return;
      }
      std::clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      NODELET_DEBUG("elapsed time for correspondence service call: %lf [msec].", elapsed_secs * 1000);
      model_edges = element_corresp_srv.response.corresponding_edges;
      model_polys = element_corresp_srv.response.corresponding_polygons;
      model_poses = element_corresp_srv.response.corresponding_poses;
    }

    // get indices vec
    std::vector<pcl::IndicesPtr> edge_indices_vec;
    std::vector<pcl::IndicesPtr> poly_indices_vec;
    for (size_t i = 0; i < edge_indices_msg->cluster_indices.size(); i++) {
      pcl::IndicesPtr indices(new std::vector<int> (edge_indices_msg->cluster_indices[i].indices));
      if (indices->size() > max_edge_indices_) {
        std::random_shuffle(indices->begin(), indices->end());
        pcl::IndicesPtr sub_indices(new std::vector<int> (indices->begin(), indices->begin() + max_edge_indices_));
        edge_indices_vec.push_back(sub_indices);
      } else {
        edge_indices_vec.push_back(indices);
      }
    }
    for (size_t i = 0; i < poly_indices_msg->cluster_indices.size(); i++) {
      pcl::IndicesPtr indices(new std::vector<int> (poly_indices_msg->cluster_indices[i].indices));
      if (indices->size() > max_polygon_indices_) {
        std::random_shuffle(indices->begin(), indices->end());
        pcl::IndicesPtr sub_indices(new std::vector<int> (indices->begin(), indices->begin() + max_polygon_indices_));
        poly_indices_vec.push_back(sub_indices);
      } else {
        poly_indices_vec.push_back(indices);
      }
    }

    // init pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr detected_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointElement>::Ptr model_cloud(new pcl::PointCloud<pcl::PointElement>());
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    pcl::fromROSMsg(*cloud_msg, *cloud);
    extract.setInputCloud(cloud);

    // get pointcloud on edge
    {
      std::clock_t begin = clock();
      for (size_t i = 0; i < edge_indices_vec.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setIndices(edge_indices_vec[i]);
        extract.filter(*extracted_cloud);

        // merge detected edge cloud
        *detected_cloud += *extracted_cloud;

        // generate cloud for model edge cloud
        {
          // generate point for model edge cloud
          pcl::PointElement model_edge_point;
          {
            Eigen::Vector3d start_point;
            Eigen::Vector3d end_point;
            tf::pointMsgToEigen(model_edges.edges[i].start_point, start_point);
            tf::pointMsgToEigen(model_edges.edges[i].end_point, end_point);
            if (start_point.isZero() && end_point.isZero()) {
              model_edge_point.label = pcl::registration::ELEMENT_INVALID;
            } else {
              jsk_recognition_utils::Segment edge(start_point.cast<float> (), end_point.cast<float> ());
              model_edge_point.edge = edge;
              model_edge_point.label = pcl::registration::ELEMENT_EDGE;
            }
          }

          // merge model edge cloud
          for (size_t j = 0; j < edge_indices_vec[i]->size(); j++) {
            model_cloud->points.push_back(model_edge_point);
          }
        }
      }
      std::clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      NODELET_DEBUG("elapsed time for getting edge cloud: %lf [msec].", elapsed_secs * 1000);
    }

    // get pointcloud on polygon
    {
      std::clock_t begin = clock();
      for (size_t i = 0; i < poly_indices_vec.size(); i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        extract.setIndices(poly_indices_vec[i]);
        extract.filter(*extracted_cloud);

        // merge detected polygon cloud
        *detected_cloud += *extracted_cloud;

        // generate cloud for model polygon cloud
        {
          // generate point for model polygon cloud
          pcl::PointElement model_poly_point;
          {
            if (model_polys.polygons[i].polygon.points.size() == 0) {
              model_poly_point.label = pcl::registration::ELEMENT_INVALID;
            } else {
              Vertices vertices;
              for (size_t j = 0; j < model_polys.polygons[i].polygon.points.size(); j++) {
                Eigen::Vector3d vertex;
                tf::pointMsgToEigen(toPoint(model_polys.polygons[i].polygon.points[j]), vertex);
                vertices.push_back(vertex.cast<float> ());
              }
              jsk_recognition_utils::Polygon poly(vertices);
              poly.decomposeToTriangles();
              model_poly_point.polygon = poly;
              model_poly_point.label = pcl::registration::ELEMENT_POLYGON;
            }
          }

          // merge model polygon cloud
          for (size_t j = 0; j < poly_indices_vec[i]->size(); j++) {
            model_cloud->points.push_back(model_poly_point);
          }
        }
      }
      std::clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      NODELET_DEBUG("elapsed time for getting polygon cloud: %lf [msec].", elapsed_secs * 1000);
    }

    // get pointcloud on pose
    {
      std::clock_t begin = clock();
      for (size_t i = 0; i < poses_msg->poses.size(); i++) {
        // merge detected pose point
        {
          pcl::PointXYZ detected_pose_point;
          detected_pose_point.x = poses_msg->poses[i].pose.pose.position.x;
          detected_pose_point.y = poses_msg->poses[i].pose.pose.position.y;
          detected_pose_point.z = poses_msg->poses[i].pose.pose.position.z;

          detected_cloud->points.push_back(detected_pose_point);
        }

        // merge model pose point
        {
          // generate point for model pose
          pcl::PointElement model_pose_point;
          {
            if (model_poses.poses[i].label == std::string("")) {
              model_pose_point.label = pcl::registration::ELEMENT_INVALID;
            } else {
              Eigen::Vector3d point;
              tf::pointMsgToEigen(model_poses.poses[i].pose.pose.position, point);
              model_pose_point.point = point.cast<float> ();
              model_pose_point.label = pcl::registration::ELEMENT_POINT;
            }
          }

          model_cloud->points.push_back(model_pose_point);
        }
      }
      std::clock_t end = clock();
      double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
      NODELET_DEBUG("elapsed time for getting pose cloud: %lf [msec].", elapsed_secs * 1000);
    }

    // keep current pose
    {
      Eigen::Affine3f current_trans;
      tf::poseMsgToEigen(current_pose.pose, current_trans);

      for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 2; j++) {
          // generate fixed point
          Eigen::Vector4f transed_fixed_vec;
          {
            Eigen::Vector4f fixed_vec = Eigen::Vector4f::Zero(4);
            double fixed_vec_length = 0.1;
            fixed_vec(i) = fixed_vec_length;
            if (j == 1) {
              fixed_vec(i) = -1 * fixed_vec(i);
            }
            fixed_vec(3) = 1;
            transed_fixed_vec = current_trans * fixed_vec;
          }

          // merge detected pose point
          {
            pcl::PointXYZ detected_pose_point;
            detected_pose_point.x = transed_fixed_vec(0);
            detected_pose_point.y = transed_fixed_vec(1);
            detected_pose_point.z = transed_fixed_vec(2);

            detected_cloud->points.push_back(detected_pose_point);
          }

          // merge model pose point
          {
            // generate point for model pose
            pcl::PointElement model_pose_point;
            model_pose_point.point = transed_fixed_vec.block(0, 0, 3, 1);
            model_pose_point.label = pcl::registration::ELEMENT_FIXED;

            model_cloud->points.push_back(model_pose_point);
          }
        }
      }
    }

    detected_cloud->width = (int)detected_cloud->points.size();
    detected_cloud->height = 1;
    model_cloud->width = (int)model_cloud->points.size();
    model_cloud->height = 1;

    // estimate transformation
    Eigen::Affine3f estimated_trans_local = Eigen::Affine3f::Identity();
    Eigen::Affine3f estimated_trans_local_inv;
    trans_est_.estimateRigidTransformation(*detected_cloud, *model_cloud, estimated_trans_local.matrix());
    estimated_trans_local_inv = estimated_trans_local.inverse(); // inverse transformation because src and dest is flipped in estimation
    // std::cout << "estimated transformation" << std::endl << estimated_trans_local.matrix() << std::endl;

    // consider initial transformation
    {
      Eigen::Affine3f current_trans;
      Eigen::Affine3f estimated_trans;
      tf::poseMsgToEigen(current_pose.pose, current_trans);
      estimated_trans = estimated_trans_local_inv * current_trans;
      tf::poseEigenToMsg(estimated_trans, current_pose.pose);
    }

    // transform frame of estimated pose
    if (frame_id_ == std::string("")) {
      estimated_pose_ = current_pose;
    } else {
      if (tf_listener_->waitForTransform(current_pose.header.frame_id, frame_id_,
                                         current_pose.header.stamp, ros::Duration(1.0))) {
        tf_listener_->transformPose(frame_id_, current_pose, estimated_pose_);
      } else {
        NODELET_ERROR("No tf transformation from %s to %s is available",
                      current_pose.header.frame_id.c_str(), frame_id_.c_str());
        estimated_pose_ = current_pose;
      }
    }

    // publish estimated pose
    pub_.publish(estimated_pose_);

    // publish tf
    if (publish_tf_) {
      boost::mutex::scoped_lock lock(tf_mutex_);
      tf::poseMsgToTF(estimated_pose_.pose, estimated_tf_);
    }

    // publish model edges
    if (publish_model_element_) {
      pub_edges_.publish(model_edges);
      pub_polys_.publish(model_polys);
      pub_poses_.publish(model_poses);
    }

    // display debug viewer
    if (debug_viewer_) {
      // transform pointcloud with estimated transformation
      pcl::PointCloud<pcl::PointElement>::Ptr model_cloud_transformed(new pcl::PointCloud<pcl::PointElement>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr detected_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
      pcl::transformPointCloud(*model_cloud, *model_cloud_transformed, estimated_trans_local);
      pcl::transformPointCloud(*detected_cloud, *detected_cloud_transformed, estimated_trans_local_inv);

      // visualize pointcloud
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor(0, 0, 0);
      int point_idx = 0;
      for (size_t i = 0; i < edge_indices_vec.size(); i++) {
        pcl::PointCloud<pcl::PointElement>::Ptr model_edge_cluster_cloud(new pcl::PointCloud<pcl::PointElement>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr detected_edge_cluster_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr detected_edge_cluster_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>());
        for (size_t j = 0; j < edge_indices_vec[i]->size(); j++) {
          model_edge_cluster_cloud->points.push_back(model_cloud->points[point_idx]);
          detected_edge_cluster_cloud->points.push_back(detected_cloud->points[point_idx]);
          detected_edge_cluster_cloud_transformed->points.push_back(detected_cloud_transformed->points[point_idx]);
          point_idx++;
        }
        std_msgs::ColorRGBA cluster_color = jsk_topic_tools::colorCategory20(i);
        // model_cloud
        std::string label_model_cloud = "model_cloud" + std::to_string(i);
        viewer->addPointCloudNormals<pcl::PointElement>(model_edge_cluster_cloud, 6, 1.0, label_model_cloud);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  cluster_color.r, cluster_color.g, cluster_color.b, label_model_cloud);
        // detected_edge_cluster_cloud
        std::string label_detected_cloud = "detected_cloud" + std::to_string(i);
        viewer->addPointCloud<pcl::PointXYZ>(detected_edge_cluster_cloud, label_detected_cloud);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, label_detected_cloud);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  cluster_color.r, cluster_color.g, cluster_color.b, label_detected_cloud);
        // detected_edge_cluster_cloud_transformed
        std::string label_detected_cloud_transformed = "detected_cloud_transformed" + std::to_string(i);
        viewer->addPointCloud<pcl::PointXYZ>(detected_edge_cluster_cloud_transformed, label_detected_cloud_transformed);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, label_detected_cloud_transformed);
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,
                                                  cluster_color.r, cluster_color.g, cluster_color.b, label_detected_cloud_transformed);
      }
      // visualize loop
      while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
      viewer->removeAllPointClouds();
    }
  }

  bool ElementMatchingPoseEstimation::reset_estimated_pose_cb(
    jsk_recognition_msgs::ResetPose::Request &req,
    jsk_recognition_msgs::ResetPose::Response &res)
  {
    boost::mutex::scoped_lock lock(tf_mutex_);
    if (tf_listener_->waitForTransform(estimated_pose_.header.frame_id, req.pose.header.frame_id,
                                       req.pose.header.stamp, ros::Duration(1.0))) {
      tf_listener_->transformPose(estimated_pose_.header.frame_id, req.pose, estimated_pose_);
    } else {
      NODELET_WARN("No tf transformation from %s to %s is available.",
                   estimated_pose_.header.frame_id.c_str(), req.pose.header.frame_id.c_str());
      estimated_pose_.pose = req.pose.pose;
      estimated_pose_.header.frame_id = req.pose.header.frame_id;
    }
    return true;
  }

  void ElementMatchingPoseEstimation::tfTimerCallback(
    const ros::TimerEvent& event)
  {
    if (estimated_pose_.header.frame_id == std::string("")) {
      NODELET_WARN("frame_id of estimated pose is not initialized.");
      return;
    }
    boost::mutex::scoped_lock lock(tf_mutex_);
    static tf::TransformBroadcaster tf_broadcaster;
    tf_broadcaster.sendTransform(tf::StampedTransform(estimated_tf_, ros::Time::now(),
                                                      estimated_pose_.header.frame_id, output_frame_id_));
  }

  geometry_msgs::Point ElementMatchingPoseEstimation::toPoint(geometry_msgs::Point32 pt)
  {
    geometry_msgs::Point point;
    point.x = pt.x;
    point.y = pt.y;
    point.z = pt.z;
    return point;
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ElementMatchingPoseEstimation, nodelet::Nodelet);
