/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, yuto_inagaki and JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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


#include "jsk_pcl_ros/concave_hull.h"
#include <pluginlib/class_list_macros.h>


#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void ConcaveHull::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &ConcaveHull::extract, this);

    pub_ = pnh_->advertise<jsk_pcl_ros::PointsArray>("concave_results", 1);
    pub2_ = pnh_->advertise<sensor_msgs::PointCloud2>("concave_result", 1);

    if (!pnh_->getParam("mode", mode_))
      {
        ROS_WARN("~mode is not specified, set 1");
        mode_ = 1;
      }

    if (!pnh_->getParam("angular_threshold", angular_threshold_))
      {
        ROS_WARN("~angular_threshold is not specified, set 10.0");
        angular_threshold_ = 10.0;
      }

    if (!pnh_->getParam("min_inliers", min_inliers_))
      {
        ROS_WARN("~min_inliers is not specified, set 300");
        min_inliers_ = 300;
      }

    if (!pnh_->getParam("mps_distance_threshold", mps_distance_threshold_))
      {
        ROS_WARN("~mps_distance_threshold is not specified, set 0.05");
        mps_distance_threshold_ = 0.05;
      }

    if (!pnh_->getParam("approx_threshold", approx_threshold_))
      {
        ROS_WARN("~approx_threshold is not specified, set 0.02");
        approx_threshold_ = 0.02;
      }

    if (!pnh_->getParam("max_depth_change_factor", max_depth_change_factor_))
      {
        ROS_WARN("~max_depth_change_factor is not specified, set 0.02");
        max_depth_change_factor_ = 0.02;
      }

    if (!pnh_->getParam("normal_smoothingsize", normal_smoothingsize_))
      {
        ROS_WARN("~normal_smoothingsize is not specified, set 20.0");
        normal_smoothingsize_ = 20.0;
      }

    if (!pnh_->getParam("refinement_threshold", refinement_threshold_))
      {
        ROS_WARN("~refinement_threshold is not specified, set 0.02");
        refinement_threshold_ = 0.02;
      }

  }

  void ConcaveHull::extract(const sensor_msgs::PointCloud2 pc)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PCLPointCloud2 pcl_pc;
    std::vector<int> indices;
    pcl_conversions::toPCL(pc, pcl_pc);
    pcl::fromPCLPointCloud2 (pcl_pc, *cloud);
    //    cloud->is_dense = false;
    //    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    bool depth_dependent_ = true;
    bool polygon_refinement_ = false;
    bool refine_ = false;

#if 1

    pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setNormalEstimationMethod (ne.COVARIANCE_MATRIX);
    ne.setMaxDepthChangeFactor ((float)max_depth_change_factor_);//0.02
    ne.setNormalSmoothingSize ((float)normal_smoothingsize_);//20.0
    // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    // ne.setSearchMethod (tree);
    // ne.setRadiusSearch (0.03);

    pcl::PlaneRefinementComparator<pcl::PointXYZ, pcl::Normal, pcl::Label>::Ptr refinement_compare (new pcl::PlaneRefinementComparator<pcl::PointXYZ, pcl::Normal, pcl::Label> ());
    refinement_compare->setDistanceThreshold (refinement_threshold_, depth_dependent_);
    
    pcl::OrganizedMultiPlaneSegmentation<pcl::PointXYZ, pcl::Normal, pcl::Label> mps;
    mps.setMinInliers (min_inliers_);
    mps.setAngularThreshold (0.017453 * angular_threshold_); //3 degrees
    mps.setDistanceThreshold (mps_distance_threshold_); //2cm
    mps.setRefinementComparator (refinement_compare);
    mps.setProjectPoints(true);
    mps.setMaximumCurvature(0.01);

    std::vector<pcl::PlanarRegion<pcl::PointXYZ>, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;
    pcl::PointCloud<pcl::PointXYZ>::Ptr contour (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr approx_contour (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::Normal>::Ptr normal_cloud (new pcl::PointCloud<pcl::Normal>);
    ne.setInputCloud (cloud);
    ne.compute (*normal_cloud);
    mps.setInputNormals (normal_cloud);
    mps.setInputCloud (cloud);
    if (refine_)
      mps.segmentAndRefine (regions);
    else
      mps.segment (regions);

    std::vector<sensor_msgs::PointCloud2> approx_contours;
    ROS_INFO("approx_contours num : %ld", regions.size());
    for (size_t i = 0; i < regions.size (); i++)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr approx_contour(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PlanarPolygon<pcl::PointXYZ> approx_polygon;
        pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_pointcloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::ConvexHull<pcl::PointXYZ> chull;
        double tmp_approx_threshold;
        switch(mode_){
        case 1:
          tmp_approx_threshold = approx_threshold_ * (int)(regions[i].getContour().size()) / 10;
          if (tmp_approx_threshold < approx_threshold_)
            tmp_approx_threshold = approx_threshold_;
          pcl::approximatePolygon (regions[i], approx_polygon, tmp_approx_threshold, polygon_refinement_);
          approx_contour->points = approx_polygon.getContour();
          break;
        case 2:
          tmp_pointcloud->points = regions[i].getContour();
          chull.setInputCloud (tmp_pointcloud);
          chull.reconstruct (*approx_contour);
          break;
        }
        ROS_INFO("    %ld: %ld -> %ld points", i,  regions[i].getContour().size(),approx_contour->points.size());

        sensor_msgs::PointCloud2 pointcloud2;
        pcl::toROSMsg(*approx_contour, pointcloud2);
        pointcloud2.header = pc.header;
        pointcloud2.is_dense = false;
        pub2_.publish(pointcloud2);

        if(approx_contour->points.size() > 2){
          approx_contours.push_back(pointcloud2);
        }
      }

    if(approx_contours.size() > 0){
      jsk_pcl_ros::PointsArray points_array;
      points_array.cloud_list = approx_contours;
      pub_.publish(points_array);
    }

#endif

#if 0
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.01f, 0.01f, 0.01f);
    sor.filter (*cloud);

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (min_inliers_);
    seg.setDistanceThreshold (0.02);

    std::vector<sensor_msgs::PointCloud2> approx_contours;
    while(cloud->points.size() > 10){
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
          break;
        }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud);
      extract.setIndices (inliers);
      extract.setNegative (false);

      // Get the points associated with the planar surface
      extract.filter (*cloud_plane);
      std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Project the model inliers
      pcl::PointCloud<pcl::PointXYZ>::Ptr
        cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>),
        cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::ProjectInliers<pcl::PointXYZ> proj;
      proj.setModelType (pcl::SACMODEL_PLANE);
      proj.setIndices (inliers);
      proj.setInputCloud (cloud_plane);
      proj.setModelCoefficients (coefficients);
      proj.filter (*cloud_projected);

      pcl::PointCloud<pcl::PointXYZ>::Ptr approx_contour(new pcl::PointCloud<pcl::PointXYZ>());
      switch(mode_){
        //concave
      case 1:
        concave_hull(cloud_projected,approx_contour);
        break;
        //convex
      case 2:
        convex_hull(cloud_projected,approx_contour);
        break;
        //rectangle
      default:
        rectangle(cloud_projected,approx_contour);
        break;
      }

      // Remove the planar inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_filtered);
      *cloud = *cloud_filtered;

      sensor_msgs::PointCloud2 pointcloud2;
      pcl::toROSMsg(*cloud_projected, pointcloud2);
      pointcloud2.header = pc.header;
      pointcloud2.is_dense = false;
      pub2_.publish(pointcloud2);

      if(approx_contour->points.size() > 2){
        approx_contours.push_back(pointcloud2);
      }
    }

    if(approx_contours.size() > 0){
      jsk_pcl_ros::PointsArray points_array;
      points_array.cloud_list = approx_contours;
      pub_.publish(points_array);
    }

#endif

  }

  void ConcaveHull::concave_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud)
  {
    // Create a Concave Hull representation of the projected inliers
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ConcaveHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.setAlpha (0.1);
    chull.reconstruct (*result_cloud);
  }

  void ConcaveHull::convex_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud)
  {
    pcl::ConvexHull<pcl::PointXYZ> chull;
    chull.setInputCloud (cloud);
    chull.reconstruct (*result_cloud);
  }

  void ConcaveHull::rectangle(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud)
  {
  }

}

typedef jsk_pcl_ros::ConcaveHull ConcaveHull;
PLUGINLIB_DECLARE_CLASS (jsk_pcl_ros, ConcaveHull, ConcaveHull, nodelet::Nodelet);
