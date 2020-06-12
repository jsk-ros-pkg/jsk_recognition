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
#include "jsk_pcl_ros/colorize_random_points_RF.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/common/centroid.h>

namespace jsk_pcl_ros
{
  void ColorizeMapRandomForest::onInit(void)
  {
    ConnectionBasedNodelet::onInit();
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    
    srand(time(NULL));

    double tmp_radius, tmp_pass, tmp_pass2;
    int tmp_sum_num;
    tmp_radius = 0.03;
    tmp_pass = 0.03;
    tmp_pass2 = 0.06;
    tmp_sum_num = 100;
    if (!pnh_->getParam("rs", tmp_radius))
      {
        NODELET_WARN("~rs is not specified, so set to 0.03");
      }

    if (!pnh_->getParam("po", tmp_pass))
      {
        NODELET_WARN("~po is not specified, so set to 0.03");
      }

    if (!pnh_->getParam("po2", tmp_pass2))
      {
        NODELET_WARN("~po2 is not specified, so set to 0.06");
      }

    if (!pnh_->getParam("sum_num", tmp_sum_num))
      {
        NODELET_WARN("~sum_num is not specified, so set to 100");
      }

    radius_search_ = tmp_radius;
    pass_offset_ = tmp_pass;
    pass_offset2_ = tmp_pass2;
    sum_num_ = tmp_sum_num;
    onInitPostProcess();
 }

  void ColorizeMapRandomForest::subscribe()
  {
    sub_input_ = pnh_->subscribe("input", 1, &ColorizeMapRandomForest::extract, this);
  }

  void ColorizeMapRandomForest::unsubscribe()
  {
    sub_input_.shutdown();
  }
  
  void ColorizeMapRandomForest::extract(const sensor_msgs::PointCloud2 pc)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    std::vector<int> indices;
    pcl::fromROSMsg(pc, *cloud);

    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (std::string("z"));
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud);

    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    ne.setInputCloud (cloud);
    NODELET_DEBUG("normal estimate");
    ne.setSearchMethod (tree2);
    ne.setRadiusSearch (0.02);
    ne.compute (*cloud_normals);

    for (int cloud_index = 0; cloud_index <  cloud_normals->points.size(); cloud_index++){
      cloud_normals->points[cloud_index].x = cloud->points[cloud_index].x;
      cloud_normals->points[cloud_index].y = cloud->points[cloud_index].y;
      cloud_normals->points[cloud_index].z = cloud->points[cloud_index].z;
      // cloud_normals->points[cloud_index].rgba = cloud->points[cloud_index].rgba;
    }

    int result_counter=0, call_counter = 0;
    pcl::PointXYZRGBNormal max_pt,min_pt;
    pcl::getMinMax3D(*cloud_normals, min_pt, max_pt);
    for (int i = 0 ; i < 200; i++){
      NODELET_DEBUG("fpfh %d / 200", i);
      double lucky = 0, lucky2 = 0;
      std::string axis("x"),other_axis("y");
      int rand_xy = rand()%2;
      if (rand_xy == 0){
        lucky = min_pt.x - pass_offset_ + (max_pt.x - min_pt.x - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
        lucky2 = min_pt.y + (max_pt.y - min_pt.y) * 1.0 * rand() / RAND_MAX;
      }
      else {
        lucky = min_pt.y - pass_offset_ + (max_pt.y - min_pt.y - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
        lucky2 = min_pt.x + (max_pt.x - min_pt.x) * 1.0 * rand() / RAND_MAX;
        axis = std::string("y");
        other_axis = std::string("x");
      }

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals_pass (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pcl::IndicesPtr indices_x(new std::vector<int>());
      pass.setInputCloud (cloud_normals);
      pass.setFilterFieldName (axis);
      float small = std::min(lucky, lucky + pass_offset_);
      float large = std::max(lucky, lucky + pass_offset_);
      pass.setFilterLimits (small, large);
      pass.filter (*cloud_normals_pass);
      pass.setInputCloud (cloud_normals_pass);
      pass.setFilterFieldName (other_axis);
      float small2 = std::min(lucky2, lucky2 + pass_offset2_);
      float large2 = std::max(lucky2, lucky2 + pass_offset2_);
      pass.setFilterLimits (small2, large2);
      pass.filter (*cloud_normals_pass);

      std::vector<int> tmp_indices;
      pcl::removeNaNFromPointCloud(*cloud_normals_pass, *cloud_normals_pass, tmp_indices);
      if(cloud_normals_pass->points.size() == 0){
        continue;
      }
      pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
      pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
      fpfh.setNumberOfThreads(8);
      fpfh.setInputCloud (cloud_normals_pass);
      fpfh.setInputNormals (cloud_normals_pass);
      fpfh.setSearchMethod (tree4);
      fpfh.setRadiusSearch (radius_search_);
      fpfh.compute (*fpfhs);
      int target_id, max_value;
      if ((int)fpfhs->points.size() - sum_num_ - 1 < 1){
        target_id = 0;
        max_value = (int)fpfhs->points.size();
      }else{
        target_id = rand() % ((int)fpfhs->points.size() - sum_num_ - 1);
        max_value = target_id + sum_num_;
      }
      std::vector<double> result;
      bool success = true;

      for(int index = 0; index < 33; index++){
        float sum_hist_points = 0;
        for(int kndex = target_id; kndex < max_value;kndex++)
          {
            if(pcl_isnan(fpfhs->points[kndex].histogram[index])){
              success = false;
            }else{
              sum_hist_points+=fpfhs->points[kndex].histogram[index];
            }
          }
        result.push_back( sum_hist_points/sum_num_ );
      }

      bool cloth = false;
      if(success){
        call_counter++;
        ros::ServiceClient client = pnh_->serviceClient<ml_classifiers::ClassifyData>("classify_server");
        ml_classifiers::ClassifyData srv;
        ml_classifiers::ClassDataPoint class_data_point;
        class_data_point.point = result;
        srv.request.data.push_back(class_data_point);
        if(client.call(srv))
          if (atoi(srv.response.classifications[0].c_str()) == 0)
            cloth = true;
        NODELET_DEBUG("response result : %s", srv.response.classifications[0].c_str());
      }else{
        continue;
      }

      Eigen::Vector4f c;
      pcl::compute3DCentroid (*cloud_normals_pass, c);

      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> search_octree (0.05);
      search_octree.setInputCloud (cloud_normals);
      search_octree.addPointsFromInputCloud ();

      pcl::PointXYZRGBNormal point_a;
      point_a.x = c[0];point_a.y = c[1];point_a.z = c[2];
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      search_octree.radiusSearch(point_a, 0.05, k_indices, k_sqr_distances);

      for(int index = 0; index < k_indices.size(); index++){
        int id = k_indices[index];
        if(cloth){
          cloud_normals->points[id].r = std::min(std::max(255.0 - sqrt(k_sqr_distances[index]) * 2000, 0.0) + cloud_normals->points[id].r, 255.0);
        }else{
          cloud_normals->points[id].b = std::min(std::max(255.0 - sqrt(k_sqr_distances[index]) * 2000, 0.0) + cloud_normals->points[id].b, 255.0);
        }
      }
    }
    sensor_msgs::PointCloud2 _pointcloud2;
    pcl::toROSMsg(*cloud_normals, _pointcloud2);
    _pointcloud2.header = pc.header;
    _pointcloud2.is_dense = false;
    pub_.publish(_pointcloud2);
  }
}


PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ColorizeMapRandomForest, nodelet::Nodelet);
