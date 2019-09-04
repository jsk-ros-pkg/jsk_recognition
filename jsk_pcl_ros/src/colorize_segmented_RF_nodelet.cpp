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
#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros/colorize_segmented_RF.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  void ColorizeRandomForest::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &ColorizeRandomForest::extract, this);

    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/zero", 1);
    pub2_ = pnh_->advertise<sensor_msgs::PointCloud2>("output/nonzero", 1);

    srand(time(NULL));

    double tmp_radius = 0.03, tmp_pass = 0.03, tmp_pass2 = 0.06;
    sum_num_ = 100;
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
    if (!pnh_->getParam("sum_num", sum_num_))
      {
        NODELET_WARN("~sum_num is not specified, so set to 100");
      }

    radius_search_ = tmp_radius;
    pass_offset_ = tmp_pass;
    pass_offset2_ = tmp_pass2;
  }

  void ColorizeRandomForest::extract(const sensor_msgs::PointCloud2 pc)
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

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.025);
    ec.setMinClusterSize (200);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloth_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr noncloth_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    int cluster_num = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        NODELET_DEBUG("Calculate time %d / %ld", cluster_num  , cluster_indices.size());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (cloud->points[*pit]);
        cloud_cluster->is_dense = true;
        cluster_num ++ ;

        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
        ne.setInputCloud (cloud_cluster);
        ne.setSearchMethod (tree);
        ne.setRadiusSearch (0.02);
        ne.compute (*cloud_normals);

        for (int cloud_index = 0; cloud_index <  cloud_normals->points.size(); cloud_index++){
          cloud_normals->points[cloud_index].x = cloud_cluster->points[cloud_index].x;
          cloud_normals->points[cloud_index].y = cloud_cluster->points[cloud_index].y;
          cloud_normals->points[cloud_index].z = cloud_cluster->points[cloud_index].z;
        }

        int result_counter=0, call_counter = 0;
        pcl::PointXYZRGBNormal max_pt,min_pt;
        pcl::getMinMax3D(*cloud_normals, min_pt, max_pt);

        for (int i = 0 ; i < 30; i++){
          double lucky = 0, lucky2 = 0;
          std::string axis("x"), other_axis("y");
          int rand_xy = rand()%2;
          if (rand_xy == 0){
            lucky = min_pt.x - pass_offset_ + (max_pt.x - min_pt.x - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
            lucky2 = min_pt.y + (max_pt.y - min_pt.y) * 1.0 * rand() / RAND_MAX;
          }else {
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

          if(cloud_normals_pass->points.size() == 0)
            continue;

          pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
          pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
          pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
          fpfh.setNumberOfThreads(8);
          fpfh.setInputCloud (cloud_normals_pass);
          fpfh.setInputNormals (cloud_normals_pass);
          fpfh.setSearchMethod (tree);
          fpfh.setRadiusSearch (radius_search_);
          fpfh.compute (*fpfhs);

          if((int)fpfhs->points.size() == 0)
            continue;

          std::vector<double> result;
          int target_id, max_value;
          if ((int)fpfhs->points.size() - sum_num_ - 1 < 1){
            target_id = 0;
            max_value = (int)fpfhs->points.size();
          }else{
            target_id = rand() % ((int)fpfhs->points.size() - sum_num_ - 1);
            max_value = target_id + sum_num_;
          }

          bool has_nan = false;
          for(int index = 0; index < 33; index++){
            float sum_hist_points = 0;
            for(int kndex = target_id; kndex < max_value;kndex++)
              {
                sum_hist_points+=fpfhs->points[kndex].histogram[index];
              }
            result.push_back( sum_hist_points/sum_num_ );
          }

          for(int x = 0; x < result.size(); x ++){
            if(pcl_isnan(result[x]))
              has_nan = true;
          }
          if(has_nan)
            break;

          call_counter++;
          ros::ServiceClient client = pnh_->serviceClient<ml_classifiers::ClassifyData>("classify_server");
          ml_classifiers::ClassifyData srv;
          ml_classifiers::ClassDataPoint class_data_point;
          class_data_point.point = result;
          srv.request.data.push_back(class_data_point);
          if(client.call(srv))
            if (atoi(srv.response.classifications[0].c_str()) == 0)
              result_counter += 1;
          NODELET_DEBUG("response result : %s", srv.response.classifications[0].c_str());
        }

        if(result_counter >= call_counter / 2){
          NODELET_DEBUG("Result == 0 because counter is %d / %d", result_counter, call_counter);
        }
        else{
          NODELET_DEBUG("Result != 0 because counter is %d / %d", result_counter, call_counter);
        }

        for (int i = 0; i < cloud_cluster->points.size(); i++){
          if(result_counter >= call_counter / 2){
            cloth_cloud_cluster->points.push_back (cloud_cluster->points[i]);
          }
          else{
            noncloth_cloud_cluster->points.push_back (cloud_cluster->points[i]);
          }
        }
      }
    sensor_msgs::PointCloud2 cloth_pointcloud2;
    pcl::toROSMsg(*cloth_cloud_cluster, cloth_pointcloud2);
    cloth_pointcloud2.header = pc.header;
    cloth_pointcloud2.is_dense = false;
    pub_.publish(cloth_pointcloud2);

    sensor_msgs::PointCloud2 noncloth_pointcloud2;
    pcl::toROSMsg(*noncloth_cloud_cluster, noncloth_pointcloud2);
    noncloth_pointcloud2.header = pc.header;
    noncloth_pointcloud2.is_dense = false;
    pub2_.publish(noncloth_pointcloud2);
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ColorizeRandomForest, nodelet::Nodelet);
