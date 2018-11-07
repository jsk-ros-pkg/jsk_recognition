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

#include <jsk_pcl_ros/convex_connected_voxels.h>

namespace jsk_pcl_ros
{   
    void ConvexConnectedVoxels::onInit()
    {
       DiagnosticNodelet::onInit();
       pub_indices_ = advertise<
          jsk_recognition_msgs::ClusterPointIndices>(
             *pnh_, "output/indices", 1);
       onInitPostProcess();
   }

    void ConvexConnectedVoxels::subscribe()
    {
       sub_indices_ = pnh_->subscribe(
          "input/indices", 10,
          &ConvexConnectedVoxels::indices_cb, this);
       sub_cloud_ = pnh_->subscribe(
          "input/cloud", 10,
          &ConvexConnectedVoxels::cloud_cb, this);
    }
   
    void ConvexConnectedVoxels::unsubscribe()
    {
       sub_cloud_.shutdown();
       sub_indices_.shutdown();
    }

    void ConvexConnectedVoxels::cloud_cb(
       const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
    {
       // ROS_INFO("PROCESSING CLOUD CALLBACK");
       
       // boost::mutex::scoped_lock lock(this->mutex_);
       // vital_checker_->poke();
       pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
       pcl::fromROSMsg(*cloud_msg, *cloud);
       std::vector<pcl::PointCloud<PointT>::Ptr> cloud_clusters;
       std::vector<pcl::PointCloud<pcl::Normal>::Ptr> normal_clusters;
       pcl::PointCloud<pcl::PointXYZ>::Ptr centroids(
          new pcl::PointCloud<pcl::PointXYZ>);
       this->segmentCloud(
          cloud, this->indices_, cloud_clusters, normal_clusters, centroids);
       std::vector<std::vector<int> > neigbour_idx;
       this->nearestNeigborSearch(centroids, neigbour_idx, 4);
       boost::shared_ptr<jsk_pcl_ros::RegionAdjacencyGraph> rag(
          new jsk_pcl_ros::RegionAdjacencyGraph);
       rag->generateRAG(
          cloud_clusters,
          normal_clusters,
          centroids,
          neigbour_idx,
          rag->RAG_EDGE_WEIGHT_CONVEX_CRITERIA);
       rag->splitMergeRAG(0.0);
       std::vector<int> labelMD;
       rag->getCloudClusterLabels(labelMD);
       std::map<int, pcl::PointIndices> _indices;
       this->getConvexLabelCloudIndices(
          cloud_clusters, cloud, labelMD, _indices);
       std::vector<pcl::PointIndices> all_indices;
       for (std::map<int, pcl::PointIndices>::iterator it = _indices.begin();
            it != _indices.end(); it++) {
          all_indices.push_back((*it).second);
       }

       // ROS_INFO("Size: %ld", _indices.size());

       jsk_recognition_msgs::ClusterPointIndices ros_indices;
       ros_indices.cluster_indices = pcl_conversions::convertToROSPointIndices(
          all_indices, cloud_msg->header);
       ros_indices.header = cloud_msg->header;
       pub_indices_.publish(ros_indices);
    }

    void ConvexConnectedVoxels::indices_cb(
       const jsk_recognition_msgs::ClusterPointIndices &indices_msg)
    {
       // boost::mutex::scoped_lock lock(this->mutex_);
       vital_checker_->poke();
       this->indices_.clear();
       std::vector<pcl_msgs::PointIndices> indices =
          indices_msg.cluster_indices;
       for (int i = 0; i < indices.size(); i++) {
          pcl::PointIndices _index;
          pcl_conversions::toPCL(indices[i], _index);
          this->indices_.push_back(_index);
       }
    }
   
    void ConvexConnectedVoxels::segmentCloud(
        const pcl::PointCloud<PointT>::Ptr cloud,
        const std::vector<pcl::PointIndices> &indices,
        std::vector<pcl::PointCloud<PointT>::Ptr> &cloud_clusters,
        std::vector<pcl::PointCloud<pcl::Normal>::Ptr> &normal_clusters,
        pcl::PointCloud<pcl::PointXYZ>::Ptr centroids)
    {
        boost::mutex::scoped_lock lock(this->mutex_);
        pcl::ExtractIndices<PointT>::Ptr eifilter(
           new pcl::ExtractIndices<PointT>);
        eifilter->setInputCloud(cloud);
        for (int i = 0; i < indices.size(); i++) {
           pcl::PointIndices::Ptr index(new pcl::PointIndices());
           index->indices = indices[i].indices;
           eifilter->setIndices(index);
           pcl::PointCloud<PointT>::Ptr tmp_cloud(
              new pcl::PointCloud<PointT>);
           eifilter->filter(*tmp_cloud);
           if (tmp_cloud->width > 0) {
             Eigen::Vector4f centroid;
             pcl::compute3DCentroid<PointT, float>(*cloud, *index, centroid);
             float ct_x = static_cast<float>(centroid[0]);
             float ct_y = static_cast<float>(centroid[1]);
             float ct_z = static_cast<float>(centroid[2]);
             if (!std::isnan(ct_x) && !std::isnan(ct_y) && !std::isnan(ct_z)) {
                pcl::PointCloud<pcl::Normal>::Ptr s_normal(
                   new pcl::PointCloud<pcl::Normal>);
                this->estimatePointCloudNormals(
                   tmp_cloud, s_normal, 40, 0.05, false);
                normal_clusters.push_back(s_normal);
                centroids->push_back(pcl::PointXYZ(ct_x, ct_y, ct_z));
                cloud_clusters.push_back(tmp_cloud);
             }
          }
        }
    }
   
    void ConvexConnectedVoxels::estimatePointCloudNormals(
       const pcl::PointCloud<PointT>::Ptr cloud,
       pcl::PointCloud<pcl::Normal>::Ptr s_normal,
       const int k, const double radius, bool ksearch)
    {
       pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
       ne.setInputCloud(cloud);
       ne.setNumberOfThreads(8);
       pcl::search::KdTree<PointT>::Ptr tree(
          new pcl::search::KdTree<PointT> ());
       ne.setSearchMethod(tree);
       if (ksearch) {
          ne.setKSearch(k);
       } else {
          ne.setRadiusSearch(radius);
       }
       ne.compute(*s_normal);
    }

    void ConvexConnectedVoxels::nearestNeigborSearch(
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
      std::vector<std::vector<int> > &pointIndices,
      const int k, const double radius, bool isneigbour)
    {
      pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
      kdtree.setInputCloud(cloud);
      std::vector<std::vector<float> > pointSquaredDistance;
      for (int i = 0; i < cloud->size(); i++) {
         std::vector<int>pointIdx;
         std::vector<float> pointSqDist;
         pcl::PointXYZ searchPoint = cloud->points[i];
         if (isneigbour) {
            kdtree.nearestKSearch(searchPoint, k, pointIdx, pointSqDist);
         } else {
            kdtree.radiusSearch(searchPoint, radius, pointIdx, pointSqDist);
         }
         pointIndices.push_back(pointIdx);
         pointSquaredDistance.push_back(pointSqDist);
         pointIdx.clear();
         pointSqDist.clear();
      }
    }

   void ConvexConnectedVoxels::getConvexLabelCloudIndices(
       const std::vector<pcl::PointCloud<PointT>::Ptr> &cloud_clusters,
       pcl::PointCloud<PointT>::Ptr cloud,
       const std::vector<int> &labelMD,
       std::map<int, pcl::PointIndices> &all_indices)
   {
       int icounter = 0;
       for (int i = 0; i < cloud_clusters.size(); i++) {
          int _idx = labelMD.at(i); 
          pcl::PointIndices _ind;
          for (int j = 0; j < cloud_clusters[i]->size(); j++) {
             _ind.indices.push_back(icounter++);
          }
          std::map<int, pcl::PointIndices>::iterator
             iter = all_indices.find(_idx);
          if (iter == all_indices.end()) {
             all_indices.insert(
                std::map<int, pcl::PointIndices>::value_type(_idx, _ind));
          } else {
             pcl::PointIndices prev_ind = (*iter).second;
             prev_ind.indices.insert(prev_ind.indices.end(),
                                     _ind.indices.begin(),
                                     _ind.indices.end());
             (*iter).second = prev_ind;
          }
       }
    }
}  // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_pcl_ros::ConvexConnectedVoxels, nodelet::Nodelet);
