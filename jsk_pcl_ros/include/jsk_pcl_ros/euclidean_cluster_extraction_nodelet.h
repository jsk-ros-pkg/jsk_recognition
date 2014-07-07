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

#ifndef JSK_PCL_ROS_EUCLIDEAN_CLUSTER_EXTRACTION_NODELET_H_
#define JSK_PCL_ROS_EUCLIDEAN_CLUSTER_EXTRACTION_NODELET_H_

#include <ros/ros.h>
#include <ros/names.h>

#include <std_msgs/ColorRGBA.h>

#include <dynamic_reconfigure/server.h>

#include <pcl_ros/pcl_nodelet.h>

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "jsk_pcl_ros/EuclideanSegment.h"
#include "jsk_pcl_ros/Int32Stamped.h"

#include "jsk_pcl_ros/EuclideanClusteringConfig.h"

#include<Eigen/StdVector>


using namespace std;
using namespace pcl;

namespace pcl_ros
{
  class EuclideanClustering : public PCLNodelet
  {
  public:
      
    EuclideanClustering()
    {
        // initialize colors_
        colors_.push_back(makeColor(1.0, 0.0, 0.0, 1.0));
        colors_.push_back(makeColor(0.0, 1.0, 0.0, 1.0));
        colors_.push_back(makeColor(0.0, 0.0, 1.0, 1.0));
        colors_.push_back(makeColor(1.0, 1.0, 0.0, 1.0));
        colors_.push_back(makeColor(1.0, 0.0, 1.0, 1.0));
        colors_.push_back(makeColor(0.0, 1.0, 1.0, 1.0));
        colors_.push_back(makeColor(1.0, 1.0, 1.0, 1.0));
    };
    virtual ~EuclideanClustering()
    {};

  protected:
    typedef jsk_pcl_ros::EuclideanClusteringConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
   
    void config_callback (Config &config, uint32_t level);
      
  private:
    ros::Publisher result_pub_;
    ros::Subscriber sub_input_;
    ros::Publisher pcl_pub_;
    ros::Publisher cluster_num_pub_;

    ros::ServiceServer service_;

    double tolerance;
    double label_tracking_tolerance;
    int minsize_;
    int maxsize_;
    EuclideanClusterExtraction<pcl::PointXYZ> impl_;

    // the list of COGs of each cluster
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > cogs_;

    // the colors to be used to colorize
    std::vector<std_msgs::ColorRGBA> colors_;

    static std::vector<pcl::PointIndices> pivotClusterIndices(std::vector<int>& pivot_table, std::vector<pcl::PointIndices>& cluster_indices)
    {
        std::vector<pcl::PointIndices> new_cluster_indices;
        new_cluster_indices.resize(pivot_table.size());
        for (size_t i = 0; i < pivot_table.size(); i++)
        {
            new_cluster_indices[i] = cluster_indices[pivot_table[i]];
        }
        return new_cluster_indices;
    }
      
   static std::vector<int> buildLabelTrackingPivotTable(double* D,
                                                        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > cogs,
                                                        std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > new_cogs,
                                                        double label_tracking_tolerance)
    {
          std::vector<int> pivot_table;
          // initialize pivot table
          pivot_table.resize(cogs.size());
          for (size_t i = 0; i < pivot_table.size(); i++)
              pivot_table[i] = i;
          for (size_t pivot_counter = 0; pivot_counter < pivot_table.size();
               pivot_counter++)
          {
              double minimum_distance = DBL_MAX;
              size_t minimum_previous_index = 0;
              size_t minimum_next_index = 0;
              for (size_t i = 0; i < cogs.size(); i++)
              {
                  for (size_t j = 0; j < new_cogs.size(); j++)
                  {
                      double distance = D[i * cogs.size() + j];
                      //ROS_INFO("distance %lux%lu: %f", i, j, distance);
                      if (distance < minimum_distance)
                      {
                          minimum_distance = distance;
                          minimum_previous_index = i;
                          minimum_next_index = j;
                      }
                  }
              }
              if (minimum_distance > label_tracking_tolerance)
              {
                  // ROS_WARN("minimum tracking distance exceeds tolerance: %f > %f",
                  //          minimum_distance, label_tracking_tolerance);
                  std::vector<int> dummy;
                  return dummy;
              }
              pivot_table[minimum_previous_index] = minimum_next_index;
              // fill the D matrix with DBL_MAX
              for (size_t j = 0; j < new_cogs.size(); j++)
              {
                  D[minimum_previous_index * cogs.size() + j] = DBL_MAX;
              }
          }
          return pivot_table;
      }

    static void computeDistanceMatrix(double* D,
                                      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& old_cogs,
                                      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& new_cogs)
    {
        for (size_t i = 0; i < old_cogs.size(); i++)
        {
            Eigen::Vector4f previous_cog = old_cogs[i];
            for (size_t j = 0; j < new_cogs.size(); j++)
            {
                Eigen::Vector4f next_cog = new_cogs[j];
                double distance = (next_cog - previous_cog).norm();
                // ROS_INFO("row distance (%f, %f, %f) -- (%f, %f, %f)",
                //          next_cog[0], next_cog[1], next_cog[2],
                //         previous_cog[0], previous_cog[1], previous_cog[2]);
                    
                //ROS_INFO("raw distance %lux%lu: %f", i, j, distance);
                //D[i][j] = distance;
                D[i * old_cogs.size() + j] = distance;
            }
        }
    }
      
    static void
    computeCentroidsOfClusters(std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> >& ret,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               std::vector<pcl::PointIndices> cluster_indices)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        ret.resize(cluster_indices.size());
        for (size_t i = 0; i < cluster_indices.size(); i++)
        {
            // build pointcloud
            pcl::PointCloud<pcl::PointXYZ>::Ptr segmented_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointIndices::Ptr segmented_indices (new pcl::PointIndices);
            for (size_t j = 0; j < cluster_indices[i].indices.size(); j++)
            {
                segmented_indices->indices.push_back(cluster_indices[i].indices[j]);
            }
            extract.setIndices(segmented_indices);
            extract.filter(*segmented_cloud);
            Eigen::Vector4f center;
            pcl::compute3DCentroid(*segmented_cloud, center);
            ret[i] = center;
        }
    }
      
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &input)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);
      
      pcl::fromROSMsg(*input, *cloud);
      pcl::fromROSMsg(*input, *cloud_out);

      if (cloud->points.size() == 0) {
          ROS_WARN("empty input");
      }
      
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
      tree->setInputCloud (cloud);

      std::vector<pcl::PointIndices> cluster_indices;
      impl_.setClusterTolerance (tolerance); // 2cm
      impl_.setMinClusterSize (minsize_);
      impl_.setMaxClusterSize (maxsize_);
      impl_.setSearchMethod (tree);
      impl_.setInputCloud (cloud);
      impl_.extract (cluster_indices);
      if (cluster_indices.size() == 0) {
          ROS_WARN("empty cluster");
      }
      // Publish result indices
      jsk_pcl_ros::ClusterPointIndices result;
      result.cluster_indices.resize(cluster_indices.size());
      result.header = input->header;
      if (cogs_.size() != 0 && cogs_.size() == cluster_indices.size())    
      {// tracking the labels
          //ROS_INFO("computing distance matrix");
          // compute distance matrix
          // D[i][j] --> distance between the i-th previous cluster
          //             and the current j-th cluster
          std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > new_cogs;
          computeCentroidsOfClusters(new_cogs, cloud, cluster_indices);
          double D[cogs_.size() * new_cogs.size()];
          computeDistanceMatrix(D, cogs_, new_cogs);
          std::vector<int> pivot_table = buildLabelTrackingPivotTable(D, cogs_, new_cogs, label_tracking_tolerance);
          // print pivot table
          // ROS_INFO("pivoting");
          // for (size_t i = 0; i < pivot_table.size(); i++)
          // {
          //     ROS_INFO("%lu -> %d", i, pivot_table[i]);
          // }
          if (pivot_table.size() != 0)
          {
              cluster_indices = pivotClusterIndices(pivot_table, cluster_indices);
          }
      }
      else
      {
        // if (cogs_.size() == 0)
        // {
        //   ROS_WARN("reset tracking for initialization");
        // }
        // else if (cogs_.size() != cluster_indices.size())
        // {
        //   ROS_WARN("reset tracking, cluster size changed: %lu -> %lu",
        //            cogs_.size(), cluster_indices.size());
        // }
      }
      std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > tmp_cogs;
      computeCentroidsOfClusters(tmp_cogs, cloud, cluster_indices); // NB: not efficient
      cogs_ = tmp_cogs;
      
      for (size_t i = 0; i < cluster_indices.size(); i++)
      {
#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
          result.cluster_indices[i].header
            = pcl_conversions::fromPCL(cluster_indices[i].header);
#else
// groovy
          result.cluster_indices[i].header = cluster_indices[i].header;
#endif

          result.cluster_indices[i].indices = cluster_indices[i].indices;
      }

      result_pub_.publish(result);

      // Publish point cloud after clustering
      size_t number_of_clusters = cluster_indices.size();
      
      for (size_t i = 0, color_index = 0; i<number_of_clusters; i++)
      {
          if (color_index == colors_.size())
              color_index = 0;
          
          uint32_t rgb = colorRGBAToUInt32(colors_[color_index]);
          
          for (size_t j=0; j<cluster_indices[i].indices.size(); j++)
          {
              cloud_out->points[ cluster_indices[i].indices[j] ].rgb = *reinterpret_cast<float*>(&rgb);
          }
          
          color_index++;
      }

      pcl_pub_.publish(*cloud_out);

      jsk_pcl_ros::Int32Stamped::Ptr cluster_num_msg (new jsk_pcl_ros::Int32Stamped);
      cluster_num_msg->header = input->header;
      cluster_num_msg->data = number_of_clusters;
      cluster_num_pub_.publish(cluster_num_msg);
      
    }

    bool serviceCallback(jsk_pcl_ros::EuclideanSegment::Request &req,
                         jsk_pcl_ros::EuclideanSegment::Response &res) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(req.input, *cloud);

      ROS_INFO("service input point cloud");

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif

      vector<pcl::PointIndices> cluster_indices;
      EuclideanClusterExtraction<pcl::PointXYZ> impl_;
      double tor;
      if ( req.tolerance < 0) {
        tor = tolerance;
      } else {
        tor = req.tolerance;
      }
      impl_.setClusterTolerance (tor);
      impl_.setMinClusterSize (minsize_);
      impl_.setMaxClusterSize (maxsize_);
      impl_.setSearchMethod (tree);
      impl_.setInputCloud (cloud);
      impl_.extract (cluster_indices);

      ROS_INFO("clusters: %lu", cluster_indices.size());

      res.output.resize( cluster_indices.size() );
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 7 )
      pcl::PCLPointCloud2::Ptr pcl_cloud(new pcl::PCLPointCloud2);
      pcl_conversions::toPCL(req.input, *pcl_cloud);
      pcl::ExtractIndices<pcl::PCLPointCloud2> ex;
      ex.setInputCloud(pcl_cloud);
#else
      pcl::ExtractIndices<sensor_msgs::PointCloud2> ex;
      ex.setInputCloud ( boost::make_shared< sensor_msgs::PointCloud2 > (req.input) );
#endif
      for ( size_t i = 0; i < cluster_indices.size(); i++ ) {
        ex.setIndices ( boost::make_shared< pcl::PointIndices > (cluster_indices[i]) );
        ex.setNegative ( false );
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 7 )
        pcl::PCLPointCloud2 output_cloud;
        ex.filter ( output_cloud );
        pcl_conversions::fromPCL(output_cloud, res.output[i]);
#else
        ex.filter ( res.output[i] );
#endif
      }

      return true;
    }

    static std_msgs::ColorRGBA makeColor(double r, double g, double b, double a)
    {
        std_msgs::ColorRGBA c;
        c.r = r;
        c.g = g;
        c.b = b;
        c.a = a;
        return c;
    }

    static uint32_t colorRGBAToUInt32(std_msgs::ColorRGBA c)
    {
        uint8_t r, g, b;
        r = (uint8_t)(c.r * 255);
        g = (uint8_t)(c.g * 255);
        b = (uint8_t)(c.b * 255);
        return ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);
    }

    virtual void onInit()
    {
      // boost::shared_ptr<ros::NodeHandle> pnh_;
      // pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));

      PCLNodelet::onInit();
      
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
          boost::bind (&EuclideanClustering::config_callback, this, _1, _2);
      srv_->setCallback (f);
      
      pnh_->param("tolerance", tolerance, 0.02);
      ROS_INFO("tolerance : %f", tolerance);
      pnh_->param("label_tracking_tolerance", label_tracking_tolerance, 0.02);
      ROS_INFO("label_tracking_tolerance : %f", tolerance);

      pnh_->param("max_size", maxsize_, 25000);
      ROS_INFO("max cluster size : %d", maxsize_);

      pnh_->param("min_size", minsize_, 20);
      ROS_INFO("min cluster size : %d", minsize_);

      result_pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices> ("output", 1);
      pcl_pub_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("points_output",1);
      cluster_num_pub_ = pnh_->advertise<jsk_pcl_ros::Int32Stamped> ("cluster_num", 1);
      sub_input_ = pnh_->subscribe("input", 1, &EuclideanClustering::extract, this);
      service_ = pnh_->advertiseService(pnh_->resolveName("euclidean_clustering"),
                                        &EuclideanClustering::serviceCallback, this);
    }
  };
    
}

#endif
