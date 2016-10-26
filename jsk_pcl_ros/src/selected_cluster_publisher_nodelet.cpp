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

#include "jsk_pcl_ros/selected_cluster_publisher.h"
#include <pluginlib/class_list_macros.h>
#include <pcl/filters/extract_indices.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros
{
  void SelectedClusterPublisher::onInit()
  {
    ConnectionBasedNodelet::onInit();
    pnh_->param("keep_organized", keep_organized_, false);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void SelectedClusterPublisher::subscribe()
  {
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(300); // 100 is enough?
    sub_input_.subscribe(*pnh_, "input", 1);
    sub_indices_.subscribe(*pnh_, "indices", 1);
    sub_index_.subscribe(*pnh_, "selected_index", 1);
    sync_->connectInput(sub_input_, sub_indices_, sub_index_);
    sync_->registerCallback(boost::bind(&SelectedClusterPublisher::extract, this, _1, _2, _3));
  }

  void SelectedClusterPublisher::unsubscribe()
  {
    sub_input_.unsubscribe();
    sub_indices_.unsubscribe();
    sub_index_.unsubscribe();
  }

  void SelectedClusterPublisher::extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                                         const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices,
                                         const jsk_recognition_msgs::Int32Stamped::ConstPtr& index)
  {
    if (indices->cluster_indices.size() <= index->data) {
      NODELET_ERROR("the selected index %d is out of clusters array %lu",
                    index->data,
                    indices->cluster_indices.size());
      return;
    }
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::fromROSMsg(*input, *input_cloud);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::PointIndices::Ptr pcl_indices (new pcl::PointIndices);
    pcl_indices->indices = indices->cluster_indices[index->data].indices;
    extract.setInputCloud(input_cloud);
    extract.setIndices(pcl_indices);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr extracted_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    if(keep_organized_){
      extract.setKeepOrganized(true);
    }
    extract.filter(*extracted_cloud);
    sensor_msgs::PointCloud2 ros_msg;
    pcl::toROSMsg(*extracted_cloud, ros_msg);
    ros_msg.header = input->header;
    pub_.publish(ros_msg);
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::SelectedClusterPublisher,
                        nodelet::Nodelet);
