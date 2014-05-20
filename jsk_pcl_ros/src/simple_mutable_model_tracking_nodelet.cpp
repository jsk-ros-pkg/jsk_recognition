/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki and JSK Lab
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
#include "jsk_pcl_ros/simple_mutable_model_tracking.h"
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  void SimpleMutableModelTracking::onInit(void)
  {
    PCLNodelet::onInit();
    pnh_->param("resolution_", resolution_, 0.02);
    pnh_->param("pass_offset_", pass_offset_, 2.0);

    MIN_POINTS = 10;

    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    track_model_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    octree_result_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    new_track_model_cloud = new_input_cloud = new_octree_result_cloud =  false;
    //Subscribe tracking model result and octree change result
    sub_track_model_ = pnh_->subscribe("input_track_model", 1, &SimpleMutableModelTracking::trackModelCb,this);
    sub_octree_result_ = pnh_->subscribe("input_octree_result", 1, &SimpleMutableModelTracking::octreeResultCb,this);
    sub_cloud_ = pnh_->subscribe("input_cloud", 1, &SimpleMutableModelTracking::cloudCb,this);
  }

  //tracking model callback
  void SimpleMutableModelTracking::trackModelCb(const sensor_msgs::PointCloud2 &pc)
  {
    //    ROS_INFO("track_model_callback");
    {
      ROS_INFO("track_model mtx_ before %d", __LINE__);
      boost::mutex::scoped_lock lock (mtx_);
      ROS_INFO(" track_model mtx_ after %d", __LINE__);
      convertROStoPCL(pc, track_model_cloud_);
    }
    new_track_model_cloud = true;
  }

  //raw cloud callback
  void SimpleMutableModelTracking::cloudCb(const sensor_msgs::PointCloud2 &pc)
  {
    //    ROS_INFO("inpu_cloud_callback %s", __func__);
    frame_id_ = pc.header.frame_id;
    {
      ROS_INFO("input mtx_ before %d", __LINE__);
      boost::mutex::scoped_lock lock (mtx_);
      ROS_INFO("input mtx_ after %d", __LINE__);
      convertROStoPCL(pc, cloud_);
    }
    new_input_cloud = true;
  }

  //main process callback
  void SimpleMutableModelTracking::octreeResultCb(const sensor_msgs::PointCloud2 &pc)
  {
    //    ROS_INFO("octree_result_callback %s", __func__);
    convertROStoPCL(pc, octree_result_cloud_);
    new_octree_result_cloud = true;
    {
      ROS_INFO("octree mtx_ before %d", __LINE__);
      boost::mutex::scoped_lock lock (mtx_);
      ROS_INFO(" octree mtx_ after %d", __LINE__);
      if (new_octree_result_cloud && new_track_model_cloud && new_input_cloud &&
          cloud_->points.size() > MIN_POINTS && track_model_cloud_->points.size() > MIN_POINTS){
        ROS_INFO("Update Model Message");
        new_octree_result_cloud = new_track_model_cloud = new_input_cloud = false;
        //regenerete new updated model
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr renew_model_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        updateModel(renew_model_cloud);
        //Service call to request new model
        ros::ServiceClient client = pnh_->serviceClient<jsk_pcl_ros::SetPointCloud2>("/pcl_nodelet/particle_filter_tracker/renew_model");
        jsk_pcl_ros::SetPointCloud2 srv;
        sensor_msgs::PointCloud2 renew_model_pointcloud2;
        pcl::toROSMsg(*renew_model_cloud, renew_model_pointcloud2);
        renew_model_pointcloud2.header.frame_id = frame_id_;
        srv.request.cloud = renew_model_pointcloud2;
        ROS_INFO("octree_result_callback %d", __LINE__);
        if (client.call(srv))
          ROS_INFO("Renew Tracking Model Success");
        else
          ROS_ERROR("Failed to Renew Tracking Model service");
      }
    }
  }

  void SimpleMutableModelTracking::convertROStoPCL(const sensor_msgs::PointCloud2 &pc, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::vector<int> indices;
    pcl::fromROSMsg(pc, *cloud);
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  }

  void SimpleMutableModelTracking::updateModel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& renew_model_cloud)
  {
    ROS_INFO("update model");
    //restrict target area
    pcl::PointXYZRGBA max_pt,min_pt;
    pcl::getMinMax3D(*track_model_cloud_, min_pt, max_pt);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzoctree_result_cloud_ (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr octree_result_cloud_pass(new pcl::PointCloud<pcl::PointXYZRGBA>());
    filterPassThroughXYZ (cloud_, *xyzoctree_result_cloud_,  min_pt, max_pt, pass_offset_);
    filterPassThroughXYZ (octree_result_cloud_, *octree_result_cloud_pass,  min_pt, max_pt, pass_offset_);

    //Search the nearect candidate clouds for removind old points
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> search_octree (resolution_);
    search_octree.setInputCloud (xyzoctree_result_cloud_);
    search_octree.addPointsFromInputCloud ();

    ROS_INFO("model : track_model_cloud_.points.size() : %ld", track_model_cloud_->points.size() );
    ROS_INFO("remove old points");
    //remove old target point cloud
    for (int i = 0;i < track_model_cloud_->points.size(); i++){
      if(search_octree.isVoxelOccupiedAtPoint(track_model_cloud_->points[i]))
        renew_model_cloud->points.push_back(track_model_cloud_->points[i]);
    }

    //Search the nearect candidate clouds for new adding points
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> search_octree2 (resolution_);

    search_octree2.setInputCloud (octree_result_cloud_pass);
    search_octree2.addPointsFromInputCloud ();

    int *new_target_flags = new int[octree_result_cloud_pass->points.size()](); 

    std::vector<int> pointIdxVec;
    std::vector<int>::iterator it = pointIdxVec.begin();

    ROS_INFO("add new points");
    //make new target clouds
    for (int i = 0;i < track_model_cloud_->points.size(); i++){
      if(search_octree2.voxelSearch (track_model_cloud_->points[i], pointIdxVec))
        for(it=pointIdxVec.begin(); it != pointIdxVec.end();++it)
          new_target_flags[*it] = 1;
    }

    ROS_INFO("END - add new points");
    for (int i = 0; i <  octree_result_cloud_pass->points.size();i++)
      if(new_target_flags[i])
        renew_model_cloud->points.push_back(octree_result_cloud_pass->points[i]);

    delete[] new_target_flags;
  }

  void SimpleMutableModelTracking::filterPassThroughXYZ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result, pcl::PointXYZRGBA &min_pt,pcl::PointXYZRGBA &max_pt, double offset)
  {
    pcl::IndicesPtr indices_x(new std::vector<int>());
    pcl::PassThrough<pcl::PointXYZRGBA> ptfilter (true); // Initializing with true will allow us to extract the removed indices
    ptfilter.setInputCloud (cloud);
    ptfilter.setFilterFieldName ("x");
    ptfilter.setFilterLimits (min_pt.x - offset , max_pt.x + offset);
    ptfilter.filter (*indices_x);

    pcl::IndicesPtr indices_xz(new std::vector<int>());
    ptfilter.setIndices (indices_x);
    ptfilter.setFilterFieldName ("z");
    ptfilter.setFilterLimits (min_pt.z - offset , max_pt.z + offset);
    ptfilter.filter (*indices_xz);

    // The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
    ptfilter.setIndices (indices_xz);
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (min_pt.y - offset , max_pt.y + offset);
    ptfilter.filter (result);
  }


}

typedef jsk_pcl_ros::SimpleMutableModelTracking SimpleMutableModelTracking;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, SimpleMutableModelTracking, SimpleMutableModelTracking, nodelet::Nodelet);
