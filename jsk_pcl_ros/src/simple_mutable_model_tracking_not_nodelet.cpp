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
//-----------------------ROS-----------------------------//
#include <ros/time.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/String.h>
#include <jsk_pcl_ros/SetWorkingStateAndTopic.h>

//-----------------------PCL-----------------------------//
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>

#include <pcl/io/pcd_io.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/search/pcl_search.h>
#include <pcl/common/transforms.h>

#include <boost/format.hpp>

#include <pcl/filters/filter.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/filter.h>

#include <jsk_pcl_ros/SetPointCloud2.h>
#include <jsk_pcl_ros/SetTrackerObjectId.h>


class SimpleMutableModelTracking
{
  const int  MIN_POINTS;
public:
  SimpleMutableModelTracking():MIN_POINTS(3),n_(new ros::NodeHandle("~")),working_(false)
  {
    ROS_INFO("Init Simple MutableModel");
    n_->param("resolution", resolution_, 0.02);
    n_->param("octree_resolution", octree_resolution_, 0.07);
    n_->param("pass_offset", pass_offset_, 2.0);
    n_->param("octree_pass_offset", octree_pass_offset_, 2.0);
    n_->param("tracker_id", tracker_id_, 1);
    n_->param("working", working_, false);


    cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    track_model_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    octree_result_cloud_.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);

    new_track_model_cloud = new_input_cloud = new_octree_result_cloud =  false;
    //Subscribe tracking model result and octree change result
    sub_track_model_ = n_->subscribe("input_track_model", 1, &SimpleMutableModelTracking::trackModelCb,this);
    sub_octree_result_ = n_->subscribe("input_octree_result", 1, &SimpleMutableModelTracking::octreeResultCb,this);
    sub_cloud_ = n_->subscribe("input_cloud", 1, &SimpleMutableModelTracking::cloudCb,this);

    switch_srv_ = n_->advertiseService("reset_state", &SimpleMutableModelTracking::recieve_signal, this);
  }

  //tracking model callback
  void trackModelCb(const sensor_msgs::PointCloud2 &pc)
  {
    //    ROS_INFO("track_model_callback");
    {
      boost::mutex::scoped_lock lock (mtx_);
      convertROStoPCL(pc, track_model_cloud_);
    }
    new_track_model_cloud = true;
  }

  //raw cloud callback
  void cloudCb(const sensor_msgs::PointCloud2 &pc)
  {
    if(pc.fields.size() <= 0){
      return;
    }


    //    ROS_INFO("inpu_cloud_callback %s", __func__);
    frame_id_ = pc.header.frame_id;
    {
      boost::mutex::scoped_lock lock (mtx_);
      convertROStoPCL(pc, cloud_);
    }
    new_input_cloud = true;
  }

  //main process callback
  void octreeResultCb(const sensor_msgs::PointCloud2 &pc)
  {
    if(working_){
      convertROStoPCL(pc, octree_result_cloud_);
      new_octree_result_cloud = true;
      {
        boost::mutex::scoped_lock lock (mtx_);
        if (new_octree_result_cloud && new_track_model_cloud && new_input_cloud &&
            cloud_->points.size() > MIN_POINTS && track_model_cloud_->points.size() > MIN_POINTS){

          new_octree_result_cloud = new_track_model_cloud = new_input_cloud = false;
          //regenerete new updated model
          pcl::PointCloud<pcl::PointXYZRGBA>::Ptr renew_model_cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
          updateModel(renew_model_cloud);

          //        ROS_INFO("Before %ld -> After %ld", track_model_cloud_->points.size(),renew_model_cloud->points.size());

          //if no new point cloud for octree, return. //Or twice as large as original abort
          if(renew_model_cloud->points.size() < MIN_POINTS)// || renew_model_cloud->points.size() /track_model_cloud_->points.size() > 2)
            return;

          //Service call to request new model
          ros::ServiceClient client = n_->serviceClient<jsk_pcl_ros::SetTrackerObjectId>("renew_model");
          jsk_pcl_ros::SetTrackerObjectId srv;
          sensor_msgs::PointCloud2 renew_model_pointcloud2;
          pcl::toROSMsg(*renew_model_cloud, renew_model_pointcloud2);
          renew_model_pointcloud2.header.frame_id = frame_id_;
          srv.request.cloud = renew_model_pointcloud2;
          srv.request.tracker_id = tracker_id_;
          srv.request.new_object_id = "";
          ROS_INFO("call new model from SIMPLE_MUTALVE");
          if (client.call(srv)){
            ROS_INFO("Renew Tracking Model Success");
          }
          else
            ROS_ERROR("Failed to Renew Tracking Model service");
        }
      }
    }
  }

  void convertROStoPCL(const sensor_msgs::PointCloud2 &pc, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& cloud)
  {
    cloud.reset(new pcl::PointCloud<pcl::PointXYZRGBA>);
    std::vector<int> indices;
    pcl::fromROSMsg(pc, *cloud);
    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
  }

  void updateModel(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& renew_model_cloud)
  {
    ROS_INFO("update model");
    //restrict target area
    pcl::PointXYZRGBA max_pt,min_pt;
    pcl::getMinMax3D(*track_model_cloud_, min_pt, max_pt);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr xyzpass_cloud_ (new pcl::PointCloud<pcl::PointXYZRGBA> ());
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr octree_result_cloud_pass(new pcl::PointCloud<pcl::PointXYZRGBA>());
    filterPassThroughXYZ (cloud_, *xyzpass_cloud_,  min_pt, max_pt, pass_offset_);
    filterPassThroughXYZ (octree_result_cloud_, *octree_result_cloud_pass,  min_pt, max_pt, octree_pass_offset_);

    //Search the nearect candidate clouds for removind old points
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> search_octree (resolution_);
    search_octree.setInputCloud (xyzpass_cloud_);
    search_octree.addPointsFromInputCloud ();

    //    ROS_INFO("remove old points");
    //remove old target point cloud
    for (int i = 0;i < track_model_cloud_->points.size(); i++){
      if(search_octree.isVoxelOccupiedAtPoint(track_model_cloud_->points[i]))
        renew_model_cloud->points.push_back(track_model_cloud_->points[i]);
    }


    ROS_INFO("model : octree_cloud  %ld -> pass %ld", octree_result_cloud_->points.size() , octree_result_cloud_pass->points.size() );
    ROS_INFO("model : xyz_cloud     %ld -> pass %ld", cloud_->points.size() , xyzpass_cloud_->points.size() );


    //if no new point cloud for octree, return.
    if(octree_result_cloud_pass->points.size() < MIN_POINTS){
      ROS_INFO("too small octree_result %ld", octree_result_cloud_pass->points.size());
      return;
    }


    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr restricted_xyzpass_cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>());
    filterPassThroughXYZ (xyzpass_cloud_, *restricted_xyzpass_cloud_,  min_pt, max_pt, -pass_offset_);

    //Search the nearect candidate clouds for new adding points
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> search_octree2 (octree_resolution_);
    *octree_result_cloud_pass = *octree_result_cloud_pass +  *restricted_xyzpass_cloud_;
    search_octree2.setInputCloud (octree_result_cloud_pass);
    search_octree2.addPointsFromInputCloud ();


    int *new_target_flags = new int[octree_result_cloud_pass->points.size()](); 

    std::vector<int> pointIdxVec;
    std::vector<int>::iterator it = pointIdxVec.begin();

    //    ROS_INFO("add new points");
    //make new target clouds
    for (int i = 0;i < track_model_cloud_->points.size(); i++){
      if(search_octree2.voxelSearch (track_model_cloud_->points[i], pointIdxVec))
        for(it=pointIdxVec.begin(); it != pointIdxVec.end();++it)
          new_target_flags[*it] = 1;
    }

    int tmp_counter = 0;
    for (int i = 0; i <  octree_result_cloud_pass->points.size();i++){
      if(new_target_flags[i]){
        renew_model_cloud->points.push_back(octree_result_cloud_pass->points[i]);
        tmp_counter++;
      }
    }
    ROS_INFO("model : renew_model : %ld diff %d", renew_model_cloud->points.size(), tmp_counter);
    delete[] new_target_flags;
  }

  bool recieve_signal(jsk_pcl_ros::SetWorkingStateAndTopic::Request &req,
                      jsk_pcl_ros::SetWorkingStateAndTopic::Response &response
                      )
  {
    if(req.state.data){
      ROS_INFO("RECEIVE_SIGNAL AT SIMPLE TRUE ");
    }else{
      ROS_INFO("RECEIVE_SIGNAL AT SIMPLE FALSE");
    }
    working_ = req.state.data;
    if(working_){
      sub_track_model_ = n_->subscribe(req.topic_name, 1, &SimpleMutableModelTracking::trackModelCb,this);
      sub_octree_result_ = n_->subscribe("input_octree_result", 1, &SimpleMutableModelTracking::octreeResultCb,this);
      sub_cloud_ = n_->subscribe("input_cloud", 1, &SimpleMutableModelTracking::cloudCb,this);

      ROS_ERROR("Start WORKING at SIMPLE_MODEL");
      ROS_ERROR("Start SUBSCRIBE %s at SIMPLE_MODEL", req.topic_name.c_str());
    }
    else{
      sub_track_model_.shutdown();
      sub_octree_result_.shutdown();
      sub_cloud_.shutdown();
      ROS_INFO("Stop WORKING at SIMPLE_MODEL");
    }
    return true;
  };


  void filterPassThroughXYZ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZRGBA> &result, pcl::PointXYZRGBA &min_pt,pcl::PointXYZRGBA &max_pt, double offset)
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
    ptfilter.setFilterLimits (min_pt.z - offset , max_pt.z + offset + 1000);
    ptfilter.filter (*indices_xz);

    // The indices_xz array indexes all points of cloud_in that have x between 0.0 and 1000.0 and z larger than 10.0 or smaller than -10.0
    ptfilter.setIndices (indices_xz);
    ptfilter.setFilterFieldName ("y");
    ptfilter.setFilterLimits (min_pt.y - offset , max_pt.y + offset);
    ptfilter.filter (result);
  }

protected:
  ros::NodeHandle* n_;
  ros::Subscriber sub_track_model_;
  ros::Subscriber sub_octree_result_;
  ros::Subscriber sub_cloud_;
  ros::ServiceServer switch_srv_;
  boost::mutex mtx_;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr track_model_cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr octree_result_cloud_;

  double resolution_;
  double octree_resolution_;
  double pass_offset_;
  double octree_pass_offset_;

  bool new_track_model_cloud;
  bool new_input_cloud;
  bool new_octree_result_cloud;

  bool working_;

  std::string frame_id_;

  int tracker_id_;
};

int main(int argc, char* argv[]){
  //ROS Initialize
  ros::init(argc, argv, "optical");
  std::string model_name;


  SimpleMutableModelTracking tracking;
  ros::spin();
}
