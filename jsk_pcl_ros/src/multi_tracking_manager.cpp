#include "ros/ros.h"
#include "pcl/pcl_base.h"
#include "pcl/point_types.h"
#include "jsk_pcl_ros/SetPointCloud2.h"
#include <sensor_msgs/PointCloud2.h>
#include <jsk_rviz_plugins/PointCloud2WithId.h>
#include <boost/smart_ptr/make_shared.hpp>
#include <boost/format.hpp>
#include <boost/lexical_cast.hpp>
#include <iostream>
#include <vector>
#include <string>

class ParticleFilterTrackerManager{
public:
  ParticleFilterTrackerManager():INIT_ID(1),target_id_(INIT_ID),n_("~"){
    ROS_INFO("Initializing multi tracker");

    //Get Node Param
    //    n_.param("input_cloud", input_cloud_name_, std::string("/selected_pointcloud"));
    n_.param("tracker_num", tracker_num_, 2);
    n_.param("tracker_name", tracker_name_, std::string("/pcl_nodelet/particle_filter_tracker"));
    n_.param("tracker_service_name", tracker_service_name_, std::string("/renew_model"));

    //Set subscriber
    sub_ = n_.subscribe("input_cloud", 1, &ParticleFilterTrackerManager::cloud_cb, this);
  }

  void send_service(int target_id, const sensor_msgs::PointCloud2& cloud){
    ros::ServiceClient client = n_.serviceClient<jsk_pcl_ros::SetPointCloud2>(tracker_name_+boost::lexical_cast<std::string>(target_id)+tracker_service_name_);
    jsk_pcl_ros::SetPointCloud2 srv;
    srv.request.cloud = cloud;
    if (client.call(srv))
      {
        ROS_INFO("Send Success");
      }
    else
      {
        ROS_ERROR("Failed to call service");
      }
  }

  void cloud_cb(const jsk_rviz_plugins::PointCloud2WithId& cloud_msg){
    ROS_INFO("cloud_cb id : %d tracker_num : %d tracker_name : %s tracker_sercice_name : %s", cloud_msg.id, tracker_num_, tracker_name_.c_str(), tracker_service_name_.c_str());
    if(cloud_msg.id != -1){
      target_id_ = cloud_msg.id;
    }
    if(target_id_ > tracker_num_)
      target_id_ = INIT_ID;

    send_service(target_id_,cloud_msg.cloud);
    target_id_++;
  }

  void run(){
    ros::spin();
  }

  ros::NodeHandle n_;
  ros::Subscriber sub_;

  int tracker_num_;
  std::string tracker_name_;
  std::string tracker_service_name_;
  std::string input_cloud_name_;
  int target_id_;

  const int INIT_ID;
};


int main (int argc, char** argv) {
  ros::init(argc, argv, "send_renew_model");

  ParticleFilterTrackerManager pm;
  pm.run();

  return 0;
}
