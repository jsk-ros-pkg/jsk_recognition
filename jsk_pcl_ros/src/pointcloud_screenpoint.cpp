#include <ros/ros.h>
#include <nodelet/loader.h>

#include "jsk_pcl_ros/pointcloud_screenpoint.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pointcloud_screenpoint");

  // Shared parameters to be propagated to nodelet private namespaces
  nodelet::Loader manager(true); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  manager.load(ros::this_node::getName(), "jsk_pcl_ros/PointcloudScreenpoint", remappings, my_argv);

  ros::spin();
  return 0;
}
