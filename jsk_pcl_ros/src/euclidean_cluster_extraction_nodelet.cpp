#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/euclidean_cluster_extraction_nodelet.h" 

PLUGINLIB_DECLARE_CLASS(jsk_pcl, EuclideanClustering, pcl_ros::EuclideanClustering, nodelet::Nodelet);
