#pragma once

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>

namespace jsk_pcl_ros
{
  class KeypointsPublisher: public pcl_ros::PCLNodelet
  {
  public:
    KeypointsPublisher();
    virtual ~KeypointsPublisher();
    virtual void onInit();
  protected:
    virtual void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    virtual void extractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    
    ros::Subscriber sub_input_;
    ros::Publisher keypoints_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
    std_msgs::Header input_header_;
  };
}    
