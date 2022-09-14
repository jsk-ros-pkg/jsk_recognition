#pragma once

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_cloud.h>
#include <jsk_recognition_utils/jsk_topic_tools_version.h>
#if JSK_TOPIC_TOOLS_VERSION_MINIMUM(2,2,13)
  #include <jsk_topic_tools/diagnostic_nodelet.h>
  namespace jsk_topic_tools {
    #define NODELET DiagnosticNodelet
  }
#else
  #include <jsk_topic_tools/connection_based_nodelet.h>
  namespace jsk_topic_tools {
    #define NODELET ConnectionBasedNodelet
  }
#endif

namespace jsk_pcl_ros
{
  class KeypointsPublisher: public jsk_topic_tools::NODELET
  {
  public:
    KeypointsPublisher(){}
    virtual void onInit();
  protected:
    virtual void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& input);
    virtual void extractKeypoints(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    virtual void subscribe();
    virtual void unsubscribe();
    ros::Subscriber sub_input_;
    ros::Publisher keypoints_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_;
    std_msgs::Header input_header_;
  };
}    
