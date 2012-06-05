#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pluginlib/class_list_macros.h>

#include "jsk_pcl_ros/TransformScreenpoint.h"

#include <boost/thread/mutex.hpp>
// F/K/A <ray ocnverter>
// Haseru Chen, Kei Okada, Yohei Kakiuchi

namespace jsk_pcl_ros
{
  class PointcloudScreenpoint : public pcl_ros::PCLNodelet
  {
  private:
    ros::Subscriber sub_;
    ros::ServiceServer srv_;
    pcl::PointCloud<pcl::PointXYZ> pts;
    std_msgs::Header header_;

    pcl::NormalEstimation< pcl::PointXYZ, pcl::Normal > n3d_;
    pcl::KdTree< pcl::PointXYZ >::Ptr normals_tree_;

    void onInit();
    bool screenpoint_cb(jsk_pcl_ros::TransformScreenpoint::Request &req,
			jsk_pcl_ros::TransformScreenpoint::Response &res);
    void points_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

    boost::mutex mutex_callback_;

  public:
    PointcloudScreenpoint () {};
    ~PointcloudScreenpoint () {};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

