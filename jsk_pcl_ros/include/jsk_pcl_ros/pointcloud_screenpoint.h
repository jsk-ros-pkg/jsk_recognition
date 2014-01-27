#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/point_cloud.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>

#include "jsk_pcl_ros/TransformScreenpoint.h"

#include <boost/thread/mutex.hpp>
// F/K/A <ray ocnverter>
// Haseru Chen, Kei Okada, Yohei Kakiuchi

namespace jsk_pcl_ros
{
  class PointcloudScreenpoint : public nodelet::Nodelet
  {
    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2,
                                                             geometry_msgs::PolygonStamped > PolygonApproxSyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2,
                                                             geometry_msgs::PointStamped > PointApproxSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime< sensor_msgs::PointCloud2,
                                                             sensor_msgs::PointCloud2 > PointCloudApproxSyncPolicy;

  private:
    boost::shared_ptr<ros::NodeHandle> pnh_;

    message_filters::Subscriber < sensor_msgs::PointCloud2 > points_sub_;
    message_filters::Subscriber < geometry_msgs::PolygonStamped > rect_sub_;
    message_filters::Subscriber < geometry_msgs::PointStamped > point_sub_;
    message_filters::Subscriber < sensor_msgs::PointCloud2 > point_array_sub_;

    boost::shared_ptr < message_filters::Synchronizer < PolygonApproxSyncPolicy > > sync_a_polygon_;
    boost::shared_ptr < message_filters::Synchronizer < PointApproxSyncPolicy > > sync_a_point_;
    boost::shared_ptr < message_filters::Synchronizer < PointCloudApproxSyncPolicy > > sync_a_point_array_;

    ros::Publisher pub_points_;
    ros::Publisher pub_point_;

    ros::ServiceServer srv_;
    pcl::PointCloud<pcl::PointXYZ> pts;
    std_msgs::Header header_;

    pcl::NormalEstimation< pcl::PointXYZ, pcl::Normal > n3d_;
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
    pcl::search::KdTree< pcl::PointXYZ >::Ptr normals_tree_;
#else
    pcl::KdTree< pcl::PointXYZ >::Ptr normals_tree_;
#endif

    void onInit();
    bool screenpoint_cb(jsk_pcl_ros::TransformScreenpoint::Request &req,
                        jsk_pcl_ros::TransformScreenpoint::Response &res);
    void points_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

    bool checkpoint (pcl::PointCloud< pcl::PointXYZ > &in_pts, int x, int y,
                     float &resx, float &resy, float &resz);
    bool extract_point (pcl::PointCloud< pcl::PointXYZ > &in_pts, int reqx, int reqy,
                        float &resx, float &resy, float &resz);
    void extract_rect (const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                       int st_x, int st_y, int ed_x, int ed_y);
    void point_cb (const geometry_msgs::PointStampedConstPtr& pt_ptr);
    void callback_point (const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                         const geometry_msgs::PointStampedConstPtr& pt_ptr);
    void point_array_cb (const sensor_msgs::PointCloud2ConstPtr& pt_arr_ptr);
    void callback_point_array (const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                               const sensor_msgs::PointCloud2ConstPtr& pt_arr_ptr);
    
    void rect_cb (const geometry_msgs::PolygonStampedConstPtr& array_ptr);
    void callback_polygon(const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                          const geometry_msgs::PolygonStampedConstPtr& array_ptr);
    boost::mutex mutex_callback_;

    int k_;
    int queue_size_;
    int crop_size_;
    bool publish_point_;
    bool publish_points_;

  public:
    PointcloudScreenpoint () {};
    ~PointcloudScreenpoint () {};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

