#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/transforms.h>

#include <pluginlib/class_list_macros.h>
#include <dynamic_reconfigure/server.h>

#include <pcl/range_image/range_image_planar.h>
#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
#include <pcl/common/transforms.h>
#else
#include <pcl/common/transform.h>
#endif

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>

#include <opencv2/opencv.hpp>

#include <std_srvs/Empty.h>
#include <boost/thread/mutex.hpp>

namespace jsk_pcl_ros
{
  class DepthImageCreator : public pcl_ros::PCLNodelet
  {
  private:
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    ros::Subscriber sub_as_info_;
    ros::Subscriber sub_as_cloud_;
    ros::Publisher pub_image_;
    ros::Publisher pub_cloud_;
    ros::Publisher pub_disp_image_;
    ros::ServiceServer service_;

    sensor_msgs::PointCloud2ConstPtr points_ptr_;

    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > sync_inputs_e_;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> > > sync_inputs_a_;
    boost::mutex mutex_points;
    bool use_fixed_transform;
    bool use_service;
    bool use_asynchronous;
    bool use_approximate;
    int info_throttle_;
    int info_counter_;

    tf::StampedTransform fixed_transform;
    double scale_depth;
    typedef pcl::PointXYZ Point;
    typedef pcl::PointCloud< Point > PointCloud;

    void onInit();

    bool service_cb (std_srvs::Empty::Request &req,
                     std_srvs::Empty::Response &res);

    void callback_sync(const sensor_msgs::CameraInfoConstPtr& info,
                       const sensor_msgs::PointCloud2ConstPtr& pcloud2);

    void callback_cloud(const sensor_msgs::PointCloud2ConstPtr& pcloud2);

    void callback_info(const sensor_msgs::CameraInfoConstPtr& info);

    void publish_points(const sensor_msgs::CameraInfoConstPtr& info,
                        const sensor_msgs::PointCloud2ConstPtr& pcloud2);
  public:
    DepthImageCreator() {};
    ~DepthImageCreator();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

