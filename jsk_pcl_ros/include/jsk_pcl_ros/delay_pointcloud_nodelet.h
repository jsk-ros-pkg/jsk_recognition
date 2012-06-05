// -*- mode: c++ -*-

#include <pcl_ros/pcl_nodelet.h>
#include <sensor_msgs/PointCloud2.h>
#include <queue>

namespace pcl_ros
{
    class DelayPointCloud : public nodelet::Nodelet
    {
    public:
        typedef sensor_msgs::PointCloud2 PointCloud;
        typedef PointCloud::Ptr PointCloudPtr;
        typedef PointCloud::ConstPtr PointCloudConstPtr;

        DelayPointCloud (): delay_index_(1), max_queue_size_(2) {};
        ~DelayPointCloud () {} ;
        virtual void onInit();
        virtual void input_callback(const PointCloudConstPtr& msg);
    protected:
        int delay_index_;
        std::queue<PointCloudConstPtr> queue_;
        int max_queue_size_;
        ros::Subscriber sub_;
        ros::Publisher pub_;
    };
}
