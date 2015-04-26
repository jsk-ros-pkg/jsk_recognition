#include "pcl/pcl_base.h"
#include "jsk_pcl_ros/point_types.h"
#include "jsk_pcl_ros/color_converter.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

void callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointHSV> cloud;
    pcl::fromROSMsg(*msg, cloud);
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        pcl::PointHSV point = cloud.points[i];
        std::cout << "h: " << point.hue << "s: " << point.saturation
                  << "v: " << point.value << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hoge");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("input", 1, callback);
    ros::spin();
    return 0;
}
