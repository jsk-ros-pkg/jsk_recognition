#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/color_converter_nodelet.h"

namespace pcl_ros
{
    void
    RGB2HSVColorConverter::computePublish
    (const PointCloudInConstPtr &cloud,
     const PointCloudInConstPtr &surface,
     const IndicesPtr &indices)
    {
        impl_.setInputCloud (cloud);
        impl_.setIndices (indices);
        PointCloudOut output;
        impl_.compute (output);
        output.header = cloud->header;
        pub_output_.publish (output.makeShared ());
    }
    
    void HSV2RGBColorConverter::computePublish
    (const PointCloudInConstPtr &cloud,
     const PointCloudInConstPtr &surface,
     const IndicesPtr &indices)
    {
        impl_.setInputCloud (cloud);
        impl_.setIndices (indices);
        PointCloudOut output;
        impl_.compute (output);
        output.header = cloud->header;
        pub_output_.publish (output.makeShared ());
    }
    
}

typedef pcl_ros::RGB2HSVColorConverter RGB2HSVColorConverter;
typedef pcl_ros::HSV2RGBColorConverter HSV2RGBColorConverter;

PLUGINLIB_DECLARE_CLASS (jsk_pcl, RGB2HSVColorConverter,
                         RGB2HSVColorConverter, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (jsk_pcl, HSV2RGBColorConverter,
                         HSV2RGBColorConverter, nodelet::Nodelet);
