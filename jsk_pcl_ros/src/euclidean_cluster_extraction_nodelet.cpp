#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/euclidean_cluster_extraction_nodelet.h" 

namespace pcl_ros
{
    void EuclideanClustering::config_callback (Config &config, uint32_t level)
    {
        ROS_INFO("config_callback");
        boost::mutex::scoped_lock lock (mutex_);
        if (tolerance != config.tolerance) {
            tolerance = config.tolerance;
        }
        if (maxsize_ != config.max_size) {
            maxsize_ = config.max_size;
        }
        if (minsize_ != config.min_size) {
            minsize_ = config.min_size;
        }
    }
    
}

PLUGINLIB_DECLARE_CLASS(jsk_pcl, EuclideanClustering, pcl_ros::EuclideanClustering, nodelet::Nodelet);
