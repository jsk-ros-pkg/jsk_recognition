#include <pcl_conversions/pcl_conversions.h>

#include "jsk_pcl_ros/color_based_region_growing_segmentation.h"

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_recognition_msgs/ClusterPointIndices.h"


namespace jsk_pcl_ros
{

  void ColorBasedRegionGrowingSegmentation::onInit()
  {
    ConnectionBasedNodelet::onInit();
    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    dynamic_reconfigure::Server<Config>::CallbackType f=
      boost::bind (&ColorBasedRegionGrowingSegmentation::configCallback, this, _1, _2);
    srv_->setCallback (f);
    pub_ = advertise<jsk_recognition_msgs::ClusterPointIndices>(*pnh_, "output", 1);
    onInitPostProcess();
  }

  void ColorBasedRegionGrowingSegmentation::subscribe()
  {
    sub_ = pnh_->subscribe("input", 1, &ColorBasedRegionGrowingSegmentation::segment, this);
  }

  void ColorBasedRegionGrowingSegmentation::unsubscribe()
  {
    sub_.shutdown();
  }

  void ColorBasedRegionGrowingSegmentation::configCallback(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock(mutex_);

    if (distance_threshould_ != config.distance_threshould) {
      distance_threshould_ = config.distance_threshould;
    }
    if (point_color_threshould_ != config.point_color_threshould) {
      point_color_threshould_ = config.point_color_threshould;
    }
    if (region_color_threshould_ != config.region_color_threshould) {
      region_color_threshould_ = config.region_color_threshould;
    }
    if (min_cluster_size_ != config.min_cluster_size) {
      min_cluster_size_ = config.min_cluster_size;
    }
  }

  void ColorBasedRegionGrowingSegmentation::segment(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGB> > (new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    pcl::IndicesPtr indices (new std::vector <int>);
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, 1.0);
    pass.filter (*indices);

    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud (cloud);
    reg.setIndices (indices);
    reg.setSearchMethod (tree);
    reg.setDistanceThreshold (distance_threshould_);
    reg.setPointColorThreshold (point_color_threshould_);
    reg.setRegionColorThreshold (region_color_threshould_);
    reg.setMinClusterSize (min_cluster_size_);

    std::vector <pcl::PointIndices> clusters;
    reg.extract (clusters);

    jsk_recognition_msgs::ClusterPointIndices output;
    output.header = msg->header;

    for (size_t i = 0; i < clusters.size(); i++) {
      PCLIndicesMsg indices;
      indices.header = msg->header;
      indices.indices = clusters[i].indices;
      output.cluster_indices.push_back(indices);
    }
    pub_.publish(output);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ColorBasedRegionGrowingSegmentation, nodelet::Nodelet);
