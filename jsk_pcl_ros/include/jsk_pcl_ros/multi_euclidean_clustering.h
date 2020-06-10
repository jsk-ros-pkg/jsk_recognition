#ifndef _JSK_PCL_ROS_MULTI_EUCLIDEAN_CLUSTERING_H_
#define _JSK_PCL_ROS_MULTI_EUCLIDEAN_CLUSTERING_H_

#include <ros/ros.h>

#include <dynamic_reconfigure/server.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PointIndices.h>

#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <jsk_topic_tools/diagnostic_nodelet.h>

#include "jsk_pcl_ros/MultiEuclideanClusteringConfig.h"

namespace jsk_pcl_ros 
{
  class MultiEuclideanClustering : public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef jsk_pcl_ros::MultiEuclideanClusteringConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::ClusterPointIndices,
      sensor_msgs::PointCloud2
      > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      jsk_recognition_msgs::ClusterPointIndices,
      sensor_msgs::PointCloud2
      > ApproximateSyncPolicy;
    MultiEuclideanClustering() : DiagnosticNodelet("MultiEuclideanClustering") {}
  protected:
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;

    virtual void onInit();

    void configCallback (Config &config, uint32_t level);
    virtual void updateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void subscribe();
    virtual void unsubscribe();

    virtual void callback(const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cluster_indices,
                          const sensor_msgs::PointCloud2::ConstPtr& point_cloud);

    ros::Publisher pub_cluster_indices_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessed_cloud_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;

    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_cluster_indices_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_point_cloud_;

    double tolerance_;
    int min_size_;
    int max_size_;
    bool downsample_;
    double leaf_size_;
    bool approximate_sync_;
    int queue_size_;
    std::vector<std::vector<int> > downsample_to_original_indices_;
    std::vector<int> original_to_downsample_indices_;

  private:
  };
} // namespace jsk_pcl_ros 

#endif
