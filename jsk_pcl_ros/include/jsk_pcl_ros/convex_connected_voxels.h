
#ifndef _CONVEX_CONNECTED_VOXELS_H_
#define _CONVEX_CONNECTED_VOXELS_H_

#include <jsk_pcl_ros/region_adjacency_graph.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_pcl_ros/pcl_conversion_util.h>

#include <ros/ros.h>
#include <ros/console.h>

// ROS sensor message header directives
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// OpenCV header directives
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL header directives
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/centroid.h>
#include <pcl/common/impl/common.hpp>
#include <pcl/registration/distances.h>

// JSK Nodelet
#include <jsk_topic_tools/diagnostic_nodelet.h>

namespace jsk_pcl_ros {
   class ConvexConnectedVoxels: public jsk_topic_tools::DiagnosticNodelet {

    public:
      ConvexConnectedVoxels() : DiagnosticNodelet("ConvexConnectedVoxels") {}
      typedef pcl::PointXYZRGB PointT;

    protected:
      void cloud_cb(
         const sensor_msgs::PointCloud2::ConstPtr &);
      void indices_cb(
         const jsk_recognition_msgs::ClusterPointIndices &);

      void segmentCloud(
         const pcl::PointCloud<PointT>::Ptr,
         const std::vector<pcl::PointIndices> &,
         std::vector<pcl::PointCloud<PointT>::Ptr> &,
         std::vector<pcl::PointCloud<pcl::Normal>::Ptr> &,
         pcl::PointCloud<pcl::PointXYZ>::Ptr);
      void estimatePointCloudNormals(
         const pcl::PointCloud<PointT>::Ptr,
         pcl::PointCloud<pcl::Normal>::Ptr,
         const int = 8, const double = 0.02,
         bool = true);
      void nearestNeigborSearch(
         pcl::PointCloud<pcl::PointXYZ>::Ptr,
         std::vector<std::vector<int> > &,
         const int = 8,
         const double = 0.02,
         bool = true);
      void getConvexLabelCloudIndices(
         const std::vector<pcl::PointCloud<PointT>::Ptr> &,
         pcl::PointCloud<PointT>::Ptr,
         const std::vector<int> &,
         std::map<int, pcl::PointIndices> &);
       
       boost::mutex mutex_;
       ros::Subscriber sub_cloud_;
       ros::Subscriber sub_indices_;
       ros::Publisher pub_indices_;
       ros::NodeHandle nh_;

       virtual void onInit();
       virtual void subscribe();
       virtual void unsubscribe();
       virtual void updateDiagnostic(
          diagnostic_updater::DiagnosticStatusWrapper &);
       
    private:
      std::vector<pcl::PointIndices> indices_;
   };
}  // namespace jsk_pcl_ros

#endif  // _CONVEX_CONNECTED_VOXELS_H_
