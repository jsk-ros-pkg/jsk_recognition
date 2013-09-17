// -*- mode: C++ -*-
#include <ros/ros.h>
#include <ros/names.h>

#include <std_msgs/ColorRGBA.h>

#include <dynamic_reconfigure/server.h>
#include "pcl_ros/FilterConfig.h"

#include <pcl_ros/pcl_nodelet.h>

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include "jsk_pcl_ros/ClusterPointIndices.h"
#include "jsk_pcl_ros/EuclideanSegment.h"

#include "jsk_pcl_ros/EuclideanClusteringConfig.h"

using namespace std;
using namespace pcl;

namespace pcl_ros
{
  class EuclideanClustering : public PCLNodelet
  {
  public:
      
    EuclideanClustering()
    {
        // initialize colors_
        colors_.push_back(makeColor(1.0, 0.0, 0.0, 1.0));
        colors_.push_back(makeColor(0.0, 1.0, 0.0, 1.0));
        colors_.push_back(makeColor(0.0, 0.0, 1.0, 1.0));
        colors_.push_back(makeColor(1.0, 1.0, 0.0, 1.0));
        colors_.push_back(makeColor(1.0, 0.0, 1.0, 1.0));
        colors_.push_back(makeColor(0.0, 1.0, 1.0, 1.0));
    };
    ~EuclideanClustering()
    {};

  protected:
    typedef jsk_pcl_ros::EuclideanClusteringConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;
   
    void config_callback (Config &config, uint32_t level);
      
  private:
    ros::Publisher result_pub_;
    ros::Subscriber sub_input_;
    ros::Publisher pcl_pub_;

    ros::ServiceServer service_;

    double tolerance;
    int minsize_;
    int maxsize_;
    EuclideanClusterExtraction<pcl::PointXYZ> impl_;

    // the list of COGs of each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZ> > cogs_;

    // the colors to be used to colorize
    std::vector<std_msgs::ColorRGBA> colors_;
      
    virtual void extract(const sensor_msgs::PointCloud2ConstPtr &input)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZRGB>);

      pcl::fromROSMsg(*input, *cloud);
      pcl::fromROSMsg(*input, *cloud_out);


#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
      tree->setInputCloud (cloud);

      vector<pcl::PointIndices> cluster_indices;
      impl_.setClusterTolerance (tolerance); // 2cm
      impl_.setMinClusterSize (minsize_);
      impl_.setMaxClusterSize (maxsize_);
      impl_.setSearchMethod (tree);
      impl_.setInputCloud (cloud);
      impl_.extract (cluster_indices);

      // Publish result indices
      jsk_pcl_ros::ClusterPointIndices result;
      result.cluster_indices.resize(cluster_indices.size());

      for (size_t i=0; i<cluster_indices.size(); i++)
      {
          result.cluster_indices[i].header = cluster_indices[i].header;
          result.cluster_indices[i].indices = cluster_indices[i].indices;
      }

      result_pub_.publish(result);

      // Publish point cloud after clustering
      size_t number_of_clusters = cluster_indices.size();
      
      for (size_t i = 0, color_index = 0; i<number_of_clusters; i++)
      {
          if (color_index == colors_.size())
              color_index = 0;
          
          uint8_t r, g, b;
          std_msgs::ColorRGBA c = colors_[color_index];
          r = (uint8_t)(c.r * 255);
          g = (uint8_t)(c.g * 255);
          b = (uint8_t)(c.b * 255);
          uint32_t rgb = ((uint32_t)r<<16 | (uint32_t)g<<8 | (uint32_t)b);
          
          for (size_t j=0; j<cluster_indices[i].indices.size(); j++)
          {
              cloud_out->points[ cluster_indices[i].indices[j] ].rgb = *reinterpret_cast<float*>(&rgb);
          }
          
          color_index++;
      }

      pcl_pub_.publish(*cloud_out);
    }

    bool serviceCallback(jsk_pcl_ros::EuclideanSegment::Request &req,
                         jsk_pcl_ros::EuclideanSegment::Response &res) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(req.input, *cloud);

      ROS_INFO("service input point cloud");

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
      pcl::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ>);
      tree = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif

      vector<pcl::PointIndices> cluster_indices;
      EuclideanClusterExtraction<pcl::PointXYZ> impl_;
      double tor;
      if ( req.tolerance < 0) {
        tor = tolerance;
      } else {
        tor = req.tolerance;
      }
      impl_.setClusterTolerance (tor);
      impl_.setMinClusterSize (minsize_);
      impl_.setMaxClusterSize (maxsize_);
      impl_.setSearchMethod (tree);
      impl_.setInputCloud (cloud);
      impl_.extract (cluster_indices);

      ROS_INFO("clusters: %lu", cluster_indices.size());

      res.output.resize( cluster_indices.size() );
      pcl::ExtractIndices<sensor_msgs::PointCloud2> ex;
      ex.setInputCloud ( boost::make_shared< sensor_msgs::PointCloud2 > (req.input) );
      for ( size_t i = 0; i < cluster_indices.size(); i++ ) {
        //ex.setInputCloud ( boost::make_shared< sensor_msgs::PointCloud2 > (req.input) );
        ex.setIndices ( boost::make_shared< pcl::PointIndices > (cluster_indices[i]) );
        ex.setNegative ( false );
        ex.filter ( res.output[i] );
      }

      return true;
    }

    static std_msgs::ColorRGBA makeColor(double r, double g, double b, double a)
    {
        std_msgs::ColorRGBA c;
        c.r = r;
        c.g = g;
        c.b = b;
        c.a = a;
        return c;
    }
      
    virtual void onInit()
    {
      // boost::shared_ptr<ros::NodeHandle> pnh_;
      // pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));

      PCLNodelet::onInit();

      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
          boost::bind (&EuclideanClustering::config_callback, this, _1, _2);
      srv_->setCallback (f);
      
      pnh_->param("tolerance", tolerance, 0.02);
      ROS_INFO("tolerance : %f", tolerance);

      pnh_->param("max_size", maxsize_, 25000);
      ROS_INFO("max cluster size : %d", maxsize_);

      pnh_->param("min_size", minsize_, 20);
      ROS_INFO("min cluster size : %d", minsize_);

      result_pub_ = pnh_->advertise<jsk_pcl_ros::ClusterPointIndices> ("output", 1);
      pcl_pub_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("points_output",1);
      sub_input_ = pnh_->subscribe("input", 1, &EuclideanClustering::extract, this);

      service_ = pnh_->advertiseService(pnh_->resolveName("euclidean_clustering"),
                                        &EuclideanClustering::serviceCallback, this);
    }
  };
    
}
