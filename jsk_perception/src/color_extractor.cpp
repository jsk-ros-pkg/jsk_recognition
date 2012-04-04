// -*- mode: C++ -*-
#include <ros/ros.h>
#include <ros/names.h>

#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>

using namespace std;
using namespace pcl;

// utility
typedef union
{
  struct /*anonymous*/
  {
    unsigned char Blue; // Blue channel
    unsigned char Green; // Green channel
    unsigned char Red; // Red channel
  };
  float float_value;
  long long_value;
} RGBValue;

class ColorExtractor
{

protected:
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;
  ros::Publisher pub_output_;
  ros::Subscriber sub_input_;

  bool use_h_;
  double h_min_;
  double h_max_;
  double s_min_;
  double s_max_;
  double v_min_;
  double v_max_;

public:
  //ColorExtractor() {};
  virtual ~ColorExtractor() {};

  ColorExtractor() : _nh(), _private_nh("~") {
    // RGB or HSV
    _private_nh.param("use_h", use_h_, true);
    ROS_INFO("use_h : %d", use_h_);

    _private_nh.param("h_min", h_min_, 0.0);
    ROS_INFO("h_min : %f", h_min_);
    _private_nh.param("h_max", h_max_, 360.0);
    ROS_INFO("h_max : %f", h_max_);

    _private_nh.param("v_min", v_min_, 0.0);
    ROS_INFO("v_min : %f", v_min_);
    _private_nh.param("v_max", v_max_, 1.0);
    ROS_INFO("v_max : %f", v_max_);

    _private_nh.param("s_min", s_min_, 0.0);
    ROS_INFO("s_min : %f", s_min_);
    _private_nh.param("s_max", s_max_, 1.0);
    ROS_INFO("s_max : %f", s_max_);

    pub_output_ = _private_nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

    sub_input_ = _private_nh.subscribe("input", 1, &ColorExtractor::extract, this);
  }

  virtual void extract(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);

    pcl::PointIndices inliers;
    inliers.indices.resize(0);
    { // extract color
      RGBValue rgbv;
      for (int i=0; i < cloud->points.size(); i++) {
        rgbv.float_value = cloud->points[i].rgb;
        int b = rgbv.Blue;
        int g = rgbv.Green;
        int r = rgbv.Red;
        int max = 0,min = 0;

        // rgb -> hvs
        if (r >= b) {
          if (r > g) {
            max = r;
          } else {
            max = g;
          }
        } else {
          if (b > g) {
            max = b;
          } else {
            max = g;
          }
        }
        if (r <= b) {
          if (r < g) {
            min = r;
          } else {
            min = g;
          }
        } else {
          if (b < g) {
            min = b;
          } else {
            min = g;
          }
        }

        float d = max - min;
        float v = max / 255.0;
        float s = (d == 0) ? 0 : ( (d * 255.0) / max );
        float h = 0.0;
        if (s != 0.0) {
          float rt = max - ((r * 60.0) / d);
          float gt = max - ((g * 60.0) / d);
          float bt = max - ((b * 60.0) / d);
          if(r == max) {
            h = bt - gt;
          } else if (g == max) {
            h = 120 + rt - bt;
          } else {
            h = 240 + gt - rt;
          }
          //ROS_INFO("max %d, min %d, %f %f %f", max, min, rt, gt, bt);
        }
        if (h < 0.0) h += 360.0;
        //h /= 360.0;
        s /= 255.0;

        //ROS_INFO("RGB->HSV #i(%d %d %d) -> #f(%f %f %f)", r, g, b, h, s, v);

        bool ret = true;
        if (use_h_) {
          if ( (h_min_ < h_max_) ) {
            if ( !(h_min_ <= h && h <= h_max_) )
              ret = false;
          } else { // h_min_ > h_max_
            if( !( h_max_ >= h || h >= h_min_) )
              ret = false;
          }
        }
        if (!(s_min_ <= s && s <= s_max_)) ret = false;
        if (!(v_min_ <= v && v <= v_max_)) ret = false;

        if (ret) {
          inliers.indices.push_back(i);
        }
      }
    }
    //
    ROS_INFO("extract size = %d -> %d", cloud->points.size(),inliers.indices.size());
    sensor_msgs::PointCloud2 cloud_out;
    pcl::ExtractIndices<sensor_msgs::PointCloud2> extract;
    extract.setInputCloud ( input );
    extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));

    extract.setNegative (false);
    extract.filter (cloud_out);

    pub_output_.publish(cloud_out);
  }
};

int main (int argc, char** argv)
{
  ros::init (argc, argv, "color_extractor");
  ColorExtractor p;
  ros::spin();
  return 0;
}
