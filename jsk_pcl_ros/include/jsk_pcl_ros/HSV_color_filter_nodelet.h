// -*- Mode: C++ -*-

#include <ros/ros.h>

#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/publisher.h>

#include <cv.h>

#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/PointIndices.h>

#include <vector>

#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/ColorFilterConfig.h"
#include "jsk_pcl_ros/HSVColorFilterConfig.h"

using namespace std;

namespace pcl_ros
{
  class HSVColorFilter2 : public PCLNodelet
  {
  public:
    HSVColorFilter2()
      {};
    ~HSVColorFilter2()
      {};

  protected:
    typedef jsk_pcl_ros::HSVColorFilterConfig Config;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;

  private:
    ros::Subscriber pcl_input_;
    ros::Publisher pcl_output_;
    ros::Publisher indices_output_;

    int h_max_;
    int h_min_;
    int s_max_;
    int s_min_;
    int v_max_;
    int v_min_;

    bool filter_limit_negative_;
    bool use_hue_;

    virtual void main_cb(const sensor_msgs::PointCloud2ConstPtr &input)
    {
      pcl::PointCloud<pcl::PointXYZRGB> cloud;
      pcl::PointCloud<pcl::PointXYZRGB> cloud_out;
      pcl::fromROSMsg(*input, cloud);

      IplImage *rgb_mat = cvCreateImage(cvSize(cloud.points.size(),1), IPL_DEPTH_8U, 3);
      IplImage *hsv_mat = cvCreateImage(cvSize(cloud.points.size(),1), IPL_DEPTH_8U, 3);

      for (unsigned int i=0; i<cloud.points.size(); i++)
        {
          uint32_t rgb = *reinterpret_cast<int*> (&cloud.points[i].rgb);
          unsigned char r = (rgb>>16) & 0x0000ff;
          unsigned char g = (rgb>>8)  & 0x0000ff;
          unsigned char b = (rgb)     & 0x0000ff;

          rgb_mat->imageData[i*3 + 0] = (unsigned char) r;
          rgb_mat->imageData[i*3 + 1] = (unsigned char) g;
          rgb_mat->imageData[i*3 + 2] = (unsigned char) b;
        }

      cvCvtColor(rgb_mat, hsv_mat, CV_RGB2HSV);

      // ROS_INFO("color filtering");

      std::vector<int> color_indices;

      for (unsigned int i=0; i<cloud.points.size(); i++)
        {
          if ( ( (unsigned char) (hsv_mat->imageData[i*3 + 0]) <= h_max_) &&
               ( (unsigned char) (hsv_mat->imageData[i*3 + 0]) >= h_min_) &&
               ( (unsigned char) (hsv_mat->imageData[i*3 + 1]) <= s_max_) &&
               ( (unsigned char) (hsv_mat->imageData[i*3 + 1]) >= s_min_) &&
               ( (unsigned char) (hsv_mat->imageData[i*3 + 2]) <= v_max_) &&
               ( (unsigned char) (hsv_mat->imageData[i*3 + 2]) >= v_min_) )
            color_indices.push_back(i);
        }

      pcl::PointIndices indices_out;
      indices_out.indices.resize(color_indices.size());
      indices_out.indices = color_indices;

      indices_out.header = input->header;

      pcl::copyPointCloud(cloud, color_indices, cloud_out);

      pcl_output_.publish(cloud_out);
      indices_output_.publish(indices_out);
    }

    void config_callback (Config &config, uint32_t level) {
      if ( filter_limit_negative_ != config.filter_limit_negative)
        filter_limit_negative_ = config.filter_limit_negative;

      if (h_max_ != config.h_limit_max) h_max_ = (int) ( 180 * config.h_limit_max );

      if (s_max_ != config.s_limit_max) s_max_ = (int)(255*config.s_limit_max);

      if (v_max_ != config.v_limit_max) v_max_ = (int)(255*config.v_limit_max);

      if (h_min_ != config.h_limit_min) h_min_ = (int) ( 180 * config.h_limit_min );

      if (s_min_ != config.s_limit_min) s_min_ = (int)(255*config.s_limit_min);

      if (v_min_ != config.v_limit_min) v_min_ = (int)(255*config.v_limit_min);

      if (use_hue_ != config.use_h) use_hue_ = config.use_h;
    }

    virtual void onInit()
    {
      PCLNodelet::onInit();

      double p;
      pnh_->param<double>("h_limit_max", p, 1.0);
      h_max_ = (int) (180 * p);
      pnh_->param<double>("h_limit_min", p, 0);
      h_min_ = (int) (180 * p);

      pnh_->param<double>("s_limit_max", p, 1.0);
      s_max_ = (int) (255 * p);
      pnh_->param<double>("s_limit_min", p, 0);
      s_min_ = (int) (255 * p);

      pnh_->param<double>("v_limit_max", p, 1.0);
      v_max_ = (int) (255 * p);
      pnh_->param<double>("v_limit_min", p, 0);
      v_min_ = (int) (255 * p);

      pnh_->param<bool>("filter_limit_negative", filter_limit_negative_, false);
      pnh_->param<bool>("use_h", use_hue_, true);

      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (&HSVColorFilter2::config_callback, this, _1, _2);
      srv_->setCallback (f);

      pcl_output_ = pnh_->advertise<pcl::PointCloud<pcl::PointXYZRGB> > ("output",1);
      indices_output_ = pnh_->advertise<pcl::PointIndices> ("indices_output",1);
      pcl_input_ = pnh_->subscribe("input", 1, &HSVColorFilter2::main_cb, this);
    }
  };
}
