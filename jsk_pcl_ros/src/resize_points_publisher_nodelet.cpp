#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/pcl_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/connection_based_nodelet.h>

#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/ResizePointsPublisherConfig.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  class ResizePointsPublisher : public jsk_topic_tools::ConnectionBasedNodelet
  {
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                      PCLIndicesMsg> SyncPolicy;
    typedef jsk_pcl_ros::ResizePointsPublisherConfig Config;

  private:
    int step_x_, step_y_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<PCLIndicesMsg> sub_indices_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> >  srv_;
    ros::Subscriber sub_;
    ros::Subscriber resizedmask_sub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher pub_;
    bool not_use_rgb_;
    boost::mutex mutex_;
    bool use_indices_;
    void onInit () {
      ConnectionBasedNodelet::onInit();
      pnh_->param("use_indices", use_indices_, false);
      pnh_->param("not_use_rgb", not_use_rgb_, false);
      srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
      dynamic_reconfigure::Server<Config>::CallbackType f =
        boost::bind (
                     &ResizePointsPublisher::configCallback, this, _1, _2);
      srv_->setCallback (f);
      pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);
      resizedmask_sub_ = pnh_->subscribe("input/mask", 1, &ResizePointsPublisher::resizedmaskCallback, this);
      onInitPostProcess();
    }

    void configCallback(Config &config, uint32_t level) {
      boost::mutex::scoped_lock lock(mutex_);
      step_x_ = config.step_x;
      step_y_ = config.step_y;
    }

    void resizedmaskCallback (const sensor_msgs::Image::ConstPtr& msg) {
      boost::mutex::scoped_lock lock(mutex_);
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy
        (msg, sensor_msgs::image_encodings::MONO8);
      cv::Mat mask = cv_ptr->image;
      int maskwidth = mask.cols;
      int maskheight = mask.rows;
      int cnt = 0;
      for (size_t j = 0; j < maskheight; j++){
        for (size_t i = 0; i < maskwidth; i++){
          if (mask.at<uchar>(j, i) != 0){
            cnt++;
          }
        }
      }
      int surface_per = ((double) cnt) / (maskwidth * maskheight) * 100;
      // step_x_ = surface_per /10;
      step_x_ = sqrt(surface_per);
      if (step_x_ < 1) {
        step_x_ = 1;
      }
      step_y_ = step_x_;
    }

    void subscribe()
    {
      
      if (use_indices_) {
        sub_input_.subscribe(*pnh_, "input", 1);
        sub_indices_.subscribe(*pnh_, "indices", 1);
        sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
        sync_->connectInput(sub_input_, sub_indices_);
        if (!not_use_rgb_) {
          sync_->registerCallback(boost::bind(&ResizePointsPublisher::filter<pcl::PointXYZRGB>, this, _1, _2));
        }
        else {
          sync_->registerCallback(boost::bind(&ResizePointsPublisher::filter<pcl::PointXYZ>, this, _1, _2));
        }
      }
      else {
        if (!not_use_rgb_) {
          sub_ = pnh_->subscribe(
            "input", 1,
            &ResizePointsPublisher::filter<pcl::PointXYZRGB>, this);
        }
        else {
          sub_ = pnh_->subscribe(
            "input", 1,
            &ResizePointsPublisher::filter<pcl::PointXYZ>, this);
        }
      }
    }

    void unsubscribe()
    {
      if (use_indices_) {
        sub_input_.unsubscribe();
        sub_indices_.unsubscribe();
      }
      else {
        sub_.shutdown();
      }
    }
    
    ~ResizePointsPublisher() { }

    template<class T> void filter (const sensor_msgs::PointCloud2::ConstPtr &input) {
      filter<T>(input, PCLIndicesMsg::ConstPtr());
    }
    
    template<class T> void filter (const sensor_msgs::PointCloud2::ConstPtr &input,
                                   const PCLIndicesMsg::ConstPtr &indices) {
      pcl::PointCloud<T> pcl_input_cloud, output;
      fromROSMsg(*input, pcl_input_cloud);
      boost::mutex::scoped_lock lock (mutex_);
      std::vector<int> ex_indices;
      ex_indices.resize(0);

      int width = input->width;
      int height = input->height;
      int ox, oy, sx, sy;

      sx = step_x_;
      ox = sx/2;
      if(height == 1) {
        sy = 1;
        oy = 0;
      } else {
        sy = step_y_;
        oy = sy/2;
      }

      if (indices) {
        std::vector<int> flags;
        flags.resize(width*height);

        //std::vector<int>::iterator it;
        //for(it = indices->begin(); it != indices->end(); it++)
        //flags[*it] = 1;
        for(unsigned int i = 0; i < indices->indices.size(); i++) {
          flags[indices->indices.at(i)] = 1;
        }
        for(int y = oy; y < height; y += sy) {
          for(int x = ox; x < width; x += sx) {
            if (flags[y*width + x] == 1) {
              ex_indices.push_back(y*width + x); // just use points in indices
            }
          }
        }
      } else {
        for(int y = oy; y < height; y += sy) {
          for(int x = ox; x < width; x += sx) {
            ex_indices.push_back(y*width + x);
          }
        }
      }
      pcl::ExtractIndices<T> extract;
      extract.setInputCloud (pcl_input_cloud.makeShared());
      extract.setIndices (boost::make_shared <std::vector<int> > (ex_indices));
      extract.setNegative (false);
      extract.filter (output);

      if (output.points.size() > 0) {
        sensor_msgs::PointCloud2 ros_out;
        toROSMsg(output, ros_out);
        ros_out.header = input->header;
        ros_out.width = (width - ox)/sx;
        if((width - ox)%sx) ros_out.width += 1;
        ros_out.height = (height - oy)/sy;
        if((height - oy)%sy) ros_out.height += 1;
        ros_out.row_step = ros_out.point_step * ros_out.width;
        ros_out.is_dense = input->is_dense;
#if DEBUG
        NODELET_INFO("%dx%d (%d %d)(%d %d) -> %dx%d %d", width,height, ox, oy, sx, sy,
                 ros_out.width, ros_out.height, ex_indices.size());
#endif
        pub_.publish(ros_out);
        NODELET_DEBUG("%s:: input header stamp is [%f]", getName().c_str(),
                          input->header.stamp.toSec());
        NODELET_DEBUG("%s:: output header stamp is [%f]", getName().c_str(),
                          ros_out.header.stamp.toSec());
      }
      
    }

  };
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::ResizePointsPublisher, nodelet::Nodelet);
