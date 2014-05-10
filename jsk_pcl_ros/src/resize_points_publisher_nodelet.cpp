#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/pcl_nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>


#if ROS_VERSION_MINIMUM(1, 10, 0)
// hydro and later
typedef pcl_msgs::PointIndices PCLIndicesMsg;
#else
// groovy
typedef pcl::PointIndices PCLIndicesMsg;
#endif


namespace jsk_pcl_ros
{
  class ResizePointsPublisher : public pcl_ros::PCLNodelet
  {
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                      PCLIndicesMsg> SyncPolicy;

  private:
    int step_x_, step_y_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<PCLIndicesMsg> sub_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher pub_;

    pcl::ExtractIndices<pcl::PointXYZRGB> extract_;

    boost::mutex mutex_;
    
    void onInit () {
      NODELET_INFO("[%s::onInit]", getName().c_str());
      pcl_ros::PCLNodelet::onInit();

      pnh_->param("step_x", step_x_, 2);
      ROS_INFO("step_x : %d", step_x_);
      pnh_->param("step_y", step_y_, 2);
      ROS_INFO("step_y : %d", step_y_);

      pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("output", 1);
      sub_input_.subscribe(*pnh_, "input", 1);
      if (use_indices_) {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
      sub_indices_.subscribe(*pnh_, "indices", 1);
      sync_->connectInput(sub_input_, sub_indices_);
      sync_->registerCallback(boost::bind(&ResizePointsPublisher::filter, this, _1, _2));
    }
    else {
      sub_input_.registerCallback(&ResizePointsPublisher::filter, this);
    }
    }

    ~ResizePointsPublisher() { }

    void filter (const sensor_msgs::PointCloud2::ConstPtr &input) {
      filter(input, PCLIndicesMsg::ConstPtr());
    }
    
    void filter (const sensor_msgs::PointCloud2::ConstPtr &input,
                 const PCLIndicesMsg::ConstPtr &indices) {
      pcl::PointCloud<pcl::PointXYZRGB> pcl_input_cloud, output;
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

      extract_.setInputCloud (pcl_input_cloud.makeShared());
      extract_.setIndices (boost::make_shared <std::vector<int> > (ex_indices));
      extract_.setNegative (false);
      extract_.filter (output);

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
        ROS_INFO("%dx%d (%d %d)(%d %d) -> %dx%d %d", width,height, ox, oy, sx, sy,
                 ros_out.width, ros_out.height, ex_indices.size());
#endif
        pub_.publish(ros_out);
      }
      
    }

  };
}

typedef jsk_pcl_ros::ResizePointsPublisher ResizePointsPublisher;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ResizePointsPublisher, ResizePointsPublisher, nodelet::Nodelet);
