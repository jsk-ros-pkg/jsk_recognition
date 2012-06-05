#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>
#include <pcl/filters/extract_indices.h>

#include <pcl_ros/pcl_nodelet.h>
#include <pcl_ros/filters/filter.h>
#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{
  class ResizePointsPublisher : public pcl_ros::Filter
  {
  private:
    int step_x_, step_y_;
    pcl::ExtractIndices<sensor_msgs::PointCloud2> extract_;

    void onInit () {
      NODELET_INFO("[%s::onInit]", getName().c_str());
      pcl_ros::Filter::onInit();

      pnh_->param("step_x", step_x_, 2);
      ROS_INFO("step_x : %d", step_x_);
      pnh_->param("step_y", step_y_, 2);
      ROS_INFO("step_y : %d", step_y_);
    }

    ~ResizePointsPublisher() { }

    void filter (const PointCloud2::ConstPtr &input,
                 const IndicesPtr &indices,
                 PointCloud2 &output) {
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
        for(unsigned int i = 0; i < indices->size(); i++) {
          flags[indices->at(i)] = 1;
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

      extract_.setInputCloud (input);
      extract_.setIndices (boost::make_shared <std::vector<int> > (ex_indices));
      extract_.setNegative (false);
      extract_.filter (output);

      output.header = input->header;
      output.width = (width - ox)/sx;
      if((width - ox)%sx) output.width += 1;
      output.height = (height - oy)/sy;
      if((height - oy)%sy) output.height += 1;
#if DEBUG
      ROS_INFO("%dx%d (%d %d)(%d %d) -> %dx%d %d", width,height, ox, oy, sx, sy,
               output.width, output.height, ex_indices.size());
#endif
    }

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

typedef jsk_pcl_ros::ResizePointsPublisher ResizePointsPublisher;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, ResizePointsPublisher, ResizePointsPublisher, nodelet::Nodelet);
