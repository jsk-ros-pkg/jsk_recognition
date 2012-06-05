#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/delay_pointcloud_nodelet.h"

namespace pcl_ros
{
    void DelayPointCloud::onInit()
    {
        boost::shared_ptr<ros::NodeHandle> pnh_;
        pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));

        // read parameters
        pnh_->getParam("delay_index", delay_index_);
        pnh_->getParam ("max_queue_size", max_queue_size_);
        
        pub_ = pnh_->advertise<PointCloud> ("output", max_queue_size_);
        sub_ = pnh_->subscribe ("input", max_queue_size_,
                                &DelayPointCloud::input_callback, this);
    }
    
    void DelayPointCloud::input_callback (const PointCloudConstPtr& msg)
    {
        if ((int)queue_.size() < delay_index_ )
        {
            queue_.push(msg);
            return;
        }
        else
        {
            const PointCloudConstPtr target = queue_.front();
            queue_.pop();
            queue_.push(msg);
            PointCloud output;

            // copy msg = latest
            output.header.stamp = msg->header.stamp;
            
            // copy target = previous
            output.header.frame_id = target->header.frame_id;
            output.header.seq = target->header.seq;
            output.height = target->height;
            output.width = target->width;
            output.fields = target->fields;
            output.is_bigendian = target->is_bigendian;
            output.point_step = target->point_step;
            output.row_step = target->row_step;
            output.data = target->data;
            output.is_dense = target->is_dense;
            pub_.publish(output);
        }
    }
}

typedef pcl_ros::DelayPointCloud DelayPointCloud;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, DelayPointCloud,
                         DelayPointCloud, nodelet::Nodelet);
