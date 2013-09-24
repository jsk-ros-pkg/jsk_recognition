/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

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
