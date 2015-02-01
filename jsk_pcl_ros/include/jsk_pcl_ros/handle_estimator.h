// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
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
 *   * Neither the name of the JSK Lab nor the names of its
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

#ifndef JSK_PCL_ROS_HANDLE_ESTIMATOR_H_
#define JSK_PCL_ROS_HANDLE_ESTIMATOR_H_

#include <pcl_ros/pcl_nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_pcl_ros/tf_listener_singleton.h"

#include <jsk_recognition_msgs/BoundingBox.h>
#include "jsk_recognition_msgs/Int32Stamped.h"
#include <geometry_msgs/PoseArray.h>

#include <boost/circular_buffer.hpp>
#include <boost/tuple/tuple.hpp>

#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros
{
  class HandleEstimator: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime< sensor_msgs::PointCloud2,
                                                       jsk_recognition_msgs::BoundingBox > SyncPolicy;
    enum HandleType
    {
      NO_HANDLE,
      HANDLE_SMALL_ENOUGH_STAND_ON_PLANE,
      HANDLE_SMALL_ENOUGH_LIE_ON_PLANE_Y_LONGEST,
      HANDLE_SMALL_ENOUGH_LIE_ON_PLANE_X_LONGEST
    };
    
  protected:
    virtual void onInit();
    virtual void estimate(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                          const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg);
    virtual void estimateHandle(const HandleType& handle_type,
                                const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg);
    virtual void handleSmallEnoughLieOnPlane(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg,
      bool y_longest);
    virtual void handleSmallEnoughStandOnPlane(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
      const jsk_recognition_msgs::BoundingBox::ConstPtr& box_msg);

    virtual void selectedIndexCallback( const jsk_recognition_msgs::Int32StampedConstPtr &index);

    virtual void subscribe();
    virtual void unsubscribe();
    
    ros::Publisher pub_, pub_best_, pub_preapproach_, pub_selected_, pub_selected_preapproach_;
    ros::Subscriber sub_index_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBox> sub_box_;
    tf::TransformListener* tf_listener_;
    double gripper_size_;
    double approach_offset_;
    int angle_divide_num_;
    boost::circular_buffer<boost::tuple<geometry_msgs::PoseArray, geometry_msgs::PoseArray> > output_buf;
  private:
  };
}

#endif
