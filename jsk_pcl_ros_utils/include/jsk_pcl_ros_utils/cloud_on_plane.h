// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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



#ifndef JSK_PCL_ROS_UTILS_CLOUD_ON_PLANE_H_
#define JSK_PCL_ROS_UTILS_CLOUD_ON_PLANE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_utils/geo/convex_polygon.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <jsk_pcl_ros_utils/CloudOnPlaneConfig.h>
#include <dynamic_reconfigure/server.h>
#include <jsk_recognition_msgs/BoolStamped.h>
#include <jsk_recognition_utils/pcl_util.h>

namespace jsk_pcl_ros_utils
{
  class CloudOnPlane: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef boost::shared_ptr<CloudOnPlane> Ptr;
    typedef CloudOnPlaneConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::PolygonArray > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::PolygonArray> ApproximateSyncPolicy;

    CloudOnPlane(): DiagnosticNodelet("CloudOnPlane") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void predicate(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                           const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg);
    virtual void configCallback(Config& config, uint32_t level);
    virtual void publishPredicate(const std_msgs::Header& header, const bool v);
    
    ros::Publisher pub_;
    boost::mutex mutex_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    bool approximate_sync_;
    double distance_thr_;
    int buf_size_;
    jsk_recognition_utils::SeriesedBoolean::Ptr buffer_;
  private:
    
  };
}

#endif
