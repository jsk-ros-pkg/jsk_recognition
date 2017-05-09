// -*- mode: c++ -*-
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


#ifndef JSK_PCL_ROS_TILT_LASER_LISTENER_H_
#define JSK_PCL_ROS_TILT_LASER_LISTENER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <sensor_msgs/JointState.h>
#include <jsk_recognition_msgs/TimeRange.h>
#include "jsk_pcl_ros/line_segment_collector.h"
#include <std_srvs/Empty.h>
#include <geometry_msgs/TwistStamped.h>

namespace jsk_pcl_ros
{
  class StampedJointAngle
  {
  public:
    typedef boost::shared_ptr<StampedJointAngle> Ptr;
    StampedJointAngle(const std_msgs::Header& header_arg, const double& value);
    virtual ~StampedJointAngle() {}
    std_msgs::Header header;
    virtual double getValue() { return value_; }
  protected:
    double value_;
  private:
    
  };
  
  class TiltLaserListener: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    TiltLaserListener(): DiagnosticNodelet("TiltLaserListener") { };
    enum LaserType {
      INFINITE_SPINDLE, INFINITE_SPINDLE_HALF, TILT, TILT_HALF_UP, TILT_HALF_DOWN, PERIODIC
    };
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);
    virtual void processTiltHalfUp(const ros::Time& stamp, const double& value);
    virtual void processTiltHalfDown(const ros::Time& stamp, const double& value);
    virtual void processTilt(const ros::Time& stamp, const double& value);
    virtual void processInfiniteSpindle(
      const ros::Time& stamp, const double& joint_angle, const double& velocity,
      const double& threshold);
    virtual void publishTimeRange(const ros::Time& stamp,
                                  const ros::Time& start,
                                  const ros::Time& end);
    virtual bool clearCacheCallback(
      std_srvs::Empty::Request& req,
      std_srvs::Empty::Response& res);
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void getPointCloudFromLocalBuffer(
      const std::vector<sensor_msgs::PointCloud2::ConstPtr>& target_clouds,
      sensor_msgs::PointCloud2& output_cloud);
    virtual void timerCallback(const ros::TimerEvent& e);
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Subscriber sub_cloud_;
    ros::Publisher trigger_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher twist_pub_;
    ros::ServiceServer clear_cache_service_;
    ros::ServiceClient assemble_cloud_srv_;
    jsk_topic_tools::VitalChecker::Ptr cloud_vital_checker_;
    ros::Timer periodic_timer_;

    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    LaserType laser_type_;
    std::string joint_name_;
    double prev_angle_;
    double prev_velocity_;
    double start_angle_;
    double overwrap_angle_;
    ros::Time start_time_;
    bool use_laser_assembler_;
    bool not_use_laser_assembler_service_;
    bool clear_assembled_scans_;
    boost::mutex mutex_;
    boost::mutex cloud_mutex_;
    TimeStampedVector<StampedJointAngle::Ptr> buffer_;
    TimeStampedVector<sensor_msgs::PointCloud2::ConstPtr> cloud_buffer_;
    int skip_number_;
    int skip_counter_;
    int max_queue_size_;
    double publish_rate_;
    std::string twist_frame_id_;
  private:
    
  };
}

#endif
