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

#ifndef JSK_PCL_ROS_STATIC_POLYGON_ARRAY_PUBLISHER_H_
#define JSK_PCL_ROS_STATIC_POLYGON_ARRAY_PUBLISHER_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/pcl_nodelet.h>
#include <jsk_pcl_ros/PolygonArray.h>
#include <jsk_pcl_ros/ModelCoefficientsArray.h>

#include <nodelet/nodelet.h>
#include <topic_tools/shape_shifter.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <jsk_pcl_ros/Int32Stamped.h>
#include <std_msgs/Header.h>
#include "jsk_pcl_ros/pcl_conversion_util.h"

namespace jsk_pcl_ros
{

  class StaticPolygonArrayPublisher: public pcl_ros::PCLNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    Int32Stamped > SyncPolicy;

  protected:
    ros::Publisher polygon_pub_, coefficients_pub_;
    ros::Subscriber sub_;
    jsk_pcl_ros::PolygonArray polygons_;
    jsk_pcl_ros::ModelCoefficientsArray coefficients_;
    ros::Timer periodic_timer_;
    bool use_periodic_;
    bool use_message_;
    bool use_trigger_;
    double periodic_rate_;      // in Hz
    std::vector<std::string> frame_ids_;
    ros::Timer timer_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<Int32Stamped> sub_trigger_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    virtual void onInit();
    virtual void inputCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void timerCallback(const ros::TimerEvent& event);
    virtual void publishPolygon(const ros::Time& stamp);
    virtual bool readPolygonArray(const std::string& param);
    virtual double getXMLDoubleValue(XmlRpc::XmlRpcValue val);
    virtual PCLModelCoefficientMsg polygonToModelCoefficients(const geometry_msgs::PolygonStamped& polygon);
    virtual void triggerCallback(const sensor_msgs::PointCloud2::ConstPtr& input,
                                 const Int32Stamped::ConstPtr& trigger);
  private:
  };

}

#endif
