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

#ifndef JSK_PCL_ROS_UTILS_POLYGON_TRANSFORMER_H_
#define JSK_PCL_ROS_UTILS_POLYGON_TRANSFORMER_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/pcl_nodelet.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>

#include "jsk_recognition_utils/tf_listener_singleton.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/connection_based_nodelet.h>

namespace jsk_pcl_ros_utils
{
  class PolygonArrayTransformer: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    jsk_recognition_msgs::PolygonArray,
    jsk_recognition_msgs::ModelCoefficientsArray > SyncPolicy;
  protected:
    virtual void onInit();
    virtual void transform(const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
                           const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients);
    virtual void computeCoefficients(const geometry_msgs::PolygonStamped& polygon,
                                     PCLModelCoefficientMsg& coefficient);
    virtual void transformModelCoefficient(const Eigen::Affine3d& transform,
                                           const PCLModelCoefficientMsg& coefficient,
                                           PCLModelCoefficientMsg& result);
    virtual void transformPolygon(const Eigen::Affine3d& transform,
                                  const geometry_msgs::PolygonStamped& polygon,
                                  geometry_msgs::PolygonStamped& result);
    virtual void subscribe();
    virtual void unsubscribe();
    ros::Publisher polygons_pub_, coefficients_pub_;
    tf::TransformListener* listener_;
    std::string frame_id_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
  private:
  };
}

#endif
