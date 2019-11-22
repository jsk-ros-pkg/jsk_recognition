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


#ifndef BOUNDING_BOX_FILTER_H_
#define BOUNDING_BOX_FILTER_H_

#include <pcl_ros/pcl_nodelet.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_pcl_ros/BoundingBoxFilterConfig.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>

namespace jsk_pcl_ros
{
  class BoundingBoxFilter: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef jsk_pcl_ros::BoundingBoxFilterConfig Config;

    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::BoundingBoxArray,
      jsk_recognition_msgs::ClusterPointIndices
      > SyncPolicy;

    BoundingBoxFilter() : DiagnosticNodelet("BoundingBoxFilter") {}

  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void filter(
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg);
    virtual void filterWithIndices(
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices_msg);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void subscribe();
    virtual void unsubscribe();
    void filterBoundingBoxes(
      const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& box_array_msg,
      std::vector<size_t>& keep);

    ////////////////////////////////////////////////////////
    // ROS varariables
    ////////////////////////////////////////////////////////
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<jsk_recognition_msgs::BoundingBoxArray> sub_box_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    ros::Publisher filtered_box_pub_;
    ros::Publisher filtered_indices_pub_;
    boost::mutex mutex_;
    
    ////////////////////////////////////////////////////////
    // Diagnostics Variables
    ////////////////////////////////////////////////////////
    jsk_recognition_utils::Counter remove_counter_;
    jsk_recognition_utils::Counter pass_counter_;
    
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    bool filter_limit_negative_;
    bool use_x_dimension_;
    bool use_y_dimension_;
    bool use_z_dimension_;
    double x_dimension_min_;
    double x_dimension_max_;
    double y_dimension_min_;
    double y_dimension_max_;
    double z_dimension_min_;
    double z_dimension_max_;
    
    bool with_indices_;
    
  private:
    
  };
}

#endif
