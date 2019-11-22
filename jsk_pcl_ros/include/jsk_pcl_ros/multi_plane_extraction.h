// -*- mode: C++ -*-
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
#ifndef JSK_PCL_ROS_MULTI_PLANE_EXTRACTION_H_
#define JSK_PCL_ROS_MULTI_PLANE_EXTRACTION_H_

#include <ros/ros.h>
#include <ros/names.h>
#include <pcl_ros/pcl_nodelet.h>
#include <message_filters/pass_through.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "sensor_msgs/PointCloud2.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/MultiPlaneExtractionConfig.h"
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <jsk_topic_tools/vital_checker.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"

namespace jsk_pcl_ros
{
  class MultiPlaneExtraction: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices,
    jsk_recognition_msgs::ModelCoefficientsArray,
    jsk_recognition_msgs::PolygonArray> SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices,
      jsk_recognition_msgs::ModelCoefficientsArray,
      jsk_recognition_msgs::PolygonArray> ASyncPolicy;
    typedef jsk_pcl_ros::MultiPlaneExtractionConfig Config;

    MultiPlaneExtraction(): DiagnosticNodelet("MultiPlaneExtraction") { }
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    
    virtual void fillEmptyIndices(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons);
    virtual void fillEmptyCoefficients(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons);

    virtual void extract(const sensor_msgs::PointCloud2::ConstPtr& input,
                         const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& indices,
                         const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients,
                         const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons);
    
    virtual void configCallback (Config &config, uint32_t level);

    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);

    virtual void subscribe();
    virtual void unsubscribe();
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    boost::mutex mutex_;
    ros::Publisher pub_, nonplane_pub_;
    ros::Publisher pub_indices_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::PassThrough<jsk_recognition_msgs::ClusterPointIndices> null_indices_;
    message_filters::PassThrough<jsk_recognition_msgs::ModelCoefficientsArray> null_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ASyncPolicy> > async_;


    ////////////////////////////////////////////////////////
    // Diagnostics Variables
    ////////////////////////////////////////////////////////
    jsk_recognition_utils::Counter plane_counter_;
    
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    tf::TransformListener* tf_listener_;
    bool use_async_;
    bool keep_organized_;
    int maximum_queue_size_;
    double min_height_, max_height_;
    bool use_indices_;
    double magnify_;
    bool use_sensor_frame_;
    std::string sensor_frame_;
    bool use_coefficients_;
  private:
    
    
  };
}

#endif
