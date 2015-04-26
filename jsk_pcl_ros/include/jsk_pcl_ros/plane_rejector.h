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

#ifndef JSK_PCL_ROS_PLANE_REJECTOR_H_
#define JSK_PCL_ROS_PLANE_REJECTOR_H_


// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"

#include <dynamic_reconfigure/server.h>
// pcl
#include <pcl_ros/pcl_nodelet.h>

#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include "jsk_pcl_ros/PlaneRejectorConfig.h"

#include "jsk_pcl_ros/pcl_conversion_util.h"
#include <jsk_topic_tools/time_accumulator.h>
#include <jsk_topic_tools/vital_checker.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <jsk_topic_tools/rosparam_utils.h>
#include "jsk_pcl_ros/pcl_util.h"
#include <jsk_topic_tools/connection_based_nodelet.h>


namespace jsk_pcl_ros
{
  class PlaneRejector: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime< jsk_recognition_msgs::PolygonArray,
                                                       jsk_recognition_msgs::ModelCoefficientsArray > SyncPolicy;
    typedef jsk_pcl_ros::PlaneRejectorConfig Config;
  protected:
    virtual void onInit();
    virtual void reject(const jsk_recognition_msgs::PolygonArray::ConstPtr& polygons,
                        const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coefficients);
    virtual void configCallback (Config &config, uint32_t level);

    
    virtual void updateDiagnostics(const ros::TimerEvent& event);
    virtual void updateDiagnosticsPlaneRejector(
      diagnostic_updater::DiagnosticStatusWrapper &stat);
    virtual void subscribe();
    virtual void unsubscribe();
    
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    
    bool use_tf2_;
    std::string processing_frame_id_;
    // axis
    Eigen::Vector3d reference_axis_;
    double angle_thr_;
    tf::TransformListener* listener_;
    boost::mutex mutex_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher polygons_pub_, coefficients_pub_;
    ros::Timer diagnostics_timer_;
    boost::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
    jsk_topic_tools::VitalChecker::Ptr vital_checker_;
    SeriesedBoolean::Ptr tf_success_;
    
    Counter rejected_plane_counter_;
    Counter passed_plane_counter_;
    Counter input_plane_counter_;
  private:
    
  };
}

#endif 
