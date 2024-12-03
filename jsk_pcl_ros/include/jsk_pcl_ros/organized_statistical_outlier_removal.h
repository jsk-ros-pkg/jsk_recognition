// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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
 *     disclaimer in the documentation and/or other materials provided
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


#ifndef JSK_PCL_ROS_ORGANIZED_STATISTICAL_OUTLIER_REMOVAL_H_
#define JSK_PCL_ROS_ORGANIZED_STATISTICAL_OUTLIER_REMOVAL_H_

#include <pcl/pcl_base.h>
#include <pcl_ros/pcl_nodelet.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_topic_tools/counter.h>
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#include "sensor_msgs/PointCloud2.h"
#include "jsk_recognition_msgs/ClusterPointIndices.h"

#include "jsk_pcl_ros/OrganizedStatisticalOutlierRemovalConfig.h"

namespace jsk_pcl_ros
{
  class OrganizedStatisticalOutlierRemoval: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef jsk_pcl_ros::OrganizedStatisticalOutlierRemovalConfig Config;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices > SyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::ClusterPointIndices > ApproximateSyncPolicy;
    typedef pcl::PointXYZRGB PointT;
    OrganizedStatisticalOutlierRemoval();
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();

    virtual void subscribe();

    virtual void unsubscribe();

    virtual void configCallback (Config &config, uint32_t level);

    virtual void filter(
            const pcl::PointCloud<PointT>::Ptr cloud,
            pcl::PointCloud<PointT>::Ptr cloud_filtered,
            bool keep_organized);
    virtual void filterCloud(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void filterCloudWithClusterPointIndices(
            const sensor_msgs::PointCloud2::ConstPtr& msg,
            const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& cpi_msg);


    virtual void updateDiagnostic(
      diagnostic_updater::DiagnosticStatusWrapper &stat);

    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_cpi_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> >sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> >async_;

    ros::Publisher pub_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::mutex mutex_;

    ////////////////////////////////////////////////////////
    // Diagnostics variables
    ////////////////////////////////////////////////////////

    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    bool use_cpi_;
    bool use_async_;
    int queue_size_;
    bool keep_organized_;
    bool negative_;
    double std_mul_;
    int mean_k_;
  private:

  };
}

#endif
