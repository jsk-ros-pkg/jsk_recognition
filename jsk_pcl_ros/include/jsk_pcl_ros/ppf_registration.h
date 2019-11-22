// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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

#ifndef JSK_PCL_ROS_ADD_COLOR_FROM_IMAGE_H_
#define JSK_PCL_ROS_ADD_COLOR_FROM_IMAGE_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <dynamic_reconfigure/server.h>

#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PointsArray.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <pcl/features/ppf.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/ppf_registration.h>
#include <jsk_pcl_ros/PPFRegistrationConfig.h>

namespace jsk_pcl_ros
{
  class PPFRegistration: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::PointsArray> ArraySyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    jsk_recognition_msgs::PointsArray> ArrayApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2> CloudSyncPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::PointCloud2,
    sensor_msgs::PointCloud2> CloudApproximateSyncPolicy;
    typedef jsk_pcl_ros::PPFRegistrationConfig Config;
    PPFRegistration(): DiagnosticNodelet("PPFRegistration") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual pcl::PointCloud<pcl::PointNormal>::Ptr calculateNormals (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    virtual void configCallback(Config &config, uint32_t level);
    virtual void ArrayRegistration(
            const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
            const jsk_recognition_msgs::PointsArray::ConstPtr& input_reference_points_array);
    virtual void CloudRegistration(
            const sensor_msgs::PointCloud2::ConstPtr& input_cloud,
            const sensor_msgs::PointCloud2::ConstPtr& input_reference_cloud);
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_reference_cloud_;
    message_filters::Subscriber<jsk_recognition_msgs::PointsArray> sub_reference_array_;
    boost::mutex mutex_;
    boost::shared_ptr<message_filters::Synchronizer<ArraySyncPolicy> > array_sync_;
    boost::shared_ptr<message_filters::Synchronizer<ArrayApproximateSyncPolicy> > array_async_;
    boost::shared_ptr<message_filters::Synchronizer<CloudSyncPolicy> > cloud_sync_;
    boost::shared_ptr<message_filters::Synchronizer<CloudApproximateSyncPolicy> > cloud_async_;
    bool approximate_sync_;
    int queue_size_;
    double search_radius_;
    int sampling_rate_;
    bool use_array_;
    double position_clustering_threshold_;
    double rotation_clustering_threshold_;
    ros::Publisher pub_pose_array_;
    ros::Publisher pub_points_array_;
    ros::Publisher pub_pose_stamped_;
    ros::Publisher pub_cloud_;

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals;
    pcl::PointCloud<pcl::PointNormal>::Ptr reference_cloud_with_normals;
    pcl::PPFEstimation<pcl::PointNormal, pcl::PointNormal, pcl::PPFSignature> ppf_estimator;
    pcl::PPFRegistration<pcl::PointNormal, pcl::PointNormal> ppf_registration;
  private:

  };
}

#endif
