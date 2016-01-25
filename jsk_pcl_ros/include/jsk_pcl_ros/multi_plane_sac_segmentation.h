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


#ifndef JSK_PCL_ROS_MULTI_PLANE_SEGMENTATION_H_
#define JSK_PCL_ROS_MULTI_PLANE_SEGMENTATION_H_

#include <pcl_ros/pcl_nodelet.h>
#include "jsk_recognition_utils/pcl_util.h"
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include <dynamic_reconfigure/server.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <jsk_pcl_ros/MultiPlaneSACSegmentationConfig.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include "jsk_pcl_ros/tf_listener_singleton.h"


////////////////////////////////////////////////////////
// messages
////////////////////////////////////////////////////////
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <sensor_msgs/Imu.h>

namespace jsk_pcl_ros
{
  class MultiPlaneSACSegmentation: public jsk_topic_tools::ConnectionBasedNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef jsk_pcl_ros::MultiPlaneSACSegmentationConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2 > SyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      jsk_recognition_msgs::ClusterPointIndices > SyncClusterPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::Imu
      > SyncImuPolicy;
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2,
      sensor_msgs::Imu
      > SyncNormalImuPolicy;
    typedef message_filters::Synchronizer<SyncNormalImuPolicy>
    NormalImuSynchronizer;
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void segment(const sensor_msgs::PointCloud2::ConstPtr& msg,
                         const sensor_msgs::PointCloud2::ConstPtr& msg_nromal);
    virtual void segmentWithImu(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                const sensor_msgs::Imu::ConstPtr& imu);
    virtual void segmentWithImu(const sensor_msgs::PointCloud2::ConstPtr& msg,
                                const sensor_msgs::PointCloud2::ConstPtr& msg_nromal,
                                const sensor_msgs::Imu::ConstPtr& imu);
    virtual void segmentWithClusters(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& clusters);
    virtual void applyRecursiveRANSAC(
      const pcl::PointCloud<PointT>::Ptr& input,
      const pcl::PointCloud<pcl::Normal>::Ptr& normal,
      const Eigen::Vector3f& imu_vector,
      std::vector<pcl::PointIndices::Ptr>& output_inliers,
      std::vector<pcl::ModelCoefficients::Ptr>& output_coefficients,
      std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& output_polygons);
    virtual void publishResult(
      const std_msgs::Header& header,
      const std::vector<pcl::PointIndices::Ptr>& inliers,
      const std::vector<pcl::ModelCoefficients::Ptr>& coefficients,
      const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& convexes);
    virtual void configCallback (Config &config, uint32_t level);

    virtual void subscribe();
    virtual void unsubscribe();
    
    ////////////////////////////////////////////////////////
    // ROS variabels
    ////////////////////////////////////////////////////////
    ros::Subscriber sub_;
    ros::Publisher pub_inliers_, pub_coefficients_, pub_polygons_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<SyncClusterPolicy> > sync_cluster_;
    boost::shared_ptr<message_filters::Synchronizer<SyncImuPolicy> > sync_imu_;
    boost::shared_ptr<message_filters::Synchronizer<SyncNormalImuPolicy> > sync_normal_imu_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_normal_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_clusters_;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu_;
    boost::mutex mutex_;
    tf::TransformListener* tf_listener_;
    
    ////////////////////////////////////////////////////////
    // parameters
    ////////////////////////////////////////////////////////
    double outlier_threshold_;
    int min_inliers_;
    int min_points_;
    int max_iterations_;
    bool use_normal_;
    bool use_clusters_;
    bool use_imu_parallel_;
    bool use_imu_perpendicular_;
    double eps_angle_;
    double normal_distance_weight_;
    int min_trial_;
  private:
    
  };
}

#endif
