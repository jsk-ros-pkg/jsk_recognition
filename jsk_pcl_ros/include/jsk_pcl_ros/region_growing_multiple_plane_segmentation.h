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


#ifndef JSK_PCL_ROS_REGION_GROWING_MULTIPLE_PLANE_SEGMENTATION_H_
#define JSK_PCL_ROS_REGION_GROWING_MULTIPLE_PLANE_SEGMENTATION_H_

#include <dynamic_reconfigure/server.h>
#include "jsk_pcl_ros/RegionGrowingMultiplePlaneSegmentationConfig.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_msgs/PolygonArray.h"
#include "jsk_recognition_msgs/ClusterPointIndices.h"
#include "jsk_recognition_msgs/ModelCoefficientsArray.h"
#include <jsk_recognition_utils/time_util.h>
#include <std_msgs/Float32.h>

namespace jsk_pcl_ros
{
  class RegionGrowingMultiplePlaneSegmentation:
    public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef pcl::PointXYZRGB PointT;
    typedef RegionGrowingMultiplePlaneSegmentationConfig
    Config;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2 > NormalSyncPolicy;
    RegionGrowingMultiplePlaneSegmentation()
      : DiagnosticNodelet("RegionGrowingMultiplePlaneSegmentation"), 
        timer_(10), 
        done_initialization_(false) {}
    
  protected:
    ////////////////////////////////////////////////////////
    // methods
    ////////////////////////////////////////////////////////
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void segment(
      const sensor_msgs::PointCloud2::ConstPtr& msg,
      const sensor_msgs::PointCloud2::ConstPtr& normal_msg);
    virtual void configCallback (Config &config, uint32_t level);
    ////////////////////////////////////////////////////////
    // static methods
    ////////////////////////////////////////////////////////
    // static method
    static void setCondifionFunctionParameter(
      const double angular_threshold,
      const double distance_threshold)
    {
      global_angular_threshold = angular_threshold;
      global_distance_threshold = distance_threshold;
    }
    virtual void ransacEstimation(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
      const pcl::PointIndices::Ptr& indices,
      pcl::PointIndices& inliers,
      pcl::ModelCoefficients& coefficient);
    
    static bool regionGrowingFunction(const pcl::PointXYZRGBNormal& a,
                                      const pcl::PointXYZRGBNormal& b,
                                      float distance)
    {
      if (distance > global_distance_threshold) {
        return false;
      }
      else {
        Eigen::Vector3f a_normal(a.normal_x, a.normal_y, a.normal_z);
        Eigen::Vector3f b_normal(b.normal_x, b.normal_y, b.normal_z);
        double dot = std::abs(a_normal.dot(b_normal));
        double angle;
        if (dot > 1.0) {
          angle = acos(1.0);
        }
        else if (dot < -1.0) {
          angle = acos(-1.0);
        }
        else {
          angle = acos(dot);
        }
        //ROS_INFO("angle: %f", angle);
        if (angle > global_angular_threshold) {
          return false;
        }
        else {
          return true;
        }
      }
    }
    
    ////////////////////////////////////////////////////////
    // ROS variables
    ////////////////////////////////////////////////////////
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_input_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_normal_;
    boost::shared_ptr<
      message_filters::Synchronizer<NormalSyncPolicy> > sync_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    ros::Publisher pub_polygons_;
    ros::Publisher pub_inliers_;
    ros::Publisher pub_coefficients_;
    ros::Publisher pub_clustering_result_;
    ros::Publisher pub_latest_time_;
    ros::Publisher pub_average_time_;
    boost::mutex mutex_;
    jsk_recognition_utils::WallDurationTimer timer_;
    ////////////////////////////////////////////////////////
    // Parameters
    ////////////////////////////////////////////////////////
    double angular_threshold_;
    double distance_threshold_;
    double max_curvature_;
    int min_size_;
    int max_size_;
    double min_area_;
    double max_area_;
    double cluster_tolerance_;
    double ransac_refine_outlier_distance_threshold_;
    int ransac_refine_max_iterations_;
    bool done_initialization_;
    ////////////////////////////////////////////////////////
    // static parameters
    ////////////////////////////////////////////////////////
    static double global_angular_threshold;
    static double global_distance_threshold;
    static boost::mutex global_custom_condigion_function_mutex;
  private:
    
  };
}

#endif
