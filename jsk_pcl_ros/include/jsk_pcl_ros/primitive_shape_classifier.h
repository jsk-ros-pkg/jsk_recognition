// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
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
/*
 * primitive_shape_classifier.h
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 */

#ifndef JSK_PCL_ROS_PRIMITIVE_SHAPE_CLASSIFIER_H__
#define JSK_PCL_ROS_PRIMITIVE_SHAPE_CLASSIFIER_H__

#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/ClusterPointIndices.h>
#include <jsk_recognition_msgs/ClassificationResult.h>
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_utils/geo_util.h>
#include <jsk_recognition_utils/pcl_ros_util.h>
#include <jsk_pcl_ros/PrimitiveShapeClassifierConfig.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>

namespace jsk_pcl_ros
{
  class PrimitiveShapeClassifier : public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::PointCloud2,
                                                      sensor_msgs::PointCloud2,
                                                      jsk_recognition_msgs::ClusterPointIndices,
                                                      jsk_recognition_msgs::PolygonArray> SyncPolicy;
    typedef PrimitiveShapeClassifierConfig Config;
    typedef pcl::PointXYZRGBA PointT;

    PrimitiveShapeClassifier() : DiagnosticNodelet("PrimitiveShapeClassifier") {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config& config, uint32_t level);

    virtual void
    process(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud,
            const sensor_msgs::PointCloud2::ConstPtr& ros_normal,
            const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& ros_indices,
            const jsk_recognition_msgs::PolygonArray::ConstPtr& ros_polygons);

    virtual bool
    estimate(const pcl::PointCloud<PointT>::Ptr& cloud,
             const pcl::PointCloud<pcl::Normal>::Ptr& normal,
             const pcl::ModelCoefficients::Ptr& plane,
             pcl::PointIndices::Ptr& boundary_indices,
             pcl::PointCloud<PointT>::Ptr& projected_cloud,
             float& circle_likelihood,
             float& box_likelihood);

    virtual bool
    getSupportPlane(const pcl::PointCloud<PointT>::Ptr& cloud,
                    const std::vector<jsk_recognition_utils::Polygon::Ptr>& polygons,
                    pcl::ModelCoefficients::Ptr& coeff);

    virtual bool
    checkFrameId(const sensor_msgs::PointCloud2::ConstPtr& ros_cloud,
                 const sensor_msgs::PointCloud2::ConstPtr& ros_normal,
                 const jsk_recognition_msgs::ClusterPointIndices::ConstPtr& ros_indices,
                 const jsk_recognition_msgs::PolygonArray::ConstPtr& ros_polygons);

    // properties
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_normal_;
    message_filters::Subscriber<jsk_recognition_msgs::ClusterPointIndices> sub_indices_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygons_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
    ros::Publisher pub_class_;
    ros::Publisher pub_boundary_indices_;
    ros::Publisher pub_projected_cloud_;

    // parameters
    int queue_size_;
    int min_points_num_;
    int sac_max_iterations_;
    double sac_distance_threshold_;
    double sac_radius_limit_min_, sac_radius_limit_max_;
    double box_threshold_, circle_threshold_;
  };
}

#endif // JSK_PCL_ROS_PRIMITIVE_SHAPE_CLASSIFIER_H__
