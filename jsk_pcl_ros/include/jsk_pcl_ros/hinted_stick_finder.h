// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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


#ifndef JSK_PCL_ROS_HINTED_STICK_FINDER_H_
#define JSK_PCL_ROS_HINTED_STICK_FINDER_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/PolygonStamped.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_geometry/pinhole_camera_model.h>
#include "jsk_recognition_utils/geo_util.h"
#include <dynamic_reconfigure/server.h>
#include <jsk_pcl_ros/HintedStickFinderConfig.h>

namespace jsk_pcl_ros
{
  class HintedStickFinder: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef message_filters::sync_policies::ApproximateTime<
    geometry_msgs::PolygonStamped, // line
    sensor_msgs::CameraInfo,       // camera info
    sensor_msgs::PointCloud2> ASyncPolicy;
    typedef HintedStickFinderConfig Config;
    HintedStickFinder(): DiagnosticNodelet("HintedStickFinder") {}
  protected:

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);

    /** @brief
     * Synchronized message callback
     */
    virtual void detect(
      const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg,
      const sensor_msgs::CameraInfo::ConstPtr& camera_info_msg,
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /** @brief
     * Non synchronized message callback for ~input pointcloud.
     */
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

    /** @brief
     * Non synchronized message callback for ~input/hint/line
     */
    virtual void hintCallback(
      const geometry_msgs::PolygonStamped::ConstPtr& hint_msg);

    /** @brief
     * Non synchronized message callback for ~input/camera_info
     */
    virtual void infoCallback(
      const sensor_msgs::CameraInfo::ConstPtr& info_msg);
    
    virtual jsk_recognition_utils::ConvexPolygon::Ptr polygonFromLine(
      const geometry_msgs::PolygonStamped::ConstPtr& polygon_msg,
      const image_geometry::PinholeCameraModel& model,
      Eigen::Vector3f& a,
      Eigen::Vector3f& b);
    
    virtual void filterPointCloud(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const jsk_recognition_utils::ConvexPolygon::Ptr polygon,
      pcl::PointIndices& output_indices);
    virtual void normalEstimate(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const pcl::PointIndices::Ptr indices,
      pcl::PointCloud<pcl::Normal>& normals,
      pcl::PointCloud<pcl::PointXYZ>& normals_cloud);
    
    virtual void fittingCylinder(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud,
      const pcl::PointCloud<pcl::Normal>::Ptr& cloud_nromals,
      const Eigen::Vector3f& a,
      const Eigen::Vector3f& b);

    /** @brief
     * Check direction of cylinder in 2-D image coordinate system and if it is
     * larger than eps_2d_angle_, return false
     *
     * @param cylinder Cylinder object
     * @param a 3-D ray to start point of 2-D line
     * @param b 3-D ray to end point of 2-D line
     */
    virtual bool rejected2DHint(
      const jsk_recognition_utils::Cylinder::Ptr& cylinder,
      const Eigen::Vector3f& a,
      const Eigen::Vector3f& b);
    
    boost::mutex mutex_;
    
    message_filters::Subscriber<geometry_msgs::PolygonStamped> sub_polygon_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_info_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_cloud_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_normal_;
    boost::shared_ptr<message_filters::Synchronizer<ASyncPolicy> > sync_;

    ros::Publisher pub_line_filtered_indices_;
    ros::Publisher pub_line_filtered_normal_;
    ros::Publisher pub_cylinder_marker_;
    ros::Publisher pub_cylinder_pose_;
    ros::Publisher pub_inliers_;
    ros::Publisher pub_coefficients_;
    // params from continuous_mode
    ros::Subscriber sub_no_sync_cloud_;
    ros::Subscriber sub_no_sync_camera_info_;
    ros::Subscriber sub_no_sync_polygon_;
    boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
    
    double max_radius_;
    double min_radius_;
    double filter_distance_;
    double outlier_threshold_;
    int max_iteration_;
    double eps_angle_;
    double min_probability_;
    int cylinder_fitting_trial_;
    int min_inliers_;
    double eps_2d_angle_;
    
    /** @brief
     *  True if use ~input has normal fields.
     */
    bool use_normal_;

    /** @brief
     *  Run in continuous mode. continuous mode means this nodelet does not synchronize
     *  hint and input messages but keep processing with old hint information.
     */
    bool not_synchronize_;
    
    sensor_msgs::CameraInfo::ConstPtr latest_camera_info_;
    geometry_msgs::PolygonStamped::ConstPtr latest_hint_;
    
  private:
    
  };
}

#endif
