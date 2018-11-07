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
// Haseru Chen, Kei Okada, Yohei Kakiuchi

#include <boost/thread/mutex.hpp>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <jsk_pcl_ros/PointcloudScreenpointConfig.h>
#include <jsk_recognition_msgs/TransformScreenpoint.h>
#include <jsk_topic_tools/connection_based_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <sensor_msgs/PointCloud2.h>

namespace mf = message_filters;

namespace jsk_pcl_ros
{
  class PointcloudScreenpoint : public jsk_topic_tools::ConnectionBasedNodelet
  {
   protected:
    typedef PointcloudScreenpointConfig Config;

    // approximate sync policies
    typedef mf::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      geometry_msgs::PolygonStamped > PolygonApproxSyncPolicy;

    typedef mf::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      geometry_msgs::PointStamped > PointApproxSyncPolicy;

    typedef mf::sync_policies::ApproximateTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2 > PointCloudApproxSyncPolicy;

    // exact sync policies
    typedef mf::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      geometry_msgs::PolygonStamped > PolygonExactSyncPolicy;

    typedef mf::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      geometry_msgs::PointStamped > PointExactSyncPolicy;

    typedef mf::sync_policies::ExactTime<
      sensor_msgs::PointCloud2,
      sensor_msgs::PointCloud2 > PointCloudExactSyncPolicy;

    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void configCallback(Config &config, uint32_t level);

    // service callback functions
    bool screenpoint_cb(jsk_recognition_msgs::TransformScreenpoint::Request &req,
                        jsk_recognition_msgs::TransformScreenpoint::Response &res);

    // message callback functions
    void points_cb (const sensor_msgs::PointCloud2::ConstPtr &msg);
    void point_cb (const geometry_msgs::PointStamped::ConstPtr &pt_ptr);
    void point_array_cb (const sensor_msgs::PointCloud2::ConstPtr &pt_arr_ptr);
    void rect_cb (const geometry_msgs::PolygonStamped::ConstPtr &array_ptr);
    void poly_cb (const geometry_msgs::PolygonStamped::ConstPtr &array_ptr);

    // synchronized message callback functions
    void sync_point_cb (const sensor_msgs::PointCloud2::ConstPtr &points_ptr,
                        const geometry_msgs::PointStamped::ConstPtr &pt_ptr);
    void sync_point_array_cb (const sensor_msgs::PointCloud2::ConstPtr &points_ptr,
                              const sensor_msgs::PointCloud2::ConstPtr &pt_arr_ptr);
    void sync_rect_cb (const sensor_msgs::PointCloud2ConstPtr &points_ptr,
                       const geometry_msgs::PolygonStamped::ConstPtr &array_ptr);
    void sync_poly_cb (const sensor_msgs::PointCloud2::ConstPtr &points_ptr,
                       const geometry_msgs::PolygonStamped::ConstPtr &array_ptr);

    // internal functions
    bool checkpoint (const pcl::PointCloud< pcl::PointXYZ > &in_pts,
                     int x, int y,
                     float &resx, float &resy, float &resz);
    bool extract_point (const pcl::PointCloud< pcl::PointXYZ > &in_pts,
                        int reqx, int reqy,
                        float &resx, float &resy, float &resz);
    void extract_rect (const pcl::PointCloud< pcl::PointXYZ > &in_pts,
                       int st_x, int st_y,
                       int ed_x, int ed_y,
                       sensor_msgs::PointCloud2 &out_pts);

    // ros
    boost::mutex mutex_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > dyn_srv_;
    ros::Publisher pub_points_;
    ros::Publisher pub_point_;
    ros::Publisher pub_polygon_;
    ros::ServiceServer srv_;
    mf::Subscriber < sensor_msgs::PointCloud2 > points_sub_;
    mf::Subscriber < geometry_msgs::PolygonStamped > rect_sub_;
    mf::Subscriber < geometry_msgs::PointStamped > point_sub_;
    mf::Subscriber < sensor_msgs::PointCloud2 > point_array_sub_;
    mf::Subscriber < geometry_msgs::PolygonStamped > poly_sub_;
    boost::shared_ptr < mf::Synchronizer < PolygonApproxSyncPolicy > > async_rect_;
    boost::shared_ptr < mf::Synchronizer < PointApproxSyncPolicy > > async_point_;
    boost::shared_ptr < mf::Synchronizer < PointCloudApproxSyncPolicy > > async_point_array_;
    boost::shared_ptr < mf::Synchronizer < PolygonApproxSyncPolicy > > async_poly_;

    boost::shared_ptr < mf::Synchronizer < PolygonExactSyncPolicy > > sync_rect_;
    boost::shared_ptr < mf::Synchronizer < PointExactSyncPolicy > > sync_point_;
    boost::shared_ptr < mf::Synchronizer < PointCloudExactSyncPolicy > > sync_point_array_;
    boost::shared_ptr < mf::Synchronizer < PolygonExactSyncPolicy > > sync_poly_;

    // pcl
    pcl::NormalEstimation< pcl::PointXYZ, pcl::Normal > n3d_;
    pcl::search::KdTree< pcl::PointXYZ >::Ptr normals_tree_;

    // parameters
    bool synchronization_, approximate_sync_;
    int queue_size_;
    int search_size_;
    int crop_size_;
    double timeout_;

    std_msgs::Header latest_cloud_header_;
    pcl::PointCloud<pcl::PointXYZ> latest_cloud_;

  };
}

