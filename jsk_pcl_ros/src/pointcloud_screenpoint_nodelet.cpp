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

#include <jsk_pcl_ros/pointcloud_screenpoint.h>
#include <pcl_conversions/pcl_conversions.h>

// F/K/A <ray converter>
// Haseru Chen, Kei Okada, Yohei Kakiuchi

namespace mf = message_filters;

namespace jsk_pcl_ros
{

void PointcloudScreenpoint::onInit()
{
  ConnectionBasedNodelet::onInit();


  normals_tree_ = boost::make_shared< pcl::search::KdTree< pcl::PointXYZ > > ();
  n3d_.setSearchMethod (normals_tree_);

  dyn_srv_ = boost::make_shared< dynamic_reconfigure::Server< Config > >(*pnh_);
  dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind(&PointcloudScreenpoint::configCallback, this, _1, _2);
  dyn_srv_->setCallback(f);

  srv_ = pnh_->advertiseService(
      "screen_to_point", &PointcloudScreenpoint::screenpoint_cb, this);

  pub_points_  = advertise< sensor_msgs::PointCloud2 >(*pnh_, "output", 1);
  pub_point_   = advertise< geometry_msgs::PointStamped >(*pnh_, "output_point", 1);
  pub_polygon_ = advertise< geometry_msgs::PolygonStamped >(*pnh_, "output_polygon", 1);

  onInitPostProcess();
}

void PointcloudScreenpoint::subscribe()
{
  points_sub_.subscribe(*pnh_, "points", 1);
  rect_sub_.subscribe(*pnh_, "rect", 1);
  poly_sub_.subscribe(*pnh_, "poly", 1);
  point_sub_.subscribe(*pnh_, "point", 1);
  point_array_sub_.subscribe(*pnh_, "point_array", 1);

  if (synchronization_)
  {
    if (approximate_sync_)
    {
      async_rect_ = boost::make_shared<
        mf::Synchronizer<PolygonApproxSyncPolicy> >(queue_size_);
      async_rect_->connectInput(points_sub_, rect_sub_);
      async_rect_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_rect_cb, this, _1, _2));

      async_poly_ = boost::make_shared<
        mf::Synchronizer<PolygonApproxSyncPolicy> >(queue_size_);
      async_poly_->connectInput(points_sub_, rect_sub_);
      async_poly_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_poly_cb, this, _1, _2));

      async_point_ = boost::make_shared<
        mf::Synchronizer<PointApproxSyncPolicy> >(queue_size_);
      async_point_->connectInput(points_sub_, point_sub_);
      async_point_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_point_cb, this, _1, _2));

      async_point_array_ = boost::make_shared<
        mf::Synchronizer<PointCloudApproxSyncPolicy> >(queue_size_);
      async_point_array_->connectInput(points_sub_, point_array_sub_);
      async_point_array_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_point_array_cb, this, _1, _2));
    }
    else
    {
      sync_rect_ = boost::make_shared<
        mf::Synchronizer<PolygonExactSyncPolicy> >(queue_size_);
      sync_rect_->connectInput(points_sub_, rect_sub_);
      sync_rect_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_rect_cb, this, _1, _2));

      sync_poly_ = boost::make_shared<
        mf::Synchronizer<PolygonExactSyncPolicy> >(queue_size_);
      sync_poly_->connectInput(points_sub_, rect_sub_);
      sync_poly_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_poly_cb, this, _1, _2));

      sync_point_ = boost::make_shared<
        mf::Synchronizer<PointExactSyncPolicy> >(queue_size_);
      sync_point_->connectInput(points_sub_, point_sub_);
      sync_point_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_point_cb, this, _1, _2));

      sync_point_array_ = boost::make_shared<
        mf::Synchronizer<PointCloudExactSyncPolicy> >(queue_size_);
      sync_point_array_->connectInput(points_sub_, point_array_sub_);
      sync_point_array_->registerCallback(
          boost::bind(&PointcloudScreenpoint::sync_point_array_cb, this, _1, _2));
    }
  }
  else
  {
    points_sub_.registerCallback(
        boost::bind(&PointcloudScreenpoint::points_cb, this, _1));
    rect_sub_.registerCallback(
        boost::bind(&PointcloudScreenpoint::rect_cb, this, _1));
    poly_sub_.registerCallback(
        boost::bind(&PointcloudScreenpoint::poly_cb, this, _1));
    point_sub_.registerCallback(
        boost::bind(&PointcloudScreenpoint::point_cb, this, _1));
    point_array_sub_.registerCallback(
        boost::bind(&PointcloudScreenpoint::point_array_cb, this, _1));
  }
}

void PointcloudScreenpoint::unsubscribe()
{
  points_sub_.unsubscribe();
  rect_sub_.unsubscribe();
  poly_sub_.unsubscribe();
  point_sub_.unsubscribe();
  point_array_sub_.unsubscribe();
}

void PointcloudScreenpoint::configCallback(Config& config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  bool need_resubscribe = false;
  if (synchronization_ != config.synchronization ||
      approximate_sync_ != config.approximate_sync ||
      queue_size_ != config.queue_size)
    need_resubscribe = true;

  synchronization_ = config.synchronization;
  approximate_sync_ = config.approximate_sync;
  queue_size_ = config.queue_size;
  crop_size_ = config.crop_size;
  timeout_ = config.timeout;

  if (search_size_ != config.search_size)
  {
    search_size_ = config.search_size;
    n3d_.setKSearch (search_size_);
  }

  if (need_resubscribe && isSubscribed())
  {
    unsubscribe();
    subscribe();
  }
}

// Service callback functions
bool PointcloudScreenpoint::screenpoint_cb (
    jsk_recognition_msgs::TransformScreenpoint::Request &req,
    jsk_recognition_msgs::TransformScreenpoint::Response &res)
{
  NODELET_DEBUG("PointcloudScreenpoint::screenpoint_cb");
  boost::mutex::scoped_lock lock(mutex_);

  if ( latest_cloud_.empty() && req.no_update ) {
    NODELET_ERROR("no point cloud was received");
    return false;
  }

  bool need_unsubscribe = false;
  if (!points_sub_.getSubscriber()) {
    points_sub_.registerCallback(
        boost::bind(&PointcloudScreenpoint::points_cb, this, _1));
    need_unsubscribe = true;
  }

  // wait for cloud
  ros::Time start = ros::Time::now();
  ros::Rate r(10);
  while (ros::ok())
  {
    if ( !latest_cloud_.empty() ) break;
    if ((ros::Time::now() - start).toSec() > timeout_) break;
    r.sleep();
    ros::spinOnce();
    NODELET_INFO_THROTTLE(1.0, "Waiting for point cloud...");
  }

  if (need_unsubscribe)
  {
    points_sub_.unsubscribe();
  }

  if (latest_cloud_.empty())
  {
    NODELET_ERROR("no point cloud was received");
    return false;
  }

  res.header = latest_cloud_header_;

  bool ret;
  float rx, ry, rz;
  ret = extract_point (latest_cloud_, req.x, req.y, rx, ry, rz);
  res.point.x = rx; res.point.y = ry; res.point.z = rz;

  if (!ret) {
    NODELET_ERROR("Failed to extract point");
    return false;
  }

  // search normal
  n3d_.setSearchSurface(
      boost::make_shared<pcl::PointCloud<pcl::PointXYZ > > (latest_cloud_));

  pcl::PointCloud<pcl::PointXYZ> cloud_;
  pcl::PointXYZ pt;
  pt.x = res.point.x;
  pt.y = res.point.y;
  pt.z = res.point.z;
  cloud_.points.resize(0);
  cloud_.points.push_back(pt);

  n3d_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ > > (cloud_));
  pcl::PointCloud<pcl::Normal> cloud_normals;
  n3d_.compute (cloud_normals);

  res.vector.x = cloud_normals.points[0].normal_x;
  res.vector.y = cloud_normals.points[0].normal_y;
  res.vector.z = cloud_normals.points[0].normal_z;

  if((res.point.x * res.vector.x + res.point.y * res.vector.y + res.point.z * res.vector.z) < 0) {
    res.vector.x *= -1;
    res.vector.y *= -1;
    res.vector.z *= -1;
  }

  if (pub_point_.getNumSubscribers() > 0) {
    geometry_msgs::PointStamped ps;
    ps.header = latest_cloud_header_;
    ps.point.x = res.point.x;
    ps.point.y = res.point.y;
    ps.point.z = res.point.z;
    pub_point_.publish(ps);
  }

  return true;
}

// Message callback functions
void PointcloudScreenpoint::points_cb(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  NODELET_DEBUG("PointcloudScreenpoint::points_cb, width=%d, height=%d, fields=%ld",
                msg->width, msg->height, msg->fields.size());

  latest_cloud_header_ = msg->header;
  pcl::fromROSMsg (*msg, latest_cloud_);
}

void PointcloudScreenpoint::point_cb (const geometry_msgs::PointStampedConstPtr& pt_ptr)
{
  if (latest_cloud_.empty())
  {
    NODELET_ERROR_THROTTLE(1.0, "no point cloud was received");
    return;
  }
  if (pub_point_.getNumSubscribers() > 0)
  {
    geometry_msgs::PointStamped ps;
    bool ret; float rx, ry, rz;
    ret = extract_point (latest_cloud_, pt_ptr->point.x, pt_ptr->point.y, rx, ry, rz);

    if (ret) {
      ps.point.x = rx; ps.point.y = ry; ps.point.z = rz;
      ps.header = latest_cloud_header_;
      pub_point_.publish (ps);
    }
  }
  if (pub_points_.getNumSubscribers() > 0)
  {
    int st_x = pt_ptr->point.x - crop_size_;
    int st_y = pt_ptr->point.y - crop_size_;
    int ed_x = pt_ptr->point.x + crop_size_;
    int ed_y = pt_ptr->point.y + crop_size_;
    sensor_msgs::PointCloud2 out_pts;
    extract_rect (latest_cloud_, st_x, st_y, ed_x, ed_y, out_pts);
    pub_points_.publish(out_pts);
  }
}

void PointcloudScreenpoint::point_array_cb (const sensor_msgs::PointCloud2ConstPtr& pt_arr_ptr)
{
  if (latest_cloud_.empty())
  {
    NODELET_ERROR_THROTTLE(1.0, "no point cloud was received");
    return;
  }
  if (pub_points_.getNumSubscribers() > 0) {
    pcl::PointCloud<pcl::PointXY>::Ptr point_array_cloud(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*pt_arr_ptr, *point_array_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    result_cloud->header = pcl_conversions::toPCL(latest_cloud_header_); // FIXME: if hydro
    for (size_t i = 0; i < point_array_cloud->points.size(); i++) {
      pcl::PointXY point = point_array_cloud->points[i];
      geometry_msgs::PointStamped ps;
      bool ret; float rx, ry, rz;
      ret = extract_point (latest_cloud_, point.x, point.y, rx, ry, rz);
      if (ret) {
        pcl::PointXYZ point_on_screen;
        point_on_screen.x = rx;
        point_on_screen.y = ry;
        point_on_screen.z = rz;
        result_cloud->points.push_back(point_on_screen);
      }
    }
    sensor_msgs::PointCloud2::Ptr ros_cloud(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*result_cloud, *ros_cloud);
    ros_cloud->header = latest_cloud_header_;
    pub_points_.publish(ros_cloud);
  }
}

void PointcloudScreenpoint::rect_cb (const geometry_msgs::PolygonStampedConstPtr& array_ptr) {
  if (latest_cloud_.empty())
  {
    NODELET_ERROR_THROTTLE(1.0, "no point cloud was received");
    return;
  }

  int num_pts = array_ptr->polygon.points.size();
  if ( num_pts < 2) {
    NODELET_ERROR("Point size must be 2.");
    return;
  } else if ( num_pts > 2 ) {
    NODELET_WARN("Expected point size is 2, got %ld. "
                 "Used first 2 points to compute mid-coords.",
                 array_ptr->polygon.points.size());
  }

  int st_x = array_ptr->polygon.points[0].x;
  int st_y = array_ptr->polygon.points[0].y;
  int ed_x = array_ptr->polygon.points[1].x;
  int ed_y = array_ptr->polygon.points[1].y;

  if (pub_point_.getNumSubscribers() > 0)
  {
    geometry_msgs::PointStamped ps;
    bool ret; float rx, ry, rz;
    ret = extract_point (latest_cloud_, (st_x + ed_x) / 2, (st_y + ed_y) / 2, rx, ry, rz);

    if (ret) {
      ps.point.x = rx; ps.point.y = ry; ps.point.z = rz;
      ps.header = latest_cloud_header_;
      pub_point_.publish (ps);
    }
  }
  if (pub_points_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 out_pts;
    extract_rect (latest_cloud_, st_x, st_y, ed_x, ed_y, out_pts);
    pub_points_.publish(out_pts);
  }
}

void PointcloudScreenpoint::poly_cb(const geometry_msgs::PolygonStampedConstPtr& array_ptr)
{
  if (latest_cloud_.empty())
  {
    NODELET_ERROR_THROTTLE(1.0, "no point cloud was received");
    return;
  }
  if (pub_polygon_.getNumSubscribers() > 0)
  {
    geometry_msgs::PolygonStamped result_polygon;
    result_polygon.header = latest_cloud_header_;
    for (size_t i = 0; i < array_ptr->polygon.points.size(); i++) {
      geometry_msgs::Point32 p = array_ptr->polygon.points[i];
      float rx, ry, rz;
      bool ret = extract_point (latest_cloud_, p.x, p.y, rx, ry, rz);
      if (!ret) {
        NODELET_ERROR("Failed to project point");
        continue;
      }
      geometry_msgs::Point32 p_projected;
      p_projected.x = rx;
      p_projected.y = ry;
      p_projected.z = rz;
      result_polygon.polygon.points.push_back(p_projected);
    }
    pub_polygon_.publish(result_polygon);
  }
}

void PointcloudScreenpoint::sync_point_cb (
    const sensor_msgs::PointCloud2::ConstPtr& points_ptr,
    const geometry_msgs::PointStamped::ConstPtr& pt_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);
  points_cb (points_ptr);
  point_cb (pt_ptr);
}

void PointcloudScreenpoint::sync_point_array_cb (
    const sensor_msgs::PointCloud2::ConstPtr& points_ptr,
    const sensor_msgs::PointCloud2::ConstPtr& pt_arr_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);
  points_cb (points_ptr);
  point_array_cb(pt_arr_ptr);
}

void PointcloudScreenpoint::sync_poly_cb(
    const sensor_msgs::PointCloud2::ConstPtr& points_ptr,
    const geometry_msgs::PolygonStamped::ConstPtr& array_ptr)
{
  boost::mutex::scoped_lock lock(mutex_);
  points_cb (points_ptr);
  poly_cb(array_ptr);
}

void PointcloudScreenpoint::sync_rect_cb(
    const sensor_msgs::PointCloud2::ConstPtr& points_ptr,
    const geometry_msgs::PolygonStamped::ConstPtr& array_ptr) {
  boost::mutex::scoped_lock lock(mutex_);
  points_cb(points_ptr);
  rect_cb (array_ptr);
}

bool PointcloudScreenpoint::checkpoint (const pcl::PointCloud< pcl::PointXYZ > &in_pts,
                                        int x, int y,
                                        float &resx, float &resy, float &resz)
{
  if ((x < 0) || (y < 0) || (x >= in_pts.width) || (y >= in_pts.height)) {
    NODELET_WARN("Requested point is out of image size. "
             "point: (%d, %d)  size: (%d, %d)",
             x, y, in_pts.width, in_pts.height);
    return false;
  }

  pcl::PointXYZ p = in_pts.points[in_pts.width * y + x];
  // search near points
  NODELET_DEBUG("Request: screenpoint (%d, %d) => (%f, %f, %f)", x, y, p.x, p.y, p.z);
  //return !(isnan (p.x) || ( (p.x == 0.0) && (p.y == 0.0) && (p.z == 0.0)));

  if ( !std::isnan (p.x) && ((p.x != 0.0) || (p.y != 0.0) || (p.z == 0.0)) ) {
    resx = p.x; resy = p.y; resz = p.z;
    return true;
  }
  return false;
}

bool PointcloudScreenpoint::extract_point(const pcl::PointCloud< pcl::PointXYZ > &in_pts,
                                          int reqx, int reqy,
                                          float &resx, float &resy, float &resz)
{
  int x, y;

  x = reqx < 0.0 ? ceil(reqx - 0.5) : floor(reqx + 0.5);
  y = reqy < 0.0 ? ceil(reqy - 0.5) : floor(reqy + 0.5);
  NODELET_DEBUG("Request : %d %d", x, y);

  if (checkpoint (in_pts, x, y, resx, resy, resz)) {
    return true;
  } else {
    for (int n = 1; n < crop_size_; n++) {
      for (int y2 = 0; y2 <= n; y2++) {
        int x2 = n - y2;
        if (checkpoint (in_pts, x + x2, y + y2, resx, resy, resz)) {
          return true;
        }
        if (x2 != 0 && y2 != 0) {
          if (checkpoint (in_pts, x - x2, y - y2, resx, resy, resz)) {
            return true;
          }
        }
        if (x2 != 0) {
          if (checkpoint (in_pts, x - x2, y + y2, resx, resy, resz)) {
            return true;
          }
        }
        if (y2 != 0) {
          if (checkpoint (in_pts, x + x2, y - y2, resx, resy, resz)) {
            return true;
          }
        }
      }
    }
  }
  return false;
}

void PointcloudScreenpoint::extract_rect (const pcl::PointCloud< pcl::PointXYZ >& in_pts,
                                          int st_x, int st_y,
                                          int ed_x, int ed_y,
                                          sensor_msgs::PointCloud2& out_pts)
{
  sensor_msgs::PointCloud2::Ptr points_ptr(new sensor_msgs::PointCloud2);
  pcl::toROSMsg(in_pts, *points_ptr);

  if ( st_x < 0 ) st_x = 0;
  if ( st_y < 0 ) st_y = 0;
  if ( ed_x >= points_ptr->width ) ed_x = points_ptr->width -1;
  if ( ed_y >= points_ptr->height ) ed_y = points_ptr->height -1;

  int wd = points_ptr->width;
  int ht = points_ptr->height;
  int rstep = points_ptr->row_step;
  int pstep = points_ptr->point_step;

  out_pts.header = points_ptr->header;
  out_pts.width = ed_x - st_x + 1;
  out_pts.height = ed_y - st_y + 1;
  out_pts.row_step = out_pts.width * pstep;
  out_pts.point_step = pstep;
  out_pts.is_bigendian = false;
  out_pts.fields = points_ptr->fields;
  out_pts.is_dense = false;
  out_pts.data.resize(out_pts.row_step * out_pts.height);

  unsigned char * dst_ptr = &(out_pts.data[0]);

  for (size_t idx_y = st_y; idx_y <= ed_y; idx_y++) {
    for (size_t idx_x = st_x; idx_x <= ed_x; idx_x++) {
      const unsigned char * src_ptr = &(points_ptr->data[idx_y * rstep + idx_x * pstep]);
      memcpy(dst_ptr, src_ptr, pstep);
      dst_ptr += pstep;
    }
  }
}

} // namespace jsk_pcl_ros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointcloudScreenpoint, nodelet::Nodelet)
