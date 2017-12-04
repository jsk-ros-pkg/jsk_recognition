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

#include <jsk_topic_tools/log_utils.h>
#include "jsk_pcl_ros/pointcloud_screenpoint.h"
#include <pcl_conversions/pcl_conversions.h>

// F/K/A <ray converter>
// Haseru Chen, Kei Okada, Yohei Kakiuchi

void jsk_pcl_ros::PointcloudScreenpoint::onInit()
{
  PCLNodelet::onInit();

  queue_size_ = 1;
  crop_size_ = 10;
  k_ = 16; // todo : this value should be set from parameter.

  pnh_->param ("queue_size", queue_size_, 1);
  pnh_->param ("crop_size", crop_size_, 10);
  pnh_->param ("search_size", k_, 16);

  pnh_->param ("use_rect", use_rect_, false);
  pnh_->param ("use_point", use_point_, false);
  pnh_->param ("use_sync", use_sync_, false);
  pnh_->param ("use_point_array", use_point_array_, false);
  pnh_->param ("use_poly", use_poly_, false);
  pnh_->param("publish_points", publish_points_, false);
  pnh_->param("publish_point", publish_point_, false);

  srv_ = pnh_->advertiseService("screen_to_point", &PointcloudScreenpoint::screenpoint_cb, this);

  if (publish_point_) {
    pub_point_ = pnh_->advertise< geometry_msgs::PointStamped > ("output_point", 1);
    ROS_INFO("Publish output as geometry_msgs::PointStamped to '%s'", pub_point_.getTopic().c_str());
  }

  if (publish_points_) {
    pub_points_ = pnh_->advertise< sensor_msgs::PointCloud2 > ("output", 1);
    ROS_INFO("Publish output as sensor_msgs::PointCloud2 to '%s'", pub_points_.getTopic().c_str());
  }

  pub_polygon_ = pnh_->advertise<geometry_msgs::PolygonStamped>("output_polygon", 1);
  ROS_INFO("Publish output as geometry_msgs::PolygonStamped to '%s'", pub_polygon_.getTopic().c_str());

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
  normals_tree_ = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
  n3d_.setKSearch (k_);
  n3d_.setSearchMethod (normals_tree_);

  points_sub_.subscribe (*pnh_, "points", queue_size_);
  ROS_INFO("Subscribe '%s' of sensor_msgs::PointCloud2", points_sub_.getTopic().c_str());

  if (use_rect_) {
    rect_sub_.subscribe   (*pnh_, "rect", queue_size_);
    ROS_INFO("Subscribe '%s' of geometry_msgs::PolygonStamped", rect_sub_.getTopic().c_str());
    if (use_sync_) {
      sync_a_rect_ = boost::make_shared < message_filters::Synchronizer< PolygonApproxSyncPolicy > > (queue_size_);
      sync_a_rect_->connectInput (points_sub_, rect_sub_);
      sync_a_rect_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_rect, this, _1, _2));
    } else {
      rect_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::rect_cb, this, _1));
    }
  }
  
  if (use_poly_) {
    poly_sub_.subscribe   (*pnh_, "poly", queue_size_);
    ROS_INFO("Subscribe '%s' of geometry_msgs::PolygonStamped", poly_sub_.getTopic().c_str());
    if (use_sync_) {
      sync_a_poly_ = boost::make_shared < message_filters::Synchronizer< PolygonApproxSyncPolicy > > (queue_size_);
      sync_a_poly_->connectInput (points_sub_, rect_sub_);
      sync_a_poly_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_poly, this, _1, _2));
    } else {
      poly_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::poly_cb, this, _1));
    }
  }

  if (use_point_) {
    point_sub_.subscribe  (*pnh_, "point", queue_size_);
    ROS_INFO("Subscribe '%s' of geometry_msgs::PointStamped", point_sub_.getTopic().c_str());
    if (use_sync_) {
      sync_a_point_ = boost::make_shared < message_filters::Synchronizer< PointApproxSyncPolicy > > (queue_size_);
      sync_a_point_->connectInput (points_sub_, point_sub_);
      sync_a_point_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_point, this, _1, _2));
    } else {
      point_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::point_cb, this, _1));
    }
  }

  if (use_point_array_) {
    point_array_sub_.subscribe(*pnh_, "point_array", queue_size_);
    ROS_INFO("Subscribe '%s' of sensor_msgs::PointCloud2", point_array_sub_.getTopic().c_str());
    if (use_sync_) {
      sync_a_point_array_ = boost::make_shared < message_filters::Synchronizer< PointCloudApproxSyncPolicy > > (queue_size_);
      sync_a_point_array_->connectInput (points_sub_, point_array_sub_);
      sync_a_point_array_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_point_array, this, _1, _2));
    } else {
      point_array_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::point_array_cb, this, _1));
    }
  }

  points_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::points_cb, this, _1));
}


bool jsk_pcl_ros::PointcloudScreenpoint::checkpoint (pcl::PointCloud< pcl::PointXYZ > &in_pts, int x, int y,
                                                     float &resx, float &resy, float &resz)  {
  if ((x < 0) || (y < 0) || (x >= in_pts.width) || (y >= in_pts.height)) {
    ROS_WARN("Requested point is out of image size.  point: (%d, %d)  size: (%d, %d)", x, y, in_pts.width, in_pts.height);
    return false;
  }
  pcl::PointXYZ p = in_pts.points[in_pts.width * y + x];
  // search near points
  ROS_INFO("Request: screenpoint (%d, %d) => (%f, %f, %f)", x, y, p.x, p.y, p.z);
  //return !(isnan (p.x) || ( (p.x == 0.0) && (p.y == 0.0) && (p.z == 0.0)));

  if ( !std::isnan (p.x) && ((p.x != 0.0) || (p.y != 0.0) || (p.z == 0.0)) ) {
    resx = p.x; resy = p.y; resz = p.z;
    return true;
  }
  return false;
}

bool jsk_pcl_ros::PointcloudScreenpoint::extract_point (pcl::PointCloud< pcl::PointXYZ > &in_pts, int reqx, int reqy,
                                                        float &resx, float &resy, float &resz) {
  int x, y;

  x = reqx < 0.0 ? ceil(reqx - 0.5) : floor(reqx + 0.5);
  y = reqy < 0.0 ? ceil(reqy - 0.5) : floor(reqy + 0.5);
  ROS_INFO("Request : %d %d", x, y);

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

bool jsk_pcl_ros::PointcloudScreenpoint::screenpoint_cb (jsk_recognition_msgs::TransformScreenpoint::Request &req,
                                                         jsk_recognition_msgs::TransformScreenpoint::Response &res)
{
  ROS_DEBUG("PointcloudScreenpoint::screenpoint_cb");
  boost::mutex::scoped_lock lock(this->mutex_callback_);
  if ( pts_.points.size() == 0 ) {
    ROS_ERROR("no point cloud was received");
    return false;
  }

  res.header = header_;

  bool ret;
  float rx, ry, rz;
  ret = extract_point (pts_, req.x, req.y, rx, ry, rz);
  res.point.x = rx; res.point.y = ry; res.point.z = rz;

  if (!ret) {
    return false;
  }

  // search normal
  n3d_.setSearchSurface(boost::make_shared<pcl::PointCloud<pcl::PointXYZ > > (pts_));

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

  if (publish_point_) {
    geometry_msgs::PointStamped ps;
    ps.header = header_;
    ps.point.x = res.point.x;
    ps.point.y = res.point.y;
    ps.point.z = res.point.z;
    pub_point_.publish(ps);
  }

  return true;
}

void jsk_pcl_ros::PointcloudScreenpoint::points_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {
  ROS_DEBUG("PointcloudScreenpoint::points_cb, width=%d, height=%d, fields=%d", msg->width, msg->height, msg->fields.size());
  //boost::mutex::scoped_lock lock(this->mutex_callback_);
  header_ = msg->header;
  pcl::fromROSMsg (*msg, pts_);
}

void jsk_pcl_ros::PointcloudScreenpoint::extract_rect (const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                                                       int st_x, int st_y, int ed_x, int ed_y) {
  if ( st_x < 0 ) st_x = 0;
  if ( st_y < 0 ) st_y = 0;
  if ( ed_x >= points_ptr->width ) ed_x = points_ptr->width -1;
  if ( ed_y >= points_ptr->height ) ed_y = points_ptr->height -1;

  int wd = points_ptr->width;
  int ht = points_ptr->height;
  int rstep = points_ptr->row_step;
  int pstep = points_ptr->point_step;

  sensor_msgs::PointCloud2 pt;
  pt.header = points_ptr->header;
  pt.width = ed_x - st_x + 1;
  pt.height = ed_y - st_y + 1;
  pt.row_step = pt.width * pstep;
  pt.point_step = pstep;
  pt.is_bigendian = false;
  pt.fields = points_ptr->fields;
  pt.is_dense = false;
  pt.data.resize(pt.row_step * pt.height);

  unsigned char * dst_ptr = &(pt.data[0]);

  for (size_t idx_y = st_y; idx_y <= ed_y; idx_y++) {
    for (size_t idx_x = st_x; idx_x <= ed_x; idx_x++) {
      const unsigned char * src_ptr = &(points_ptr->data[idx_y * rstep + idx_x * pstep]);
      memcpy(dst_ptr, src_ptr, pstep);
      dst_ptr += pstep;
    }
  }

  pub_points_.publish (pt);
}

void jsk_pcl_ros::PointcloudScreenpoint::point_cb (const geometry_msgs::PointStampedConstPtr& pt_ptr) {
  if (publish_point_) {
    geometry_msgs::PointStamped ps;
    bool ret; float rx, ry, rz;
    ret = extract_point (pts_, pt_ptr->point.x, pt_ptr->point.y, rx, ry, rz);

    if (ret) {
      ps.point.x = rx; ps.point.y = ry; ps.point.z = rz;
      ps.header = header_;
      pub_point_.publish (ps);
    }
  }
}

void jsk_pcl_ros::PointcloudScreenpoint::callback_point (const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                                                         const geometry_msgs::PointStampedConstPtr& pt_ptr) {
  point_cb (pt_ptr);

  if (publish_points_) {
    int st_x = pt_ptr->point.x - crop_size_;
    int st_y = pt_ptr->point.y - crop_size_;
    int ed_x = pt_ptr->point.x + crop_size_;
    int ed_y = pt_ptr->point.y + crop_size_;

    extract_rect (points_ptr, st_x, st_y, ed_x, ed_y);
  }
}

void jsk_pcl_ros::PointcloudScreenpoint::point_array_cb (const sensor_msgs::PointCloud2ConstPtr& pt_arr_ptr) {
  if (publish_points_) {
    pcl::PointCloud<pcl::PointXY>::Ptr point_array_cloud(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromROSMsg(*pt_arr_ptr, *point_array_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    result_cloud->header = pcl_conversions::toPCL(header_); // if hydro
    for (size_t i = 0; i < point_array_cloud->points.size(); i++) {
      pcl::PointXY point = point_array_cloud->points[i];
      geometry_msgs::PointStamped ps;
      bool ret; float rx, ry, rz;
      ret = extract_point (pts_, point.x, point.y, rx, ry, rz);
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
    ros_cloud->header = header_;
    pub_points_.publish(ros_cloud);
  }
}
void jsk_pcl_ros::PointcloudScreenpoint::callback_point_array (const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                                                               const sensor_msgs::PointCloud2ConstPtr& pt_arr_ptr) {
  point_array_cb(pt_arr_ptr);
}

void jsk_pcl_ros::PointcloudScreenpoint::rect_cb (const geometry_msgs::PolygonStampedConstPtr& array_ptr) {
  if (array_ptr->polygon.points.size() > 1) {
    int st_x = array_ptr->polygon.points[0].x;
    int st_y = array_ptr->polygon.points[0].y;
    int ed_x = array_ptr->polygon.points[1].x;
    int ed_y = array_ptr->polygon.points[1].y;
    if (publish_point_) {
      geometry_msgs::PointStamped ps;
      bool ret; float rx, ry, rz;
      ret = extract_point (pts_, (st_x + ed_x) / 2, (st_y + ed_y) / 2, rx, ry, rz);

      if (ret) {
        ps.point.x = rx; ps.point.y = ry; ps.point.z = rz;
        ps.header = header_;
        pub_point_.publish (ps);
      }
    }
  }
}

void jsk_pcl_ros::PointcloudScreenpoint::poly_cb(const geometry_msgs::PolygonStampedConstPtr& array_ptr) {
  // publish polygon
  geometry_msgs::PolygonStamped result_polygon;
  result_polygon.header = header_;
  for (size_t i = 0; i < array_ptr->polygon.points.size(); i++) {
    geometry_msgs::Point32 p = array_ptr->polygon.points[i];
    float rx, ry, rz;
    bool ret = extract_point (pts_, p.x, p.y, rx, ry, rz);
    if (!ret) {
      NODELET_ERROR("Failed to project point");
      return;
    }
    geometry_msgs::Point32 p_projected;
    p_projected.x = rx;
    p_projected.y = ry;
    p_projected.z = rz;
    result_polygon.polygon.points.push_back(p_projected);
  }
  pub_polygon_.publish(result_polygon);
}

void jsk_pcl_ros::PointcloudScreenpoint::callback_poly(const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                                                       const geometry_msgs::PolygonStampedConstPtr& array_ptr) {
  poly_cb(array_ptr);
}



void jsk_pcl_ros::PointcloudScreenpoint::callback_rect(const sensor_msgs::PointCloud2ConstPtr& points_ptr,
                                                       const geometry_msgs::PolygonStampedConstPtr& array_ptr) {
  if (array_ptr->polygon.points.size() > 1) {
    int st_x = array_ptr->polygon.points[0].x;
    int st_y = array_ptr->polygon.points[0].y;
    int ed_x = array_ptr->polygon.points[1].x;
    int ed_y = array_ptr->polygon.points[1].y;

    rect_cb (array_ptr);

    if (publish_points_) {
      extract_rect (points_ptr, st_x, st_y, ed_x, ed_y);
    }
  }
}

PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::PointcloudScreenpoint, nodelet::Nodelet);
