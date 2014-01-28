#include "jsk_pcl_ros/pointcloud_screenpoint.h"
#include <pcl_conversions/pcl_conversions.h>

// F/K/A <ray converter>
// Haseru Chen, Kei Okada, Yohei Kakiuchi

void jsk_pcl_ros::PointcloudScreenpoint::onInit()
{
  NODELET_INFO("[%s::onInit]", getName().c_str());

  //pnh_.reset (new ros::NodeHandle (getMTPrivateNodeHandle ()));
  pnh_.reset (new ros::NodeHandle (getPrivateNodeHandle ()));

  queue_size_ = 1;
  crop_size_ = 10;
  k_ = 16; // todo : this value should be set from parameter.

  pnh_->param ("queue_size", queue_size_, 1);
  pnh_->param ("crop_size", crop_size_, 10);
  pnh_->param ("search_size", k_, 16);

  bool use_rect, use_point, use_sync, use_point_array;

  pnh_->param ("use_rect", use_rect, false);
  pnh_->param ("use_point", use_point, false);
  pnh_->param ("use_sync", use_sync, false);
  pnh_->param ("use_point_array", use_point_array, false);

  pnh_->param("publish_points", publish_points_, false);
  pnh_->param("publish_point", publish_point_, false);

  points_sub_.subscribe (*pnh_, "points", queue_size_);

  if (use_rect) {
    rect_sub_.subscribe   (*pnh_, "rect", queue_size_);
    if (use_sync) {
      sync_a_polygon_ = boost::make_shared < message_filters::Synchronizer< PolygonApproxSyncPolicy > > (queue_size_);
      sync_a_polygon_->connectInput (points_sub_, rect_sub_);
      sync_a_polygon_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_polygon, this, _1, _2));
    } else {
      rect_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::rect_cb, this, _1));
    }
  }

  if (use_point) {
    point_sub_.subscribe  (*pnh_, "point", queue_size_);
    if (use_sync) {
      sync_a_point_ = boost::make_shared < message_filters::Synchronizer< PointApproxSyncPolicy > > (queue_size_);
      sync_a_point_->connectInput (points_sub_, point_sub_);
      sync_a_point_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_point, this, _1, _2));
    } else {
      point_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::point_cb, this, _1));
    }
  }

  if (use_point_array) {
    point_array_sub_.subscribe(*pnh_, "point_array", queue_size_);
    if (use_sync) {
      sync_a_point_array_ = boost::make_shared < message_filters::Synchronizer< PointCloudApproxSyncPolicy > > (queue_size_);
      sync_a_point_array_->connectInput (points_sub_, point_array_sub_);
      sync_a_point_array_->registerCallback (boost::bind (&PointcloudScreenpoint::callback_point_array, this, _1, _2));
    } else {
      point_array_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::point_array_cb, this, _1));
    }
  }

  points_sub_.registerCallback (boost::bind (&PointcloudScreenpoint::points_cb, this, _1));

  srv_ = pnh_->advertiseService("screen_to_point", &PointcloudScreenpoint::screenpoint_cb, this);

  if (publish_point_) {
    pub_point_ = pnh_->advertise< geometry_msgs::PointStamped > ("output_point", 1);
  }

  if (publish_points_) {
    pub_points_ = pnh_->advertise< sensor_msgs::PointCloud2 > ("output", 1);
  }

#if ( PCL_MAJOR_VERSION >= 1 && PCL_MINOR_VERSION >= 5 )
  normals_tree_ = boost::make_shared< pcl::search::KdTree<pcl::PointXYZ> > ();
#else
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
#endif
  n3d_.setKSearch (k_);
  n3d_.setSearchMethod (normals_tree_);
}

bool jsk_pcl_ros::PointcloudScreenpoint::checkpoint (pcl::PointCloud< pcl::PointXYZ > &in_pts, int x, int y,
                                                     float &resx, float &resy, float &resz)  {
  if ((x < 0) || (y < 0) || (x >= in_pts.width) || (y >= in_pts.height)) return false;
  pcl::PointXYZ p = in_pts.points[in_pts.width * y + x];
  // search near points
  ROS_INFO_STREAM("Request: screenpoint ("<<x<<","<<y<<")="<<"(" << p.x << ", " <<p.y << ", " <<p.z <<")");
  //return !(isnan (p.x) || ( (p.x == 0.0) && (p.y == 0.0) && (p.z == 0.0)));

  if ( !isnan (p.x) && ((p.x != 0.0) || (p.y != 0.0) || (p.z == 0.0)) ) {
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
  ROS_WARN("Request : %d %d", x, y);

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

bool jsk_pcl_ros::PointcloudScreenpoint::screenpoint_cb (jsk_pcl_ros::TransformScreenpoint::Request &req,
                                                         jsk_pcl_ros::TransformScreenpoint::Response &res)
{
  ROS_DEBUG("PointcloudScreenpoint::screenpoint_cb");
  boost::mutex::scoped_lock lock(this->mutex_callback_);
  if ( pts.points.size() == 0 ) {
    ROS_ERROR("no point cloud was received");
    return false;
  }

  res.header = header_;

  bool ret;
  float rx, ry, rz;
  ret = extract_point (pts, req.x, req.y, rx, ry, rz);
  res.point.x = rx; res.point.y = ry; res.point.z = rz;

  if (!ret) {
    return false;
  }

  // search normal
  n3d_.setSearchSurface(boost::make_shared<pcl::PointCloud<pcl::PointXYZ > > (pts));

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
  //ROS_DEBUG("PointcloudScreenpoint::points_cb");
  //boost::mutex::scoped_lock lock(this->mutex_callback_);
  header_ = msg->header;
  pcl::fromROSMsg (*msg, pts);
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
    ret = extract_point (pts, pt_ptr->point.x, pt_ptr->point.y, rx, ry, rz);

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
      ret = extract_point (pts, point.x, point.y, rx, ry, rz);
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
      ret = extract_point (pts, (st_x + ed_x) / 2, (st_y + ed_y) / 2, rx, ry, rz);

      if (ret) {
        ps.point.x = rx; ps.point.y = ry; ps.point.z = rz;
        ps.header = header_;
        pub_point_.publish (ps);
      }
    }
  }
}

void jsk_pcl_ros::PointcloudScreenpoint::callback_polygon(const sensor_msgs::PointCloud2ConstPtr& points_ptr,
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

typedef jsk_pcl_ros::PointcloudScreenpoint PointcloudScreenpoint;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, PointcloudScreenpoint, PointcloudScreenpoint, nodelet::Nodelet);
