#include "jsk_pcl_ros/pointcloud_screenpoint.h"

// F/K/A <ray converter>
// Haseru Chen, Kei Okada, Yohei Kakiuchi

void jsk_pcl_ros::PointcloudScreenpoint::onInit()
{
  NODELET_INFO("[%s::onInit]", getName().c_str());
  PCLNodelet::onInit();
  
  sub_ = pnh_->subscribe("points", 1, &PointcloudScreenpoint::points_cb, this);
  srv_ = pnh_->advertiseService("screen_to_point", &PointcloudScreenpoint::screenpoint_cb, this);
  
  int k_ = 16; // todo : this value should be set from parameter.
  
  normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZ> > ();
  n3d_.setKSearch (k_);
  n3d_.setSearchMethod (normals_tree_);
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
  int x, y;
  res.header = header_;
  x = req.x < 0.0 ? ceil(req.x - 0.5) : floor(req.x + 0.5);
  y = req.y < 0.0 ? ceil(req.y - 0.5) : floor(req.y + 0.5);
  ROS_WARN("Request : %d %d", x, y);

  pcl::PointXYZ p = pts.points[pts.width*y + x];
  ROS_INFO_STREAM("Request: screenpoint ("<<x<<","<<y<<")="<<"(" << p.x << ", " <<p.y << ", " <<p.z <<")");
  if ( isnan(p.x) || ( (p.x == 0.0) && (p.y == 0.0) && (p.z == 0.0)) ) {
    // try
    {
      bool point_found = false; int radius=10, step_size = 1;
      for (int y2=y-radius; y2<=y+radius; y2+=step_size)
	{
	  for (int x2=x-radius; x2<=x+radius; x2+=step_size)
	    {
	      pcl::PointXYZ p2 = pts.points[pts.width*y2 + x2];
	      ROS_INFO_STREAM("Request: lookfor new screenpoint ("<<x2<<","<<y2<<")="<<"(" << p2.x << ", " <<p2.y << ", " <<p2.z <<")");
	      if ( isnan(p2.x) || ( (p2.x == 0.0) && (p2.y == 0.0) && (p2.z == 0.0)) ) {
		continue;
	      }
	      res.point.x = p2.x;
	      res.point.y = p2.y;
	      res.point.z = p2.z;
	      point_found = true;
	      break;
	    }
	  if(point_found) break;
	}
      if(!point_found)  return false;
    }
  } else {
    res.point.x = p.x;
    res.point.y = p.y;
    res.point.z = p.z;
  }

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

  if((res.point.x * res.vector.x + res.point.y * res.vector.y + res.point.z * res.vector.z) < 0)
    {
      res.vector.x *= -1;
      res.vector.y *= -1;
      res.vector.z *= -1;
    }
  return true;
}

void jsk_pcl_ros::PointcloudScreenpoint::points_cb(const sensor_msgs::PointCloud2ConstPtr &msg) {
  ROS_DEBUG("PointcloudScreenpoint::points_cb");
  boost::mutex::scoped_lock lock(this->mutex_callback_);
  header_ = msg->header;
  pcl::fromROSMsg (*msg, pts);
}

typedef jsk_pcl_ros::PointcloudScreenpoint PointcloudScreenpoint;
PLUGINLIB_DECLARE_CLASS (jsk_pcl, PointcloudScreenpoint, PointcloudScreenpoint, nodelet::Nodelet);
