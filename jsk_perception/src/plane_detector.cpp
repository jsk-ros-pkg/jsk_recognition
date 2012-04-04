#include "ros/ros.h"

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_ros/publisher.h>
#include <pcl_ros/transforms.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf/transform_broadcaster.h>
#include "pcl/surface/convex_hull.h"

typedef pcl::PointXYZRGB Point;
typedef pcl::KdTree<Point>::Ptr KdTreePtr;

class PlaneDetector
{

public:
  PlaneDetector();
  virtual ~PlaneDetector() { };
  void pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void pcCallback2(const sensor_msgs::PointCloud2ConstPtr &msg);
  void planeEstimate(pcl::PointCloud<Point>::ConstPtr,
                     pcl::PointCloud<pcl::Normal>::ConstPtr);
  void planeEstimate(pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr);
  void polygonEstimate(pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr);
  pcl::PointCloud<pcl::Normal>::ConstPtr normalEstimate
  (pcl::PointCloud<Point>::ConstPtr);

protected:
  ros::NodeHandle _nh;
  ros::NodeHandle _private_nh;
  ros::Subscriber _pc_sub;
  ros::Subscriber _pc_sub2;
  ros::Publisher _plane_pub;
  pcl_ros::Publisher<Point> _pc_pub;
  //flag
  bool _use_normal;

  // for plane estimation
  pcl::PointIndices _plane_inliers;
  pcl::ModelCoefficients _plane_coefficients;
  tf::Transform _plane_trans;

  // for normal estimation
  pcl::NormalEstimation<Point, pcl::Normal> _n3d;
  pcl::PointCloud<pcl::Normal> _cloud_normals;

  // parameters
  double _distance_thr;
  int _num_iteration;
  int _max_plane_num;
  int _min_plane_size;
  double _normal_distance_weight;
  double _probability;
  double _plane_surface_threshold;
  std::string _plane_frame_id;
};

PlaneDetector::PlaneDetector() : _nh(), _private_nh("~")
{
  // resolve parameter
  _private_nh.param<bool>("use_normal", _use_normal, false);
  _private_nh.param<double>("distance_thr", _distance_thr, 0.03);
  _private_nh.param<double>("probability", _probability, 0.99);
  _private_nh.param<double>("normal_distance_weight", _normal_distance_weight, 0.1);
  _private_nh.param<double>("plane_surface_threshold", _plane_surface_threshold, 0.0);
  _private_nh.param<int>("num_iteration", _num_iteration, 1000);
  _private_nh.param<int>("max_plane_num", _max_plane_num, 5);
  _private_nh.param<int>("min_plane_size", _min_plane_size, 100);
  _private_nh.param<std::string>("plane_frame_id", _plane_frame_id, std::string("/plane"));

  _pc_sub = _nh.subscribe("points_in", 10,
                          &PlaneDetector::pcCallback, this);
  _pc_sub2 = _nh.subscribe("points_in2", 10,
                           &PlaneDetector::pcCallback2, this);
  _pc_pub.advertise(_nh, "points_out", 1);
  _plane_pub = _nh.advertise<geometry_msgs::PolygonStamped>("estimated_plane", 10);
  _n3d.setKSearch(10);
}

/*copied from tabletop_object_detector */
/*! Assumes plane coefficients are of the form ax+by+cz+d=0, normalized */
tf::Transform getPlaneTransform (pcl::ModelCoefficients coeffs,
                                 double up_direction)
{
  ROS_ASSERT(coeffs.values.size() > 3);
  double a = coeffs.values[0], b = coeffs.values[1],
      c = coeffs.values[2], d = coeffs.values[3];
  //asume plane coefficients are normalized
  btVector3 position(-a*d, -b*d, -c*d);
  btVector3 z(a, b, c);
  //make sure z points "up"
  ROS_DEBUG("z.dot: %0.3f", z.dot(btVector3(0,0,1)));
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);
  if ( z.dot( btVector3(0, 0, up_direction) ) < 0)
  {
    z = -1.0 * z;
  }
  ROS_DEBUG("in getPlaneTransform, z: %0.3f, %0.3f, %0.3f", z[0], z[1], z[2]);

  //try to align the x axis with the x axis of the original frame
  //or the y axis if z and x are too close too each other
  btVector3 x(1, 0, 0);
  if ( fabs(z.dot(x)) > 1.0 - 1.0e-4) x = btVector3(0, 1, 0);
  btVector3 y = z.cross(x).normalized();
  x = y.cross(z).normalized();

  btMatrix3x3 rotation;
  rotation[0] = x; // x
  rotation[1] = y; // y
  rotation[2] = z; // z
  rotation = rotation.transpose();
  btQuaternion orientation;
  rotation.getRotation(orientation);
  return tf::Transform(orientation, position);
}

void PlaneDetector::planeEstimate(pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr)
{
  pcl::SACSegmentation<Point> seg;
  seg.setDistanceThreshold (_distance_thr);
  seg.setMaxIterations (_num_iteration);
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  //seg.setMethodType (pcl::SAC_LMEDS);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setProbability (_probability);

  seg.setInputCloud(pcl_cloud_ptr);
  seg.segment(_plane_inliers, _plane_coefficients);
}

void PlaneDetector::polygonEstimate(pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr)
{
  // project the table inliners on the table
  pcl::PointCloud<Point> plane_projected;
  pcl::ProjectInliers<Point> proj;
  proj.setModelType (pcl::SACMODEL_PLANE);
  proj.setInputCloud (pcl_cloud_ptr);

  pcl::PointIndices::ConstPtr plane_inliers_ptr
    = boost::make_shared<const pcl::PointIndices> (_plane_inliers);
  proj.setIndices(plane_inliers_ptr);
  pcl::ModelCoefficients::ConstPtr plane_coefficients_ptr =
    boost::make_shared<const pcl::ModelCoefficients> (_plane_coefficients);
  proj.setModelCoefficients (plane_coefficients_ptr);
  proj.filter (plane_projected);

  pcl::PointCloud<Point>::ConstPtr plane_projected_ptr =
    boost::make_shared<const pcl::PointCloud<Point> > (plane_projected);

  pcl::ConvexHull<Point> hull;
  // estimate the convex hull
  pcl::PointCloud<Point> plane_hull;
  hull.setInputCloud (plane_projected_ptr);
  hull.reconstruct (plane_hull);

  // transform point cloud...
  pcl::PointCloud<Point> transed_hull;
  tf::Transform inv_plane_trans = _plane_trans.inverse();

  pcl_ros::transformPointCloud<Point>(plane_hull, transed_hull, inv_plane_trans);

  if ( plane_hull.points.size() >= 3)
    {
      geometry_msgs::PolygonStamped poly;
      poly.header.stamp = pcl_cloud_ptr->header.stamp;
      poly.header.frame_id =  _plane_frame_id;
      // here, i check the direction of points
      // polygon must have CLOCKWISE direction
      tf::Vector3 O(plane_hull.points[1].x,
                    plane_hull.points[1].y,
                    plane_hull.points[1].z);
      tf::Vector3 B(plane_hull.points[0].x,
                    plane_hull.points[0].y,
                    plane_hull.points[0].z);
      tf::Vector3 A(plane_hull.points[2].x,
                    plane_hull.points[2].y,
                    plane_hull.points[2].z);
      tf::Vector3 OA = A-O;
      tf::Vector3 OB = B-O;
      tf::Vector3 N = OA.cross(OB);
      double theta = N.angle(O);
      bool reversed = false;
      if ( theta < M_PI / 2.0)
        reversed = true;
      for ( size_t i = 0; i < transed_hull.points.size(); i++ )
        {
          geometry_msgs::Point32 p;
          if ( reversed )
            {
              size_t j = transed_hull.points.size() - i - 1;
              p.x = transed_hull.points[j].x;
              p.y = transed_hull.points[j].y;
              p.z = transed_hull.points[j].z;
            }
          else
            {
              p.x = transed_hull.points[i].x;
              p.y = transed_hull.points[i].y;
              p.z = transed_hull.points[i].z;
            }
          poly.polygon.points.push_back(p);
        }

      _plane_pub.publish(poly);
    }
  else
    {
      ROS_FATAL("too small points are estimated");
    }
}


void PlaneDetector::planeEstimate(pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr,
                                  pcl::PointCloud<pcl::Normal>::ConstPtr cloud_normals_ptr)
{
  pcl::SACSegmentationFromNormals<Point, pcl::Normal> seg;
  seg.setDistanceThreshold (_distance_thr);
  seg.setMaxIterations (_num_iteration);
  seg.setNormalDistanceWeight (_normal_distance_weight);
  seg.setOptimizeCoefficients (true);
  //seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  //seg.setMethodType (pcl::SAC_LMEDS);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setProbability (_probability);

  seg.setInputCloud(pcl_cloud_ptr);
  seg.setInputNormals(cloud_normals_ptr);
  seg.segment(_plane_inliers, _plane_coefficients);
}

pcl::PointCloud<pcl::Normal>::ConstPtr PlaneDetector::normalEstimate
(pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr)
{
  KdTreePtr normals_tree = boost::make_shared<pcl::KdTreeFLANN<Point> > ();
  _n3d.setSearchMethod (normals_tree);
  _n3d.setInputCloud (pcl_cloud_ptr);
  _n3d.compute (_cloud_normals);
  /* _cloud_normals_ptr = 
     boost::make_shared<const pcl::PointCloud<pcl::Normal> > (_cloud_normals);
  */
  return boost::make_shared<const pcl::PointCloud<pcl::Normal> >
    (_cloud_normals);
}

void PlaneDetector::pcCallback2(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  sensor_msgs::PointCloud2 outbuf;
  // sensor_msgs::PointCloud -> sensor_msgs::PointCloud2
  //sensor_msgs::convertPointCloudToPointCloud2 (*msg, outbuf);
  pcl::PointCloud<Point> pcl_cloud;

  //remove points which couldn't get coordinates
  pcl::PointCloud<Point> pcl_cloud_before_passthroughfilter;
  // sensor_msgs::PointCloud2 -> pcl::PointCloud
  pcl::fromROSMsg(*msg, pcl_cloud_before_passthroughfilter);
  pcl::PassThrough<Point> pass;
  pass.setInputCloud (boost::make_shared<pcl::PointCloud<Point> > (pcl_cloud_before_passthroughfilter));
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.01);
  pass.setFilterLimitsNegative(true);
  pass.filter (pcl_cloud);

  pcl::PointCloud<Point>::Ptr pcl_cloud_ptr =
    boost::make_shared< pcl::PointCloud<Point> > (pcl_cloud);

  int counter = 0;
  while(counter < _max_plane_num) {
    //ROS_INFO("points:  %d", pcl_cloud_ptr->points.size());
    if (_use_normal)
      {
        //normal estimate 25Hz
        pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud_ptr =
          normalEstimate(pcl_cloud_ptr);
        planeEstimate(pcl_cloud_ptr, normal_cloud_ptr);
      }
    else
      {
        // plane estimate, 44Hz
        planeEstimate(pcl_cloud_ptr);
      }

    pcl::ExtractIndices<Point> extract;
    extract.setInputCloud
      (boost::make_shared<pcl::PointCloud<Point> > (*pcl_cloud_ptr));
    extract.setIndices (boost::make_shared<pcl::PointIndices> (_plane_inliers));

    if (_plane_coefficients.values.size () >=3 )
      {
        //_plane_trans = getPlaneTransform(_plane_coefficients, -1);
        extract.setNegative (false);
        pcl::PointCloud<Point> cloud_p;
        extract.filter (cloud_p); // on plane

        _pc_pub.publish(cloud_p);

        if (pcl_cloud_ptr->points.size() == _plane_inliers.indices.size()) {
          break;
        }

        pcl::PointCloud<Point> next_p;
        extract.setNegative (true);
        extract.filter (next_p);

        if (next_p.points.size() < _min_plane_size) break;
        pcl_cloud_ptr =
          boost::make_shared< pcl::PointCloud<Point> > (next_p);
      }
    else
      {
        break;
      }
    counter++;
  }
}

void PlaneDetector::pcCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  sensor_msgs::PointCloud2 outbuf;
  // sensor_msgs::PointCloud -> sensor_msgs::PointCloud2
  //sensor_msgs::convertPointCloudToPointCloud2 (*msg, outbuf);
  pcl::PointCloud<Point> pcl_cloud;

  //remove points which couldn't get coordinates
  pcl::PointCloud<Point> pcl_cloud_before_passthroughfilter;
  // sensor_msgs::PointCloud2 -> pcl::PointCloud
  pcl::fromROSMsg(*msg, pcl_cloud_before_passthroughfilter);
  pcl::PassThrough<Point> pass;
  pass.setInputCloud (boost::make_shared<pcl::PointCloud<Point> > (pcl_cloud_before_passthroughfilter));
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-0.01, 0.01);
  pass.setFilterLimitsNegative(true);
  pass.filter (pcl_cloud);

  pcl::PointCloud<Point>::ConstPtr pcl_cloud_ptr =
    boost::make_shared<const pcl::PointCloud<Point> > (pcl_cloud);

  if (_use_normal)
    {
      //normal estimate 25Hz
      pcl::PointCloud<pcl::Normal>::ConstPtr normal_cloud_ptr =
        normalEstimate(pcl_cloud_ptr);
      planeEstimate(pcl_cloud_ptr, normal_cloud_ptr);
    }
  else
    {
      // plane estimate, 44Hz
      planeEstimate(pcl_cloud_ptr);
    }

  pcl::ExtractIndices<Point> extract;
  extract.setInputCloud(boost::make_shared<pcl::PointCloud<Point> > (pcl_cloud));
  extract.setIndices (boost::make_shared<pcl::PointIndices> (_plane_inliers));
  if (_plane_coefficients.values.size () >=3 )
    {
      // calc plane transform
      _plane_trans = getPlaneTransform(_plane_coefficients, -1);
      tf::Transform plane_trans_inv = _plane_trans.inverse();
      static tf::TransformBroadcaster br;
      br.sendTransform(tf::StampedTransform(_plane_trans,
                                            pcl_cloud_ptr->header.stamp,
                                            pcl_cloud_ptr->header.frame_id,
                                            _plane_frame_id));
      extract.setNegative (true);
      pcl::PointCloud<Point> cloud_p;
      extract.filter (cloud_p);
      /*pcl::PointIndices::ConstPtr table_inliers_ptr =
        boost::make_shared<const pcl::PointIndices> (table_inliers);
        pcl::ModelCoefficients::ConstPtr table_coefficients_ptr = 
        boost::make_shared<const pcl::ModelCoefficients> (table_coefficients);*/

      //FIXME: we should make another method...
      //here we remove points under estimated plane
      // first of all,
      pcl::PointCloud<Point> on_plane_cloud;

      // SENSOR_FRAME_ID -> PLANE
      pcl_ros::transformPointCloud<Point>(pcl_cloud, //cloud_p,
                                          on_plane_cloud,
                                          plane_trans_inv);
      pcl::PassThrough<Point> pass_z;
      pass_z.setInputCloud (boost::make_shared<pcl::PointCloud<Point> >(on_plane_cloud));
      pass_z.setFilterFieldName ("z");
      pass_z.setFilterLimits (_plane_surface_threshold, FLT_MAX); // add limitation on extraction
      pass_z.setFilterLimitsNegative (false);
      pcl::PointCloud<Point> on_plane_cloud_filtered;
      pass_z.filter(on_plane_cloud_filtered);

      pcl::PointCloud<Point> sensor_frame_cloud_filtered;
      // PLANE ->  SENSOR_FRAME_ID
      pcl_ros::transformPointCloud<Point>(on_plane_cloud_filtered,
                                          sensor_frame_cloud_filtered,
                                          _plane_trans);
      polygonEstimate(pcl_cloud_ptr);
      _pc_pub.publish(sensor_frame_cloud_filtered);
    }
  else
    {
      ROS_WARN("missed plane estimation");
      _pc_pub.publish(pcl_cloud);
    }
}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "plane_detector");
  PlaneDetector p;
  ros::spin();
  return 0;
}
