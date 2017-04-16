#include <iostream>
// using namespace std;
#include <ros/ros.h>
#include <ros/console.h>

// ROS msgs
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <jsk_pcl_ros/PointsArray.h> //

// PCL related
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>

ros::Publisher pub_voxel; // voxel cloud, pub_voxel
ros::Publisher pub_plane; // plane cloud, pub_planelane
ros::Publisher pub_rot; // rotated cloud, pub_rot
ros::Publisher pub_red; // reduced cloud, pub_red
// How to avoid hard-coding a topic name?
ros::Publisher pub_marker;
// ros::Publisher pub_pca; // pca arrow
// ros::Publisher pub_text; // text
// ros::Publisher pub_center;

// For research, refering to existing equation,
// explain how parameter value is defined.

// Use nodelet to devide this process into threads

pcl::PointCloud<pcl::PointXYZ>::Ptr divide(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_rot,
                                           double x_min, double x_max, double y_min, double y_max,
                                           double z_min, double z_max) {
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_xyz_rot_vector;
  cloud_xyz_rot_vector = cloud_xyz_rot->points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduced_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr =
         cloud_xyz_rot_vector.begin(); itr != cloud_xyz_rot_vector.end(); ++itr) {
    if (x_min < itr->x && itr->x < x_max) { // 1.5~1.75 or 1.75~2.00: 1.5~1.675
      if (y_min < itr->y && itr->y < y_max) {
        if (z_min < itr->z && itr->z < z_max) {
          pcl::PointXYZ p;
          p.x = itr->x; p.y = itr->y; p.z = itr->z;
          cloud_reduced_xyz->points.push_back(p);
        }
      }
    }
  } return cloud_reduced_xyz;
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& input) {
  // std::cerr << "in cloud_cb" << std::endl;
  /* 0. Importing input cloud */
  pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; // initialize object
  pcl_conversions::toPCL(*input, *cloud); // from input, generate content of cloud
  // reduce cloud above z > 3, width also should be narrowed

  /* 1. Downsampling and Publishing voxel */
  // LeafSize: should small enough to caputure a leaf of plants
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud); // imutable pointer
  pcl::PCLPointCloud2 cloud_voxel; // cloud_filtered to cloud_voxel
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

  sor.setInputCloud(cloudPtr); // set input
  sor.setLeafSize(0.02f, 0.02f, 0.02f); // 2cm, model equation
  sor.filter(cloud_voxel); // set output

  sensor_msgs::PointCloud2 output_voxel;
  pcl_conversions::fromPCL(cloud_voxel, output_voxel);
  pub_voxel.publish(output_voxel);

  /* 2. Filtering with RANSAC */
  // RANSAC initialization
  pcl::SACSegmentation<pcl::PointXYZ> seg; // Create the segmentation object
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  seg.setOptimizeCoefficients(true); // Optional
  seg.setModelType(pcl::SACMODEL_PLANE); // Mandatory
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations (1000); // added
  seg.setDistanceThreshold(0.05); // default: 0.02 // 閾値（しきい値）

  // convert from PointCloud2 to PointXYZ
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_voxel_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(cloud_voxel, *cloud_voxel_xyz);

  // RANSAC application
  seg.setInputCloud(cloud_voxel_xyz);
  seg.segment(*inliers, *coefficients);

  // inliers.indices have array index of the points which are included as inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  for (std::vector<int>::const_iterator pit = inliers->indices.begin ();
       pit != inliers->indices.end (); pit++) {
    cloud_plane_xyz->points.push_back (cloud_voxel_xyz->points[*pit]);
  }
  cloud_plane_xyz->width = cloud_plane_xyz->points.size ();
  cloud_plane_xyz->height = 1;

  // Conversions: PointCloud<T>, PCLPointCloud2, sensor_msgs::PointCloud2
  pcl::PCLPointCloud2 out_p;
  pcl::toPCLPointCloud2(*cloud_plane_xyz, out_p);
  sensor_msgs::PointCloud2 output_plane;
  pcl_conversions::fromPCL(out_p, output_plane);
  output_plane.header.frame_id = "odom";
  output_plane.header.stamp = ros::Time::now();
  pub_plane.publish(output_plane);

  /* 3. PCA application to get eigen */
  pcl::PCA<pcl::PointXYZ> pca;
  pca.setInputCloud(cloud_plane_xyz);
  Eigen::Matrix3f eigen_vectors = pca.getEigenVectors(); // 3x3
  // std::cerr << eigen_vectors(0,0) << ", " // error output
  //           << eigen_vectors(0,1) << ", "
  //           << eigen_vectors(0,2) << std::endl;

  /* 4. PCA Visualization */
  visualization_msgs::Marker points;
  points.header.frame_id = "/odom";
  points.header.stamp = ros::Time::now();
  points.ns = "pca"; // namespace + id
  points.id = 0; // pca/0
  points.action = visualization_msgs::Marker::ADD;
  points.type = visualization_msgs::Marker::ARROW;

  points.pose.orientation.w = 1.0;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.scale.z = 0.05;
  points.color.g = 0.25f;
  points.color.r = 0.25f;
  points.color.b = 1.0f;
  points.color.a = 1.0;

  geometry_msgs::Point p_0, p_1;
  p_0.x = 0; p_0.y = 0; p_0.z = 0;
  p_1.x = eigen_vectors(0,0);
  p_1.y = eigen_vectors(0,1);
  p_1.z = eigen_vectors(0,2);
  points.points.push_back(p_0);
  points.points.push_back(p_1);
  pub_marker.publish(points);

  /* 5. Point Cloud Rotation  */
  eigen_vectors(0,2) = 0; // ignore very small z-value
  double norm = pow((pow(eigen_vectors(0,0), 2) + pow(eigen_vectors(0,1), 2)), 0.5);
  double nx = eigen_vectors(0,0) / norm;
  double ny = eigen_vectors(0,1) / norm;

  Eigen::Matrix4d rot_z; // rotation inversed, convert to Matrix4f
  rot_z(0,0) = nx; rot_z(0,1) = ny; rot_z(0,2) = 0; rot_z(0,3) = 0;
  rot_z(1,0) = -ny; rot_z(1,1) = nx; rot_z(1,2) = 0; rot_z(1,3) = 0;
  rot_z(2,0) = 0; rot_z(2,1) = 0; rot_z(2,2) = 1; rot_z(2,3) = 0;
  rot_z(3,0) = 0; rot_z(3,1) = 0; rot_z(3,2) = 0; rot_z(3,3) = 1;
  // std::cerr << rot_z(0,0) << ", " << rot_z(0,1) << ", " // error output
  //           << rot_z(0,2) << ", " << rot_z(0,3) << std::endl;
  // std::cerr << rot_z(1,0) << ", " << rot_z(1,1) << ", "
  //           << rot_z(1,2) << ", " << rot_z(1,3) << std::endl;
  // std::cerr << rot_z(2,0) << ", " << rot_z(2,1) << ", "
  //           << rot_z(2,2) << ", " << rot_z(2,3) << std::endl;
  // std::cerr << rot_z(3,0) << ", " << rot_z(3,1) << ", "
  //           << rot_z(3,2) << ", " << rot_z(3,3) << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz_rot (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(*cloudPtr, *cloud_xyz); // from PointCloud2 to PointXYZ
  // Transformation: Rotation, Translation
  // original, transformed, transformation matrix
  pcl::transformPointCloud(*cloud_xyz, *cloud_xyz_rot, rot_z);

  pcl::PCLPointCloud2 out_rot;
  sensor_msgs::PointCloud2 output_rot;
  pcl::toPCLPointCloud2(*cloud_xyz_rot, out_rot);
  pcl_conversions::fromPCL(out_rot, output_rot);
  pub_rot.publish(output_rot);

  /* 6. Point Cloud Reduction */
  // iterator should be shorten
  // std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_xyz_rot_vector;
  // cloud_xyz_rot_vector = cloud_xyz_rot->points;
  // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduced_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  // double y_mean, y_mean_old;
  // for (std::vector<int>::const_iterator pitr = cloud_xyz_rot->points.begin();
  //      pitr != cloud_xyz_rot->points.end(); pitr++) {
  //   pcl::PointXYZ p = cloud_xyz_rot->points[*pitr]; // tmp var
  //   if (1.50 < p.x && p.x < 1.675) { // 1.5~1.75 or 1.75~2.00, need 3 or more ranges
  //     if (-0.675 < p.y && p.y < 0.675) {
  //       if (-0.3125 < p.z && p.z < 2.0) {
  //         cloud_reduced_xyz->points.push_back(p);
  //       }
  //     }
  //   }
  // }

  // A. Divide by range of x value : function returning cloud_reduced_xyz (cloud, x_min, x_max)
  // std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> > cloud_xyz_rot_vector;
  // cloud_xyz_rot_vector = cloud_xyz_rot->points;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_reduced_xyz (new pcl::PointCloud<pcl::PointXYZ>);
  cloud_reduced_xyz = divide(cloud_xyz_rot, 1.50, 1.675, -0.675, 0.675, -0.3125, 2.0); // o(n) = n

  // double y_mean, y_mean_old;
  // int i = 0; // loop counter
  // for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr =
  //        cloud_xyz_rot_vector.begin(); itr != cloud_xyz_rot_vector.end(); ++itr) {
  //   if (1.50 < itr->x && itr->x < 1.675) { // 1.5~1.75 or 1.75~2.00, need 3 or more ranges
  //     if (-0.675 < itr->y && itr->y < 0.675) {
  //       if (-0.3125 < itr->z && itr->z < 2.0) {
  //         // std::cerr << "interator position = " << *itr << std::endl;
  //         pcl::PointXYZ p;
  //         p.x = itr->x; p.y = itr->y; p.z = itr->z;
  //         cloud_reduced_xyz->points.push_back(p);
  //         // if (i == 0) {
  //         //   y_mean_old = itr->y;
  //         // } else {
  //         //   y_mean = ((i - 1) * y_mean_old + itr->y) / i; // really need this?
  //         // }
  //       }
  //     }
  //   } i++;
  // } std::cerr << "mean value of y = " << y_mean << std::endl;

  // 関数化しよう return marker with cumulative ID
  double width_min = 2.0; // initialize with a constant
  double width_stitch = 4.0;
  geometry_msgs::Point p_s, p_e, p_m; // p_l, p_r;
  std::vector<geometry_msgs::Point> p_vector;
  // o(n) = n^2
  for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr_1 =
         cloud_reduced_xyz->points.begin(); itr_1 != cloud_reduced_xyz->points.end(); ++itr_1) {
    if (itr_1->y < 0) {
      for (std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ> >::const_iterator itr_2 =
             cloud_reduced_xyz->points.begin(); itr_2 != cloud_reduced_xyz->points.end(); ++itr_2) {
        if (0 <= itr_2->y) {
          double tmp;
          tmp = sqrt(pow(fabs(itr_1->x - itr_2->x), 2)
                     + pow(fabs(itr_1->y - itr_2->y), 2)
                     + pow(fabs(itr_1->z - itr_2->z), 2));
          if (tmp <= width_min) {
            width_min = tmp;
            p_s.x = itr_1->x; p_s.y = itr_1->y; p_s.z = itr_1->z;
            p_e.x = itr_2->x; p_e.y = itr_2->y; p_e.z = itr_2->z;
            p_m.x = itr_1->x; // ignore adding sqrt
            p_m.y = itr_1->y + sqrt(pow(fabs(itr_1->y - itr_2->y), 2)) / 2;
            p_m.z = itr_1->z; // ignore adding sqrt
          } else if (-0.3125 < itr_1->z && itr_1->z < 0.325 && tmp <= width_stitch) {
            width_stitch = tmp;
          }
        }
      }
    }
  } // id=itr
  p_vector.push_back(p_s);
  p_vector.push_back(p_e);
  p_vector.push_back(p_m);

  pcl::PCLPointCloud2 out_red;
  pcl::toPCLPointCloud2(*cloud_reduced_xyz, out_red);
  sensor_msgs::PointCloud2 output_red;
  pcl_conversions::fromPCL(out_red, output_red);
  output_red.header.frame_id = "odom";
  output_red.header.stamp = ros::Time::now();
  pub_red.publish(output_red);
  std::cerr << "width_min = " << width_min << std::endl
            << "width_stitch = " << width_stitch << std::endl
            << "point inbetween = "  << std::endl
            << "(" << p_s.x << ", " << p_s.y << ", " << p_s.z << ")" << std::endl
            << "(" << p_e.x << ", " << p_e.y << ", " << p_e.z << ")" << std::endl
            << "(" << p_m.x << ", " << p_m.y << ", " << p_m.z << ")" << std::endl;

  visualization_msgs::Marker texts; // TEXT_VIEW_FACING
  texts.header.frame_id = "/odom";
  texts.header.stamp = ros::Time::now();
  texts.ns = "text"; // namespace + ID
  texts.id = 0;
  texts.action = visualization_msgs::Marker::ADD;
  texts.type = visualization_msgs::Marker::TEXT_VIEW_FACING;

  texts.pose.position.x = p_m.x;
  texts.pose.position.y = p_m.y;
  texts.pose.position.z = 1.00;
  texts.pose.orientation.x = 0.0;
  texts.pose.orientation.y = 0.0;
  texts.pose.orientation.z = 0.0;
  texts.pose.orientation.w = 1.0;

  texts.scale.x = 0.2;
  texts.scale.y = 0.2;
  texts.scale.z = 0.2;
  texts.color.r = 1.0f;
  texts.color.g = 1.0f;
  texts.color.b = 1.0f;
  texts.color.a = 1.0;

  // setText
  std::ostringstream strs; strs << width_min;
  std::string str = strs.str();
  texts.text = str;
  pub_marker.publish(texts);

  /* 6. Visualize center line */
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "/odom";
  line_strip.header.stamp = ros::Time::now();
  line_strip.ns = "center";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 0; // set id

  line_strip.scale.x = 0.05;
  line_strip.color.r = 1.0f;
  line_strip.color.g = 0.0f;
  line_strip.color.b = 0.0f;
  line_strip.color.a = 1.0;

  // geometry_msgs::Point p_stitch, p_min;
  p_s.x = 0; p_s.y = 0; p_s.z = 0;
  p_e.x = p_m.x; p_e.y = p_m.y; p_e.z = 0;
  line_strip.points.push_back(p_s);
  line_strip.points.push_back(p_e);
  pub_marker.publish(line_strip);

  /* PCA Visualization */
  // geometry_msgs::Pose pose; tf::poseEigenToMsg(pca.getEigenVectors, pose);
  /* to use Pose marker in rviz */
  /* Automatic Measurement */
  // 0-a. stitch measurement: -0.5 < z < -0.3
  // 0-b. min width measurement: 0.3 < z < 5m
  // 1. iterate
  // 2. pick point if y < 0
  // 3. compare point with all points if 0 < y
  // 4. pick point-pare recording shortest distance
  // 5. compare the point with previous point
  // 6. update min
  // 7. display value in text in between 2 points
}

int main (int argc, char** argv) {
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  // assemble_cloud -> cloud_pcd
  std::cerr << "my_pcl_tutorial started." << std::endl;
  ros::Subscriber sub = nh.subscribe("cloud_pcd", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub_plane = nh.advertise<sensor_msgs::PointCloud2>("plane", 1);
  pub_voxel = nh.advertise<sensor_msgs::PointCloud2>("voxel", 1);
  pub_rot = nh.advertise<sensor_msgs::PointCloud2>("cloud_rotated", 1);
  pub_red = nh.advertise<sensor_msgs::PointCloud2>("cloud_reduced", 1);
  pub_marker = nh.advertise<visualization_msgs::Marker>("marker", 1, 0);
  // pub_text = nh.advertise<visualization_msgs::Marker>("texts", 1, 0);
  // pub_center = nh.advertise<visualization_msgs::Marker>("center", 1, 0);

  // Spin
  ros::spin();
}
