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

#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros_utils/pointcloud_to_stl.h"

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/TextureMesh.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/mls.h>
#include <pcl/io/obj_io.h>
#include <pcl/filters/filter.h>

#include "jsk_recognition_utils/pcl_conversion_util.h"

namespace jsk_pcl_ros_utils
{
  void PointCloudToSTL::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud);
    exportSTL(cloud);


    visualization_msgs::Marker marker_msg;
    marker_msg.header = input->header;
    marker_msg.mesh_resource = std::string("package://jsk_pcl_ros_utils/temp.stl");//file_name_;//latest_output_path_;
    marker_msg.ns = "pcl_mesh_reconstrunction";
    marker_msg.id = 0;
    marker_msg.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker_msg.action = visualization_msgs::Marker::ADD;

    marker_msg.pose.position.x = 1;
    marker_msg.pose.position.y = 1;
    marker_msg.pose.position.z = 1;
    marker_msg.pose.orientation.x = 0.0;
    marker_msg.pose.orientation.y = 0.0;
    marker_msg.pose.orientation.z = 0.0;
    marker_msg.pose.orientation.w = 1.0;
    marker_msg.scale.x = 1;
    marker_msg.scale.y = 1;
    marker_msg.scale.z = 1;
    pub_mesh_.publish(marker_msg);

  }

  void PointCloudToSTL::exportSTL(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &input_cloud){

    pcl::PolygonMesh triangles;
    ofm.setInputCloud (input_cloud);
    ofm.reconstruct (triangles);

    /*


    // Store the results in a temporary object
    boost::shared_ptr<std::vector<pcl::Vertices> > temp_verts (new std::vector<pcl::Vertices>);
    ofm.reconstruct (*temp_verts);


    ROS_INFO("Got Input PointClouds");
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*input_cloud, *cloud, indices);

    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr mls_tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
 
    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (mls_tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (*mls_points);

    mls_points->is_dense = false;
    pcl::removeNaNFromPointCloud(*mls_points, *mls_points, indices);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (mls_points);//cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (search_radius_);

    // Set typical values for the parameters
    gp3.setMu (mu_);
    gp3.setMaximumNearestNeighbors (maximum_nearest_neighbors_);
    gp3.setMaximumSurfaceAngle(maximum_surface_angle_); // 45 degrees
    gp3.setMinimumAngle(minimum_angle_); // 10 degrees
    gp3.setMaximumAngle(maximum_angle_); // 120 degrees
    gp3.setNormalConsistency(normal_consistency_);

    // Get result
    gp3.setInputCloud (mls_points);//cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

*/

    ros::Time now_time = ros::Time::now();
    std::stringstream ss;
    if (file_name_.length())
      ss << file_name_.c_str();
    else
      ss << "/tmp/" << now_time.toNSec() << "_pointcloud.stl";
    
    ROS_INFO("Writing... %s", ss.str().c_str());
    pcl::io::savePolygonFileSTL(ss.str(),triangles);
    latest_output_path_ = ss.str();

    /*
    std::vector<std::string> tex_files;
    tex_files.push_back("text4.jpg");

    // initialize texture mesh
    pcl::TextureMesh tex_mesh;
    tex_mesh.cloud = triangles.cloud;

    // add the 1st mesh
    tex_mesh.tex_polygons.push_back(triangles.polygons);

    // update mesh and texture mesh
    //gp3.updateMesh(cloud_with_normals, triangles, tex_mesh);
    // set texture for added cloud
    tex_files.push_back("tex8.jpg");
    // save updated mesh

    pcl::TextureMapping<pcl::PointXYZ> tm;

    tm.setF(0.01);
    tm.setVectorField(1, 0, 0);

    pcl::TexMaterial tex_material;

    tex_material.tex_Ka.r = 0.2f;
    tex_material.tex_Ka.g = 0.2f;
    tex_material.tex_Ka.b = 0.2f;

    tex_material.tex_Kd.r = 0.8f;
    tex_material.tex_Kd.g = 0.8f;
    tex_material.tex_Kd.b = 0.8f;

    tex_material.tex_Ks.r = 1.0f;
    tex_material.tex_Ks.g = 1.0f;
    tex_material.tex_Ks.b = 1.0f;
    tex_material.tex_d = 1.0f;
    tex_material.tex_Ns = 0.0f;
    tex_material.tex_illum = 2;

    tm.setTextureMaterials(tex_material);
    tm.setTextureFiles(tex_files);
    tm.mapTexture2Mesh(tex_mesh);

    pcl::io::saveOBJFile ("/tmp/test.obj", tex_mesh);
    */
  }

  bool PointCloudToSTL::createSTL(jsk_recognition_msgs::SetPointCloud2::Request &req,
                                  jsk_recognition_msgs::SetPointCloud2::Response &res)
  {
    if(req.name.length())
      file_name_ = req.name;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(req.cloud, *cloud);
    exportSTL(cloud);
    res.output = latest_output_path_;
  }

  void PointCloudToSTL::onInit(void)
  {
    PCLNodelet::onInit();
    pnh_->param("filename", file_name_ , std::string(""));
    pnh_->param("search_radius", search_radius_ , 0.05);
    pnh_->param("mu", mu_, 3.5);
    pnh_->param("maximum_nearest_neighbors", maximum_nearest_neighbors_, 100);
    pnh_->param("maximum_surface_angle", maximum_surface_angle_, 0.7853981633974483);
    pnh_->param("minimum_angle", minimum_angle_, 0.17453292519943295);
    pnh_->param("maximum_angle", maximum_angle_, 2.0943951023931953);
    pnh_->param("normal_consistency", normal_consistency_, false);

    pnh_->param("triangle_pixel_size", triangle_pixel_size_, 1.0);
    pnh_->param("max_edge_length", max_edge_length_, 4.5);
    pnh_->param("store_shadow_faces", store_shadow_faces_, true);

    sub_input_ = pnh_->subscribe("input", 1, &PointCloudToSTL::cloudCallback, this);
    create_stl_srv_
      = pnh_->advertiseService("create_stl", &PointCloudToSTL::createSTL, this);

    pub_mesh_ = pnh_->advertise<visualization_msgs::Marker>("pc_stl_mesh", 1);

    ofm.setTrianglePixelSize (triangle_pixel_size_);
    ofm.setTriangulationType (pcl::OrganizedFastMesh<pcl::PointXYZRGB>::TRIANGLE_ADAPTIVE_CUT);
    ofm.setMaxEdgeLength(max_edge_length_);
    ofm.storeShadowedFaces(store_shadow_faces_);
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros_utils::PointCloudToSTL, nodelet::Nodelet);
