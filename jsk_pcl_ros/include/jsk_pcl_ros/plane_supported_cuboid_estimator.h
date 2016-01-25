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


#ifndef JSK_PCL_ROS_PLANE_SUPPORTED_CUBOID_ESTIMATOR_H_
#define JSK_PCL_ROS_PLANE_SUPPORTED_CUBOID_ESTIMATOR_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <boost/random.hpp>
#include <dynamic_reconfigure/server.h>
#include "jsk_recognition_utils/geo_util.h"
#include "jsk_recognition_utils/pcl_conversion_util.h"
#include "jsk_pcl_ros/tf_listener_singleton.h"
#include <algorithm>


// ROS messages
#include <jsk_recognition_msgs/PolygonArray.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <jsk_recognition_msgs/ModelCoefficientsArray.h>
#include <jsk_recognition_msgs/HistogramWithRange.h>
#include <jsk_pcl_ros/PlaneSupportedCuboidEstimatorConfig.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

// pcl
#include <pcl/tracking/tracking.h>
#include "jsk_pcl_ros/pcl/simple_particle_filter.h"
#include "jsk_pcl_ros/pcl/particle_cuboid.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace jsk_pcl_ros
{

  // Utility function to compute likelihood
  /**
   * @brief
   * return 1.0 if v satisfies min < v < max, return 0 otherwise.
   */
  inline double binaryLikelihood(double v, double min, double max)
  {
    if (v < min) {
      return 0;
    }
    else if (v > max) {
      return 0;
    }
    else {
      return 1;
    }
  }
  
  template <class Config>
  double rangeLikelihood(const pcl::tracking::ParticleCuboid& p,
                         pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                         const std::vector<Polygon::Ptr>& planes,
                         const Config& config)
  {
    double likelihood = 1.0;
    Polygon::Ptr plane = planes[p.plane_index];
    if (p.plane_index == -1) {
      // Do nothing
    }
    else {
      Eigen::Vector3f projected_point;
      plane->project(Eigen::Vector3f(p.getVector3fMap()), projected_point);
      if (plane->isInside(projected_point)) {
        likelihood *= 1.0;
      }
      else {
        likelihood = 0.0;
      }
    }
    float local_z = plane->distanceToPoint(Eigen::Vector3f(p.getVector3fMap()));
    likelihood *= binaryLikelihood(local_z, config.range_likelihood_local_min_z, config.range_likelihood_local_max_z);
    return likelihood;
  }

  template <class Config>
  double supportPlaneAngularLikelihood(
    const pcl::tracking::ParticleCuboid& p,
    const std::vector<Polygon::Ptr>& planes,
    const Config& config)
  {
    Polygon::Ptr plane = planes[p.plane_index];
    if (config.use_support_plane_angular_likelihood) {
      // double cos_likelihood = (p.toEigenMatrix().rotation() * Eigen::Vector3f::UnitZ()).dot(plane->getNormal());
      double cos_likelihood = (p.toEigenMatrix().rotation() * Eigen::Vector3f::UnitX()).dot(plane->getNormal());
      // ROS_INFO("cos_likelihood: %f", cos_likelihood);
      return pow(std::abs(cos_likelihood),
                 config.support_plane_angular_likelihood_weight_power);
    }
    else {
      return 1.0;
    }
  }


  template <class Config>
  double surfaceAreaLikelihood(
    const pcl::tracking::ParticleCuboid& p,
    const Config& config)
  {
    if (config.use_surface_area_likelihood) {
      double v = p.area();
      return 1.0 / (1.0 + pow(v, config.surface_area_error_power));
    }
    else {
      return 1.0;
    }
  }

  template <class Config>
  double distanceFromPlaneBasedError(
    const pcl::tracking::ParticleCuboid& p,
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
    pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
    const Eigen::Vector3f& viewpoint,
    const Config& config)
  {
    Eigen::Affine3f pose = p.toEigenMatrix();
    Eigen::Affine3f pose_inv = pose.inverse();
    const Eigen::Vector3f local_vp = pose_inv * viewpoint;
    std::vector<int> visible_faces = p.visibleFaceIndices(local_vp);
    double r = sqrt(p.dx /2 * p.dx/2 + p.dy/2 * p.dy/2 + p.dz/2 * p.dz/2);
    std::vector<int> candidate_point_indices;
    std::vector<float> candidate_point_distances;
    pcl::PointXYZ xyz_point;
    xyz_point.getVector3fMap() = p.getVector3fMap();
    size_t inliers = 0;
    // roughly search near points by kdtree radius search
    tree.radiusSearch(xyz_point, r + config.outlier_distance, candidate_point_indices, candidate_point_distances);
    if (candidate_point_indices.size() < config.min_inliers) {
      return 0;
    }
    else {
      //ROS_INFO("indices: %lu", candidate_point_indices.size());
      double error = 0.0;
      
      jsk_recognition_utils::Cube::Ptr cube = p.toCube();
      std::vector<Polygon::Ptr> faces = cube->faces();
      for (size_t i = 0; i < candidate_point_indices.size(); i++) {
        int index = candidate_point_indices[i];
        Eigen::Vector3f v = cloud->points[index].getVector3fMap();
        if (config.use_occlusion_likelihood) {
          double d = p.distanceNearestToPlaneWithOcclusion(v, visible_faces, faces);
          if (d <= config.outlier_distance) {
            //error *= 1 / (1 + pow(d, config.plane_distance_error_power));
            error += pow(d, config.plane_distance_error_power);
            ++inliers;
          }
        }
        else {
          Eigen::Vector3f local_v = pose_inv * v;
          double d = p.signedDistanceToPlane(local_v, p.nearestPlaneIndex(local_v));
          if (config.use_inside_points_distance_zero) {
            if (d < 0) {
              d = 0;
            }
          }
          else {
            d = std::abs(d);
          }
          if (d <= config.outlier_distance) {
            //error *= 1 / (1 + pow(d, config.plane_distance_error_power));
            error += pow(d, config.plane_distance_error_power);
            ++inliers;
          }
        }
      }
      // ROS_INFO("inliers: %lu", inliers);
      // ROS_INFO("error: %f", error);
      size_t expected_num = p.volume() / config.expected_density / config.expected_density / config.expected_density;
      // ROS_INFO("expected: %lu", expected_num);
      if (inliers < config.min_inliers) {
        return 0;
      }
      else {
        double non_inlier_value = 1 / (1 + error / inliers);
        if (config.use_inliers) {
          // how to compute expected inliers...?
          // double inliers_err = std::abs(p.volume() / config.expected_density - inliers);
          // return non_inlier_value * (1 / (1 + pow(inliers_err, config.inliers_power)));
          double inlier_likelihood = 1 / (1 + pow(expected_num - inliers, config.inliers_power));
          // ROS_INFO("inlier likelihood: %f", inlier_likelihood);
          return non_inlier_value * inlier_likelihood;
        }
        else {
          return non_inlier_value;
        }
      }
    }
  }

  template <class Config>
  double planeLikelihood(const pcl::tracking::ParticleCuboid& p,
                         const std::vector<float>& polygon_likelihood,
                         const Config& config)
  {
    if (config.use_polygon_likelihood) {
      return polygon_likelihood[p.plane_index];
    }
    else {
      return 1.0;
    }
  }
  
  template <class Config>
  double computeLikelihood(const pcl::tracking::ParticleCuboid& p,
                           pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                           pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                           const Eigen::Vector3f& viewpoint,
                           const std::vector<Polygon::Ptr>& polygons,
                           const std::vector<float>& polygon_likelihood,
                           const Config& config)
  {
    double range_likelihood = 1.0;
    if (config.use_range_likelihood) {
      range_likelihood = rangeLikelihood(p, cloud, polygons, config);
    }
    //p.weight = std::abs(p.z);
    if (range_likelihood == 0.0) {
     return range_likelihood;
    }
    else {
      return (range_likelihood * distanceFromPlaneBasedError(p, cloud, tree, viewpoint, config)
              * supportPlaneAngularLikelihood(p, polygons, config)
              * surfaceAreaLikelihood(p, config)
              * planeLikelihood(p, polygon_likelihood, config));
    }    
  }
  
  class PlaneSupportedCuboidEstimator: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef pcl::tracking::ParticleCuboid Particle;
    typedef pcl::PointCloud<Particle> ParticleCloud;
    typedef boost::shared_ptr<PlaneSupportedCuboidEstimator> Ptr;
    typedef PlaneSupportedCuboidEstimatorConfig Config;
    typedef message_filters::sync_policies::ExactTime<
      jsk_recognition_msgs::PolygonArray,
      jsk_recognition_msgs::ModelCoefficientsArray> PolygonSyncPolicy;
    
    PlaneSupportedCuboidEstimator(): DiagnosticNodelet("PlaneSupportedCuboidEstimator") {}

  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void polygonCallback(
      const jsk_recognition_msgs::PolygonArray::ConstPtr& polygon_msg,
      const jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr& coef_msg);
    virtual void cloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void estimate(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual void fastCloudCallback(
      const sensor_msgs::PointCloud2::ConstPtr& msg);
    virtual pcl::PointCloud<pcl::tracking::ParticleCuboid>::Ptr initParticles();
    virtual size_t chooseUniformRandomPlaneIndex(const std::vector<Polygon::Ptr>& polygons);
    virtual void configCallback(Config& config, uint32_t level);
    
    // For particle filtering
    virtual pcl::tracking::ParticleCuboid sample(const pcl::tracking::ParticleCuboid& p);
    // Likelihood
    virtual void likelihood(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
                            pcl::tracking::ParticleCuboid& p);
    virtual void publishHistogram(ParticleCloud::Ptr particles, int index,
                                  ros::Publisher& pub,
                                  const std_msgs::Header& header);
    virtual bool resetCallback(std_srvs::EmptyRequest& req,
                               std_srvs::EmptyResponse& res);

    /**
     * @brief
     * Compute the nearest polygon to the particle `p'.
     */
    virtual size_t getNearestPolygon(
      const Particle& p,
      const std::vector<jsk_recognition_utils::ConvexPolygon::Ptr>& polygons);
    /**
     * @brief
     * Compute distances between particles and polygons and assing each particle
     * to the nearest polygon.
     * This method is called only if the "polygon sensor" measurement is updated.
     * For simplicity, this method expects convex polygon.
     */
    virtual void updateParticlePolygonRelationship(ParticleCloud::Ptr particles);
    boost::mutex mutex_;
    ros::Subscriber sub_cloud_;
    ros::Subscriber sub_fast_cloud_;
    ros::Publisher pub_result_;
    ros::Publisher pub_particles_;
    ros::Publisher pub_candidate_cloud_;
    ros::Publisher pub_histogram_global_x_;
    ros::Publisher pub_histogram_global_y_;
    ros::Publisher pub_histogram_global_z_;
    ros::Publisher pub_histogram_global_roll_;
    ros::Publisher pub_histogram_global_pitch_;
    ros::Publisher pub_histogram_global_yaw_;
    ros::Publisher pub_histogram_dx_;
    ros::Publisher pub_histogram_dy_;
    ros::Publisher pub_histogram_dz_;
    ros::Publisher pub_result_pose_;
    ros::ServiceServer srv_reset_;
    message_filters::Subscriber<jsk_recognition_msgs::PolygonArray> sub_polygon_;
    message_filters::Subscriber<jsk_recognition_msgs::ModelCoefficientsArray> sub_coefficients_;
    boost::shared_ptr<message_filters::Synchronizer<PolygonSyncPolicy> > sync_polygon_;
    jsk_recognition_msgs::PolygonArray::ConstPtr latest_polygon_msg_;
    jsk_recognition_msgs::ModelCoefficientsArray::ConstPtr latest_coefficients_msg_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud_;
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    // Parameters for initialization of particles
    // local position z is sampled by uniform distribution
    Config config_;
    double init_local_position_z_min_;
    double init_local_position_z_max_;
    bool use_init_world_position_z_model_;
    double init_world_position_z_min_;
    double init_world_position_z_max_;
    double init_local_orientation_roll_mean_;
    double init_local_orientation_roll_variance_;
    double init_local_orientation_pitch_mean_;
    double init_local_orientation_pitch_variance_;
    double init_local_orientation_yaw_mean_;
    double init_local_orientation_yaw_variance_;
    bool use_global_init_yaw_;
    double init_global_orientation_yaw_mean_;
    double init_global_orientation_yaw_variance_;

    bool disable_init_roll_;
    bool disable_init_pitch_;

    double init_dx_mean_;
    double init_dx_variance_;
    double init_dy_mean_;
    double init_dy_variance_;
    double init_dz_mean_;
    double init_dz_variance_;

    double step_x_variance_;
    double step_y_variance_;
    double step_z_variance_;
    double step_roll_variance_;
    double step_pitch_variance_;
    double step_yaw_variance_;
    double step_dx_variance_;
    double step_dy_variance_;
    double step_dz_variance_;

    double min_dx_;
    double min_dy_;
    double min_dz_;
    double fast_cloud_threshold_;
    bool use_init_polygon_likelihood_;
    int particle_num_;
    std::string sensor_frame_;
    boost::mt19937 random_generator_;
    tf::TransformListener* tf_;
    Eigen::Vector3f viewpoint_;
    bool support_plane_updated_;
    pcl::tracking::ROSCollaborativeParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleCuboid>::Ptr tracker_;
    std::vector<Polygon::Ptr> polygons_;
    pcl::KdTreeFLANN<pcl::PointXYZ> tree_;
  private:
  };
}

#endif
