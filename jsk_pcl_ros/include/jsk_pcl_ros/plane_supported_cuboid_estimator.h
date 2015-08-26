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
#include "jsk_pcl_ros/geo_util.h"
#include "jsk_pcl_ros/pcl_conversion_util.h"
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
#include <pcl/point_cloud.h>
#include "jsk_pcl_ros/ros_collaborative_particle_filter.h"
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
  namespace tracking
  {
    struct _ParticleCuboid
    {
      PCL_ADD_POINT4D;
      union
      {
        struct
        {
          float roll;
          float pitch;
          float yaw;
          float dx;
          float dy;
          float dz;
          float weight;
          
        };
        float data_c[8];
      };
      
      //jsk_pcl_ros::Polygon::Ptr plane;
      int plane_index;
    };
    
    struct EIGEN_ALIGN16 ParticleCuboid : public _ParticleCuboid
    {

      // Copy constructor
      inline ParticleCuboid(const ParticleCuboid& obj)
      {
        x = obj.x;
        y = obj.y;
        z = obj.z;
        roll = obj.roll;
        pitch = obj.pitch;
        yaw = obj.yaw;
        dx = obj.dx;
        dy = obj.dy;
        dz = obj.dz;
        weight = obj.weight;
        plane_index = obj.plane_index;
      }
      
      inline ParticleCuboid()
      {
        x = y = z = roll = pitch = yaw = dx = dy = dz = weight = 0.0;
        data[3] = 1.0f;
        plane_index = -1;
      }
      inline ParticleCuboid (float _x, float _y, float _z, float _roll, float _pitch, float _yaw)
      {
        x = _x; y = _y; z = _z;
        roll = _roll; pitch = _pitch; yaw = _yaw;
        dx = dy = dz = 0.0;
        data[3] = 1.0f;
      }

      inline Eigen::Affine3f toEigenMatrix() const
      {
        return getTransformation(x, y, z, roll, pitch, yaw);
      }

      inline void fromEigen(const Eigen::Affine3f& pose) 
      {
        Eigen::Vector3f pos(pose.translation());
        getEulerAngles(pose, roll, pitch, yaw);
        getVector3fMap() = pos;
      }

      inline void zero()
      {
        x = y = z = roll = pitch = yaw = dx = dy = dz = weight = 0.0;
      }
      
      inline float volume() const
      {
        return dx * dy * dz;
      }

      inline float area() const
      {
        return (dx * dy + dy * dz + dz * dx) * 2.0;
      }

      static pcl::tracking::ParticleCuboid
      toState (const Eigen::Affine3f &trans)
      {
        float trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw;
        getTranslationAndEulerAngles (trans,
                                      trans_x, trans_y, trans_z,
                                      trans_roll, trans_pitch, trans_yaw);
        return pcl::tracking::ParticleCuboid (trans_x, trans_y, trans_z, trans_roll, trans_pitch, trans_yaw);
      }

      // a[i]
      inline float operator [] (unsigned int i) const
      {
        switch (i)
        {
        case 0: return x;
        case 1: return y;
        case 2: return z;
        case 3: return roll;
        case 4: return pitch;
        case 5: return yaw;
        case 6: return dx;
        case 7: return dy;
        case 8: return dz;
        case 9: return weight;
        default: return 0.0;
        }
      }
      
      inline void sample(
        const std::vector<double>& mean, const std::vector<double>& cov)
      {
        // Do nothing because it is not used
      }

      inline static int stateDimension()
      {
        return 9;
      }

      inline jsk_recognition_msgs::BoundingBox toBoundingBox()
      {
        jsk_recognition_msgs::BoundingBox box;
        Eigen::Affine3f affine = toEigenMatrix();
        tf::poseEigenToMsg(affine, box.pose);
        box.dimensions.x = dx;
        box.dimensions.y = dy;
        box.dimensions.z = dz;
        return box;
      }

      inline std::vector<int> visibleFaceIndices(const Eigen::Vector3f local_view_point) const
      {
        std::vector<int> visible_faces;
        visible_faces.reserve(3);
        if (local_view_point[0] > 0) {
          visible_faces.push_back(0);
        }
        else if (local_view_point[0] < 0) {
          visible_faces.push_back(2);
        }
        if (local_view_point[1] > 0) {
          visible_faces.push_back(1);
        }
        else if (local_view_point[1] < 0) {
          visible_faces.push_back(3);
        }
        if (local_view_point[2] > 0) {
          visible_faces.push_back(4);
        }
        else if (local_view_point[2] < 0) {
          visible_faces.push_back(5);
        }
        return visible_faces;
      }

      inline double distanceNearestToPlaneWithOcclusion(
        const Eigen::Vector3f& v,
        const std::vector<int>& visible_faces) const
      {
        double min_distance = DBL_MAX;
        int nearest_plane_index = nearestPlaneIndex(v);
        // If the face is visible, use distance-to-plane
        if (std::find(visible_faces.begin(), visible_faces.end(),
                      nearest_plane_index)
            != visible_faces.end()) {
          return distanceToPlane(v, nearest_plane_index);
        }
        else {
          // If not the face is visible, use distance from centroid
          // It's an approximated distance.
          return (getVector3fMap() - v).norm();
        }
      }
      
      // V should be on local coordinates.
      // TODO: update to take into boundary account
      inline double distanceToPlane(const Eigen::Vector3f& v, const int plane_index) const
      {
        Eigen::Vector3f n;
        double d;
        switch (plane_index)
        {
        case 0: {
          n = Eigen::Vector3f::UnitX();
          d = - 0.5 * dx;
          break;
        }
        case 1: {
          n = Eigen::Vector3f::UnitY();
          d = - 0.5 * dy;
          break;
        }
        case 2: {
          n = - (Eigen::Vector3f::UnitX());
          d = - 0.5 * dx;
          break;
        }
        case 3: {
          n = - (Eigen::Vector3f::UnitY());
          d = - 0.5 * dy;
          break;
        }
        case 4: {
          n = Eigen::Vector3f::UnitZ();
          d = - 0.5 * dz;
          break;
        }
        case 5: {
          n = - (Eigen::Vector3f::UnitZ());
          d = - 0.5 * dz;
          break;
        }

        }
        return std::abs(n.dot(v) + d) / sqrt(n.dot(n) + d * d);
      }

      inline int nearestPlaneIndex(const Eigen::Vector3f& local_v) const
      {
        const float x = local_v[0];
        const float y = local_v[1];
        const float z = local_v[2];
        const float abs_x = std::abs(x);
        const float abs_y = std::abs(y);
        const float abs_z = std::abs(z);
        if (x > 0 && abs_x >= abs_y && abs_x >= abs_z) {
          return 0;
        }
        else if (x < 0 && abs_x >= abs_y && abs_x >= abs_z) {
          return 2;
        }
        else if (y > 0 && abs_y >= abs_x && abs_y >= abs_z) {
          return 1;
        }
        else if (y < 0 && abs_y >= abs_x && abs_y >= abs_z) {
          return 3;
        }
        else if (z > 0 && abs_z >= abs_x && abs_z >= abs_y) {
          return 4;
        }
        else if (z < 0 && abs_z >= abs_x && abs_z >= abs_y) {
          return 5;
        }
      }
      
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    inline ParticleCuboid operator* (const ParticleCuboid& p, const Eigen::Affine3f& transform)
    {
      Eigen::Affine3f particle_affine = p.toEigenMatrix();
      Eigen::Affine3f transformed_affine = transform * particle_affine;
      ParticleCuboid new_p;
      new_p.x = transformed_affine.translation()[0];
      new_p.y = transformed_affine.translation()[1];
      new_p.z = transformed_affine.translation()[2];
      getEulerAngles(transformed_affine, new_p.roll, new_p.pitch, new_p.yaw);
      new_p.dx = p.dx;
      new_p.dy = p.dy;
      new_p.dz = p.dz;
      return new_p;
    }

    // a + b
    inline pcl::tracking::ParticleCuboid operator+ (const ParticleCuboid& a, const ParticleCuboid& b)
    {
      pcl::tracking::ParticleCuboid newp;
      newp.x = a.x + b.x;
      newp.y = a.y + b.y;
      newp.z = a.z + b.z;
      newp.roll = a.roll + b.roll;
      newp.pitch = a.pitch + b.pitch;
      newp.yaw = a.yaw + b.yaw;
      newp.dx = a.dx + b.dx;
      newp.dy = a.dy + b.dy;
      newp.dz = a.dz + b.dz;
      return (newp);
    }
    
    inline pcl::tracking::ParticleCuboid operator * (const ParticleCuboid& p, double val)
    {
      pcl::tracking::ParticleCuboid newp;
      newp.x     = static_cast<float> (p.x * val);
      newp.y     = static_cast<float> (p.y * val);
      newp.z     = static_cast<float> (p.z * val);
      newp.roll  = static_cast<float> (p.roll * val);
      newp.pitch = static_cast<float> (p.pitch * val);
      newp.yaw   = static_cast<float> (p.yaw * val);
      newp.dx = static_cast<float> (p.dx * val);
      newp.dy = static_cast<float> (p.dy * val);
      newp.dz = static_cast<float> (p.dz * val);
      return (newp);
    }
    
    inline pcl::tracking::ParticleCuboid operator * (double val, const ParticleCuboid& p)
    {
      return p * val;
    }
    
    inline pcl::tracking::ParticleCuboid operator- (const ParticleCuboid& a, const ParticleCuboid& b)
    {
      return a + (-1.0) * b;
    }
    
  } // tracking

  
  template<>
  class DefaultPointRepresentation<pcl::tracking::ParticleCuboid>: public PointRepresentation<pcl::tracking::ParticleCuboid>
  {
  public:
    DefaultPointRepresentation()
    {
      nr_dimensions_ = 10;
    }

    virtual void
    copyToFloatArray (const pcl::tracking::ParticleCuboid &p, float * out) const
    {
      for (int i = 0; i < nr_dimensions_; ++i)
        out[i] = p[i];
    }
  };
  
} // pcl

// These registration is required to convert
// pcl::PointCloud<pcl::tracking::ParticleCuboid> to PCLPointCloud2.
// And pcl::fromROSMsg and pcl::toROSMsg depends on PCLPointCloud2
// conversions.
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::tracking::_ParticleCuboid,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, roll, roll)
                                   (float, pitch, pitch)
                                   (float, yaw, yaw)
                                   (float, dx, dx)
                                   (float, dy, dy)
                                   (float, dz, dz)
                                   (float, weight, weight)
  )
POINT_CLOUD_REGISTER_POINT_WRAPPER(pcl::tracking::ParticleCuboid, pcl::tracking::_ParticleCuboid)

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
      double cos_likelihood = (p.toEigenMatrix().rotation() * Eigen::Vector3f::UnitZ()).dot(plane->getNormal());
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
    double r = sqrt(p.dx * p.dx + p.dy * p.dy + p.dz * p.dz) / 2.0;
    std::vector<int> candidate_point_indices;
    std::vector<float> candidate_point_distances;
    pcl::PointXYZ xyz_point;
    xyz_point.getVector3fMap() = p.getVector3fMap();
    // roughly search near points by kdtree radius search
    tree.radiusSearch(xyz_point, r + config.outlier_distance, candidate_point_indices, candidate_point_distances);
    if (candidate_point_indices.size() < config.min_inliers) {
      return 0;
    }
    else {
      double error = 0.0;
      size_t inliers = 0;
      for (size_t i = 0; i < candidate_point_indices.size(); i++) {
        int index = candidate_point_indices[i];
        Eigen::Vector3f v = cloud->points[index].getVector3fMap();
        Eigen::Vector3f local_v = pose_inv * v;
        
        if (config.use_occlusion_likelihood) {
          double d = p.distanceNearestToPlaneWithOcclusion(local_v, visible_faces);
          if (d < config.outlier_distance) {
            //error *= 1 / (1 + pow(d, config.plane_distance_error_power));
            error += pow(d, config.plane_distance_error_power);
            ++inliers;
          }
        }
        else {
          double d = p.distanceToPlane(local_v, p.nearestPlaneIndex(local_v));
          if (d < config.outlier_distance) {
            //error *= 1 / (1 + pow(d, config.plane_distance_error_power));
            error += pow(d, config.plane_distance_error_power);
            ++inliers;
          }
        }
      }
      // ROS_INFO("inliers: %lu", inliers);
      if (inliers < config.min_inliers) {
        return 0;
      }
      else {
        return 1 / (1 + error / inliers);
      }
    }
  }

  
  template <class Config>
  double computeLikelihood(const pcl::tracking::ParticleCuboid& p,
                           pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,
                           pcl::KdTreeFLANN<pcl::PointXYZ>& tree,
                           const Eigen::Vector3f& viewpoint,
                           const std::vector<Polygon::Ptr>& polygons,
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
              * surfaceAreaLikelihood(p, config));
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
      const std::vector<ConvexPolygon::Ptr>& polygons);
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
    double init_local_orientation_roll_variance_;
    double init_local_orientation_pitch_variance_;
    
    double init_local_orientation_yaw_mean_;
    double init_local_orientation_yaw_variance_;

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
