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
      jsk_pcl_ros::Polygon::Ptr plane;
    };
    
    struct EIGEN_ALIGN16 ParticleCuboid : public _ParticleCuboid
    {
      inline ParticleCuboid()
      {
        x = y = z = roll = pitch = yaw = dx = dy = dz = weight = 0.0;
        data[3] = 1.0f;
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

      inline void zero()
      {
        x = y = z = roll = pitch = yaw = dx = dy = dz = weight = 0.0;
      }
      inline float volume()
      {
        return dx * dy * dz;
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
      inline float operator [] (unsigned int i)
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

      inline std::set<int> visibleFaceIndices(const Eigen::Vector3f local_view_point) const
      {
        std::set<int> visible_faces;
        if (local_view_point[0] > 0) {
          visible_faces.insert(0);
        }
        else {
          visible_faces.insert(2);
        }
        if (local_view_point[1] > 0) {
          visible_faces.insert(1);
        }
        else {
          visible_faces.insert(3);
        }
        if (local_view_point[2] > 0) {
          visible_faces.insert(4);
        }
        else {
          visible_faces.insert(5);
        }
        return visible_faces;
      }

      // V should be on local coordinates.
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
      Eigen::Affine3f transformed_affine = particle_affine * transform;
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
} // pcl

namespace jsk_pcl_ros
{
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
    virtual size_t chooseUniformRandomPlaneIndex();
    virtual void configCallback(Config& config, uint32_t level);
    
    // For particle filtering
    virtual pcl::tracking::ParticleCuboid sample(const pcl::tracking::ParticleCuboid& p);
    // Likelihood
    virtual void likelihood(pcl::PointCloud<pcl::PointXYZ>::ConstPtr input,
                            pcl::tracking::ParticleCuboid& p);
    virtual double rangeLikelihood(const pcl::tracking::ParticleCuboid& p);
    virtual double binaryLikelihood(double v, double min, double max);
    virtual double computeNumberOfPoints(
      const pcl::tracking::ParticleCuboid& p);
    virtual void publishHistogram(ParticleCloud::Ptr particles, int index,
                                  ros::Publisher& pub,
                                  const std_msgs::Header& header);
    virtual double distanceFromPlaneBasedError(const Particle& p);
    virtual pcl::PointCloud<pcl::PointXYZI>::Ptr convertParticlesToXYZI(ParticleCloud::Ptr particles);
    virtual bool resetCallback(std_srvs::EmptyRequest& req,
                               std_srvs::EmptyResponse& res);
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
    double init_local_position_z_min_;
    double init_local_position_z_max_;
    bool use_init_world_position_z_model_;
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
    
    int particle_num_;
    bool use_range_likelihood_;
    double range_likelihood_local_min_z_;
    double range_likelihood_local_max_z_;
    bool use_occlusion_likelihood_;
    int min_inliers_;
    double outlier_distance_;
    std::string sensor_frame_;
    boost::mt19937 random_generator_;

    pcl::tracking::ROSCollaborativeParticleFilterTracker<pcl::PointXYZ, pcl::tracking::ParticleCuboid>::Ptr tracker_;
  private:
    
  };
}

#endif
