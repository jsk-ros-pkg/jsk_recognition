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


#ifndef JSK_PCL_ROS_PARTICLE_CUBOID_H_
#define JSK_PCL_ROS_PARTICLE_CUBOID_H_

#include <pcl/point_cloud.h>
#include "jsk_pcl_ros/geo_util.h"

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
        const std::vector<int>& visible_faces,
        std::vector<jsk_pcl_ros::Polygon::Ptr>& faces) const
      {
        double min_distance = DBL_MAX;
        for (size_t i = 0; i < visible_faces.size(); i++) {
          jsk_pcl_ros::Polygon::Ptr face = faces[visible_faces[i]];
          double d;
          face->nearestPoint(v, d);
          if (min_distance > d) {
            min_distance = d;
          }
        }
        return min_distance;
      }

      
      // V should be on local coordinates.
      // TODO: update to take into boundary account
      inline double signedDistanceToPlane(
        const Eigen::Vector3f& v, const int plane_index) const
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
        return (n.dot(v) + d) / sqrt(n.dot(n) + d * d);
      }

      inline double distanceToPlane(
        const Eigen::Vector3f& v, const int plane_index) const
      {
        return std::abs(signedDistanceToPlane(v, plane_index));
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
      
      inline jsk_pcl_ros::Cube::Ptr toCube() const
    {
      Eigen::Affine3f pose = toEigenMatrix();
      Eigen::Vector3f dimensions(dx, dy, dz);
      jsk_pcl_ros::Cube::Ptr cube(new jsk_pcl_ros::Cube(Eigen::Vector3f(pose.translation()),
                                                        Eigen::Quaternionf(pose.rotation()),
                                                        dimensions));
      return cube;
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


#endif
