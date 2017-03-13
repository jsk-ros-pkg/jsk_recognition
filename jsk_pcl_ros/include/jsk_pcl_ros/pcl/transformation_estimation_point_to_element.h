/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
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
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
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
 *
 * $Id$
 *
 */
#ifndef PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_ELEMENT_H_
#define PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_ELEMENT_H_

#include <jsk_pcl_ros/pcl/point_element.h>
#include <pcl/registration/transformation_estimation.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/warp_point_rigid.h>


namespace pcl
{
  namespace registration
  {
    enum {
      ELEMENT_LINE,
      ELEMENT_EDGE,
      ELEMENT_PLANE,
      ELEMENT_POLYGON,
      ELEMENT_POINT,
      ELEMENT_FIXED,
      ELEMENT_INVALID,
    };


    /** @b TransformationEstimationPointToElement uses Levenberg Marquardt optimization to find the
      * transformation that minimizes the point-to-element distance between the given correspondences.
      *
      * \author Masaki Murooka
      * \ingroup registration
      */
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class TransformationEstimationPointToElement : public TransformationEstimationLM<PointSource, PointTarget, Scalar>
    {
      public:
        typedef boost::shared_ptr<TransformationEstimationPointToElement<PointSource, PointTarget, Scalar> > Ptr;
        typedef boost::shared_ptr<const TransformationEstimationPointToElement<PointSource, PointTarget, Scalar> > ConstPtr;

        typedef pcl::PointCloud<PointSource> PointCloudSource;
        typedef typename PointCloudSource::Ptr PointCloudSourcePtr;
        typedef typename PointCloudSource::ConstPtr PointCloudSourceConstPtr;
        typedef pcl::PointCloud<PointTarget> PointCloudTarget;
        typedef PointIndices::Ptr PointIndicesPtr;
        typedef PointIndices::ConstPtr PointIndicesConstPtr;

        typedef Eigen::Matrix<Scalar, 3, 1> Vector3;
        typedef Eigen::Matrix<Scalar, 4, 1> Vector4;

        double tolerance_line;
        double tolerance_edge;
        double tolerance_plane;
        double tolerance_polygon;
        double tolerance_point;
        double fixed_weight;

        TransformationEstimationPointToElement ()
        {
          tolerance_line = 0.0;
          tolerance_edge = 0.0;
          tolerance_plane = 0.0;
          tolerance_polygon = 0.0;
          tolerance_point = 0.0;
          fixed_weight = 0.0;
        };
        virtual ~TransformationEstimationPointToElement () {};

      protected:
        virtual Scalar
        computeDistance (const PointSource &p_src, const PointTarget &p_tgt) const
        {
          // Compute the point-to-element distance
          Vector4 p_src_vec (p_src.x, p_src.y, p_src.z, 0);
          return computeDistance (p_src_vec, p_tgt);
        }

      /**
       *  \brief compute distance between point and element
       *  \param p_src  point in source
       *  \param p_tgt  edge in target
       *                x,y,z represents start point and norm_x,y,z represents vector from start point to end point.
       *                edge length is not considered if consider_edge_length is false.
       */
        virtual Scalar
        computeDistance (const Vector4 &p_src, const PointTarget &p_tgt) const
        {
          Eigen::Vector3f p (p_src(0), p_src(1), p_src(2));
          if (p_tgt.label == ELEMENT_LINE) {
            return std::max(p_tgt.edge.distanceToPoint(p) - tolerance_line, 0.0);
          } else if (p_tgt.label == ELEMENT_EDGE) {
            return std::max(p_tgt.edge.distance(p) - tolerance_edge, 0.0);
          } else if (p_tgt.label == ELEMENT_PLANE) {
            jsk_recognition_utils::Polygon poly = p_tgt.polygon;
            return std::max(poly.distanceToPoint(p) - tolerance_plane, 0.0);
          } else if (p_tgt.label == ELEMENT_POLYGON) {
            jsk_recognition_utils::Polygon poly = p_tgt.polygon;
            return std::max(poly.distance(p) - tolerance_polygon, 0.0);
          } else if (p_tgt.label == ELEMENT_POINT) {
            Eigen::Vector3f point_tgt = p_tgt.point;
            Eigen::Vector3f point_src = p_src.block(0, 0, 3, 1);
            return std::max((point_tgt - point_src).norm() - tolerance_point, 0.0);
          } else if (p_tgt.label == ELEMENT_FIXED) {
            Eigen::Vector3f point_tgt = p_tgt.point;
            Eigen::Vector3f point_src = p_src.block(0, 0, 3, 1);
            return fixed_weight * (point_tgt - point_src).norm();
          } else if (p_tgt.label == ELEMENT_INVALID) {
            return 0;
          } else {
            std::cerr << "no supporting label" << std::endl;
            return 0;
          }
        }

    };
  }
}

#endif /* PCL_REGISTRATION_TRANSFORMATION_ESTIMATION_POINT_TO_ELEMENT_H_ */

