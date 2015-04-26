/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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

// -*- mode: c++ -*-
#ifndef JSK_PCL_ROS_POINT_TYPES_H__
#define JSK_PCL_ROS_POINT_TYPES_H__
#include <pcl/point_types.h>
namespace pcl
{

#define  HSV_C_STRUCT union {                   \
        struct {                                \
            float hue;                          \
            float saturation;                   \
            float value;                        \
        };                                      \
        float data_c[4];                        \
    }

    // type for HSV
    struct PointXYZHSV
    {
        PCL_ADD_POINT4D;
        HSV_C_STRUCT;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;

    struct PointHSV
    {
        HSV_C_STRUCT;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
    
    struct PointRGB
    {
        float rgb;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    } EIGEN_ALIGN16;
}

// register the point types into PCL
POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointRGB,
                                   (float, rgb, rgb));

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointHSV,
                                   (float, hue, hue)
                                   (float, saturation, saturation)
                                   (float, value, value));

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::PointXYZHSV,
                                   (float, x, x)
                                   (float, y, y)
                                   (float, z, z)
                                   (float, hue, hue)
                                   (float, saturation, saturation)
                                   (float, value, value));

#endif
