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
