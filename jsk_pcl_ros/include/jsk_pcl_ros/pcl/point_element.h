#ifndef PCL_POINT_ELEMENT_H_
#define PCL_POINT_ELEMENT_H_

#define PCL_NO_PRECOMPILE
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <jsk_recognition_utils/geo/segment.h>
#include <jsk_recognition_utils/geo/polygon.h>

namespace pcl
{
  struct EIGEN_ALIGN16 PointElement
  {
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    jsk_recognition_utils::Segment edge;
    jsk_recognition_utils::Polygon polygon;
    Eigen::Vector3f point;
    unsigned int label;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}

#endif
