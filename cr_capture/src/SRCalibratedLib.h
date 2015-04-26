#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

class SRCalibratedLib {
private:
  sensor_msgs::CameraInfo info_depth_;
  IplImage *ipl_depth_;
  float *map_x, *map_y, *map_z;
  int srheight, srwidth;
  CvMat *cam_matrix, *dist_coeff;

protected:
  void
  makeConvertMap ();

  void
  convert3DPos (sensor_msgs::PointCloud &pts);

public:
  // parameters
  double max_range;
  double depth_scale;
  bool short_range;
  int intensity_threshold, confidence_threshold;

  bool use_smooth;
  int smooth_size;
  double smooth_depth,smooth_space;

  bool use_filter;
  double edge1, edge2;
  int dilate_times;

  SRCalibratedLib ();

  inline int
  width ()
  {
    return srwidth;
  }

  inline int
  height ()
  {
    return srheight;
  }

  void
  setRengeImg (const sensor_msgs::ImageConstPtr &img_conf,
               const sensor_msgs::ImageConstPtr &img_intent,
               const sensor_msgs::ImageConstPtr &img_depth,
               const sensor_msgs::CameraInfoConstPtr &info);

  void
  calc3DPoints (sensor_msgs::PointCloud &pts_);
};
