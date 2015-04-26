#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>

#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include "cr_capture/PixelIndices.h"

class CRLib {
private:
  sensor_msgs::CameraInfo info_left_, info_right_;
  IplImage *ipl_left_, *ipl_right_;

public:
  // parameters
  tf::StampedTransform cam_trans;
  bool clear_uncolored_points;

  CRLib ();

  void
  setLeftImg (const sensor_msgs::ImageConstPtr &img,
              const sensor_msgs::CameraInfoConstPtr &info);

  void
  setRightImg (const sensor_msgs::ImageConstPtr &img,
               const sensor_msgs::CameraInfoConstPtr &info);

  bool
  calcColor (sensor_msgs::PointCloud &pts, int srwidth, int srheight,
             cr_capture::PixelIndices *pidx = NULL);

};
