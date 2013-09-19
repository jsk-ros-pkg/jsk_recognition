#include "SRCalibratedLib.h"

void
SRCalibratedLib::makeConvertMap ()
{
  if ((info_depth_.D[0] != cvmGet(dist_coeff, 0, 0)) ||
      (info_depth_.D[1] != cvmGet(dist_coeff, 0, 1)) ||
      (info_depth_.D[2] != cvmGet(dist_coeff, 0, 2)) ||
      (info_depth_.D[3] != cvmGet(dist_coeff, 0, 3)) ||
      (info_depth_.D[4] != cvmGet(dist_coeff, 0, 4)) ||
      ((depth_scale*info_depth_.K[3*0 + 0]) != cvmGet(cam_matrix, 0, 0)) ||
      ((depth_scale*info_depth_.K[3*0 + 2]) != cvmGet(cam_matrix, 0, 2)) ||
      ((depth_scale*info_depth_.K[3*1 + 1]) != cvmGet(cam_matrix, 1, 1)) ||
      ((depth_scale*info_depth_.K[3*1 + 2]) != cvmGet(cam_matrix, 1, 2)))
  {
    //
    if ( info_depth_.D.size() == 5 ) {
      for(int i=0; i < 5; i++) {
        cvmSet(dist_coeff, 0, i, info_depth_.D[i]);
      }
    } else if ( info_depth_.D.size() == 8 ) {
      ROS_INFO("using rational polynomial model");
      dist_coeff = cvCreateMat(1, 8, CV_64F);
      for(int i=0; i < 8; i++) {
        cvmSet(dist_coeff, 0, i, info_depth_.D[i]);
      }
    }
    //
    cvSetZero(cam_matrix);
    cvmSet(cam_matrix, 2, 2, 1.0);
    cvmSet(cam_matrix, 0, 0, depth_scale*(info_depth_.K[3*0 + 0]));//kx
    cvmSet(cam_matrix, 0, 2, depth_scale*(info_depth_.K[3*0 + 2]));//cx
    cvmSet(cam_matrix, 1, 1, depth_scale*(info_depth_.K[3*1 + 1]));//ky
    cvmSet(cam_matrix, 1, 2, depth_scale*(info_depth_.K[3*1 + 2]));//cy
    //
    CvMat *src = cvCreateMat(srheight*srwidth, 1, CV_32FC2);
    CvMat *dst = cvCreateMat(srheight*srwidth, 1, CV_32FC2);
    CvPoint2D32f *ptr = (CvPoint2D32f *)src->data.ptr;
    for (int v=0; v<srheight; v++)
      for (int u=0; u<srwidth; u++)
      {
        ptr->x = u;
        ptr->y = v;
        ptr++;
      }

    if(map_x != 0) delete map_x;
    if(map_y != 0) delete map_y;
    if(map_z != 0) delete map_z;
    map_x = new float[srwidth*srheight];
    map_y = new float[srwidth*srheight];
    map_z = new float[srwidth*srheight];

    cvUndistortPoints(src, dst, cam_matrix, dist_coeff, NULL, NULL);
    ptr = (CvPoint2D32f *)dst->data.ptr;
    for (int i=0; i<srheight*srwidth; i++)
      {
        float xx = ptr->x;
        float yy = ptr->y;
        ptr++;
        double norm = sqrt(xx * xx + yy * yy + 1.0);
        map_x[i] = xx / norm;
        map_y[i] = yy / norm;
        map_z[i] = 1.0 / norm;
      }
    cvReleaseMat(&src);
    cvReleaseMat(&dst);
    ROS_INFO("make conversion map");
  } else {
    //ROS_WARN("do nothing!");
  }
}

void
SRCalibratedLib::convert3DPos (sensor_msgs::PointCloud &pts)
{
  int lng=(srwidth*srheight);
  unsigned short *ibuf = (unsigned short*)ipl_depth_->imageData;
  geometry_msgs::Point32 *pt = &(pts.points[0]);

  for (int i=0; i<lng; i++){
    double scl = (max_range *  ibuf[i]) / (double)0xFFFF;

    if (short_range)
    {
      // magic number in this process from calibration results
      if (scl < 1.0)
      {
        scl *= ((-7.071927e+02*pow(scl,3) + 1.825608e+03*pow(scl,2) - 1.571370E+03*scl + 1.454931e+03)/1000.0);
      } else if (scl < 1.1) {
        scl *= ((1000 + (1.9763 / 100.0) * (1100.0 - 1000.0*scl)) /1000.0);
      }
    }

    if (ibuf[i] >= 0xFFF8) { // saturate
      pt->x = 0.0;
      pt->y = 0.0;
      pt->z = 0.0;
    } else {
      pt->x = (map_x[i] * scl);
      pt->y = (map_y[i] * scl);
      pt->z = (map_z[i] * scl);
    }
    pt++;
  }
}

SRCalibratedLib::SRCalibratedLib () : map_x(0), map_y(0), map_z(0)
{
    ipl_depth_ = new IplImage();
    cam_matrix = cvCreateMat(3, 3, CV_64F);
    dist_coeff = cvCreateMat(1, 5, CV_64F);
    cvSetZero(cam_matrix);
    cvSetZero(dist_coeff);
    cvmSet(cam_matrix, 2, 2, 1.0);
}

void
SRCalibratedLib::setRengeImg (const sensor_msgs::ImageConstPtr &img_conf,
                              const sensor_msgs::ImageConstPtr &img_intent,
                              const sensor_msgs::ImageConstPtr &img_depth,
                              const sensor_msgs::CameraInfoConstPtr &info)
{
  if ( (ipl_depth_->width != (int)img_depth->width) ||
       (ipl_depth_->height != (int)img_depth->height) )
  {
    ipl_depth_ = cvCreateImage(cvSize(img_depth->width, img_depth->height), IPL_DEPTH_16U, 1);
    srwidth = img_depth->width;
    srheight = img_depth->height;
  }

  IplImage iplimg(cv_bridge::toCvCopy(img_depth)->image); // pass through
  cvResize(&iplimg, ipl_depth_);
  info_depth_ = *info;

  if ( img_conf != img_depth && img_intent != img_depth )
  {
    const unsigned char *conf_buf = &(img_conf->data[0]);
    const unsigned char *intent_buf = &(img_intent->data[0]);
    unsigned short *depth_buf = (unsigned short*)ipl_depth_->imageData;
    int size = img_conf->data.size();
    for(int i=0;i<size;i++)
      {
        if( (conf_buf[i] < confidence_threshold) ||
            (intent_buf[i] < intensity_threshold) )
          depth_buf[i] = 0;
      }
  }

  if (depth_scale != 1.0)
  {
    srwidth = (int) (srwidth * depth_scale);
    srheight = (int) (srheight * depth_scale);
    IplImage *tmp_ipl_ = cvCreateImage(cvSize(srwidth, srheight), IPL_DEPTH_16U, 1);
    cvResize(ipl_depth_, tmp_ipl_);
    ipl_depth_ = tmp_ipl_;
  }
}

void
SRCalibratedLib::calc3DPoints (sensor_msgs::PointCloud &pts_)
{
  // smooth birateral filter
  if (use_smooth)
  {
    cv::Mat in_img16(ipl_depth_);
    cv::Mat in_imgf32(ipl_depth_->height, ipl_depth_->width, CV_32FC1);
    cv::Mat out_imgf32(ipl_depth_->height, ipl_depth_->width, CV_32FC1);
    in_img16.convertTo(in_imgf32, CV_32FC1);
    cv::bilateralFilter(in_imgf32, out_imgf32, smooth_size, smooth_depth, smooth_space, cv::BORDER_REPLICATE);
    out_imgf32.convertTo(in_img16, CV_16UC1);
  }

  // filter outlier
  if (use_filter)
  {
    cv::Mat in_img16(ipl_depth_);
    cv::Mat in_img(in_img16.rows, in_img16.cols, CV_8UC1);
    cv::Mat out_img(in_img16.rows, in_img16.cols, CV_8UC1);
    in_img16.convertTo(in_img, CV_8UC1, 1.0 / ( 1.0 * 256.0));

    cv::Canny(in_img, out_img, edge1, edge2);
    if(dilate_times >= 1)
      cv::dilate(out_img, out_img, cv::Mat(), cv::Point(-1, -1), dilate_times);

    unsigned short *sptr = (unsigned short *)in_img16.data;
    unsigned char *cptr = (unsigned char *)out_img.data;
    for(int i=0;i<in_img16.rows*in_img16.cols;i++)
      if(*cptr++ > 128) sptr[i] = 0;
  }

  // check info and make map
  makeConvertMap();

  //convert
  pts_.points.resize(srwidth*srheight);
  convert3DPos(pts_);
}
