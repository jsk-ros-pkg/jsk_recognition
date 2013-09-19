#include "CRLib.h"

CRLib::CRLib ()
{
  ipl_left_ = new IplImage();
  ipl_right_ = new IplImage();
}

void
CRLib::setLeftImg (const sensor_msgs::ImageConstPtr &img,
                   const sensor_msgs::CameraInfoConstPtr &info)
{
  if( (ipl_left_->width != (int)img->width) ||
      (ipl_left_->height != (int)img->height) )
  {
      ipl_left_ = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
  }
  IplImage iplimg(cv_bridge::toCvCopy(img, "rgb8")->image);
  cvResize(&iplimg, ipl_left_);
  info_left_ = *info;
}

void
CRLib::setRightImg (const sensor_msgs::ImageConstPtr &img,
                    const sensor_msgs::CameraInfoConstPtr &info)
{
  if( (ipl_right_->width != (int)img->width) ||
      (ipl_right_->height != (int)img->height) )
  {
    ipl_right_ = cvCreateImage(cvSize(img->width, img->height), IPL_DEPTH_8U, 3);
  }
  IplImage iplimg(cv_bridge::toCvCopy(img, "rgb8")->image);
  cvResize(&iplimg, ipl_right_);
  info_right_ = *info;
}

bool
CRLib::calcColor (sensor_msgs::PointCloud &pts, int srwidth, int srheight,
                  cr_capture::PixelIndices *pidx)
{
  if ( (ipl_right_->width != ipl_left_->width) ||
       (ipl_right_->height != ipl_left_->height) )
    return false;

  pts.channels.resize(1);
  pts.channels[0].name = "rgb";
  pts.channels[0].values.resize(srwidth*srheight);

  geometry_msgs::Point32 *point_ptr = &(pts.points[0]);
  float fx = info_left_.P[0];
  float cx = info_left_.P[2];
  float fy = info_left_.P[5];
  float cy = info_left_.P[6];
  float tr = info_right_.P[3]; // for ROS projection matrix (unit = m)

  int *lu_ptr = NULL, *ru_ptr = NULL, *v_ptr = NULL;
  if (pidx != NULL)
  {
    lu_ptr = new int[srheight*srwidth];
    ru_ptr = new int[srheight*srwidth];
    v_ptr = new int[srheight*srwidth];
      for(int i=0; i < srheight*srwidth; i++)
      {
        lu_ptr[i] = -1;
        ru_ptr[i] = -1;
        v_ptr[i] = -1;
      }
  }

  unsigned char *imgl = (unsigned char *)ipl_left_->imageData;
  unsigned char *imgr = (unsigned char *)ipl_right_->imageData;
  int w = ipl_left_->width;
  int h = ipl_left_->height;
  int step = ipl_left_->widthStep;

#define getPixel(img_ptr, pix_x, pix_y, color_r, color_g, color_b)      \
  { color_r = img_ptr[step*pix_y + pix_x*3 + 0];                        \
    color_g = img_ptr[step*pix_y + pix_x*3 + 1];                        \
    color_b = img_ptr[step*pix_y + pix_x*3 + 2]; }

  int ypos[srwidth];
  int lxpos[srwidth];
  int rxpos[srwidth];
  int col_x[srwidth];
  int lr_use[srwidth];

  for (int y=0; y<srheight; y++)
  {
    for (int x=0; x<srwidth; x++)
    {
      int index = y*srwidth + x;
      // convert camera coordinates
      tf::Vector3 pos(point_ptr[index].x, point_ptr[index].y, point_ptr[index].z);
      tf::Vector3 tpos = cam_trans * pos;

      float posx = tpos[0];
      float posy = tpos[1];
      float posz = tpos[2];
      if (posz > 0.100)
      { // filtering near points
        lxpos[x] = (int)(fx/posz * posx + cx);      // left cam
        rxpos[x] = (int)((fx*posx + tr)/posz + cx); // right cam
        ypos[x] = (int)(fy/posz * posy + cy);
      } else {
        lxpos[x] = -1;
        rxpos[x] = -1;
        ypos[x] = -1;
      }
      //ROS_INFO("%d\t%d\t%d",lxpos[x],rxpos[x],ypos[x]);
    }
    memset(lr_use, 0, sizeof(int)*srwidth);
    memset(col_x, 0x01000000, sizeof(int)*srwidth);

    int max_lx = -1;
    int min_rx = w;
    for (int x=0; x<srwidth; x++)
    {
      int lx = lxpos[x];
      int ly = ypos[x];

      int pr = srwidth -x -1;
      int rx = rxpos[pr];
      int ry = ypos[pr];

      if ((w > lx ) && (lx >= 0)
          && (h > ly) && (ly >= 0)) {
        if(lx >= max_lx) {
          max_lx = lx;
        } else {
          lr_use[x] = -1; // use right
        }
      }
      if ((w > rx) && (rx >= 0)
          && (h > ry) && (ry >= 0)) {
        if(rx <= min_rx) {
          min_rx = rx;
        } else {
          lr_use[pr] = 1; // use left
        }
      }
    }
    // finding similar color
    unsigned char lcolr=0, lcolg=0, lcolb=0;
    unsigned char rcolr=0, rcolg=0, rcolb=0;
    for (int x=0; x<srwidth; x++) {
      if (lr_use[x]==0)
      {
        int lx = lxpos[x];
        int rx = rxpos[x];
        int yy = ypos[x];
        if ((w > lx ) && (lx >= 0)
            && (w > rx) && (rx >= 0)
            && (h > yy) && (yy >= 0))
        {
          getPixel(imgl, lx, yy, lcolr, lcolg, lcolb);
          getPixel(imgr, rx, yy, rcolr, rcolg, rcolb);

          double norm = 0.0;
          double norm_r = (double)(lcolr - rcolr);
          double norm_g = (double)(lcolg - rcolg);
          double norm_b = (double)(lcolb - rcolb);
          norm += norm_r * norm_r;
          norm += norm_g * norm_g;
          norm += norm_b * norm_b;
          norm = sqrt(norm);

          if(norm < 50.0) { // magic number for the same color
            col_x[x] = ((0xFF & ((lcolr + rcolr)/2)) << 16) |
              ((0xFF & ((lcolg + rcolg)/2)) << 8) |
              (0xFF & ((lcolb + rcolb)/2));
            if(pidx != NULL) {
              int ptr_pos = (x + y*srwidth);
              lu_ptr[ptr_pos] = lx;
              ru_ptr[ptr_pos] = rx;
              v_ptr[ptr_pos]  = yy;
            }
          } else {
            col_x[x] = 0x2000000;
            // find nearest one in next loop
          }
        } else if ((w > lx ) && (lx >= 0)
                   && (h > yy) && (yy >= 0))
        {
          // only left camera is viewing
          getPixel(imgl, lx, yy, lcolr, lcolg, lcolb);
          col_x[x] = ((0xFF & lcolr) << 16) | ((0xFF & lcolg) << 8) | (0xFF & lcolb);

          if(pidx != NULL) {
            int ptr_pos = (x + y*srwidth);
            lu_ptr[ptr_pos] = lx;
            v_ptr[ptr_pos]  = yy;
          }
        } else if ((w > rx ) && (rx >= 0)
                   && (h > yy) && (yy >= 0))
        {
          // only right camera is viewing
          getPixel(imgr, rx, yy, rcolr, rcolg, rcolb);
          col_x[x] = ((0xFF & rcolr) << 16) | ((0xFF & rcolg) << 8) | (0xFF & rcolb);

          if (pidx != NULL) {
            int ptr_pos = (x + y*srwidth);
            ru_ptr[ptr_pos] = rx;
            v_ptr[ptr_pos]  = yy;
          }
        } else {
          // did not find corresponding points in image
          col_x[x] = 0xFF0000;
          if (clear_uncolored_points) {
            int pidx = y*srwidth + x;
            point_ptr[pidx].x = 0.0;
            point_ptr[pidx].y = 0.0;
            point_ptr[pidx].z = 0.0;
          }
        }
      } else if (lr_use[x] > 0) {
        // use left
        int lx = lxpos[x];
        int ly = ypos[x];
        getPixel(imgl, lx, ly, lcolr, lcolg, lcolb);
        col_x[x] = ((0xFF & lcolr) << 16) | ((0xFF & lcolg) << 8) | (0xFF & lcolb);

        if (pidx != NULL) {
          int ptr_pos = (x + y*srwidth);
          lu_ptr[ptr_pos] = lx;
          v_ptr[ptr_pos]  = ly;
        }
      } else {
        // use right
        int rx = rxpos[x];
        int ry = ypos[x];
        getPixel(imgr, rx, ry, rcolr, rcolg, rcolb);
        col_x[x] = ((0xFF & rcolr) << 16) | ((0xFF & rcolg) << 8) | (0xFF & rcolb);

        if (pidx != NULL) {
          int ptr_pos = (x + y*srwidth);
          ru_ptr[ptr_pos] = rx;
          v_ptr[ptr_pos]  = ry;
        }
      }
    }
    // checking color of nearest one
    for (int x=0; x<srwidth; x++) {
      if (col_x[x] & 0x02000000) {
        int n = 0x02000000;
        for (int p=0; p<srwidth; p++) {
          if ((x+p >= srwidth) &&
              (x-p < 0)) {
            break;
          } else {
            if(x+p < srwidth) {
              if(!(col_x[x+p] & 0xFF000000)){
                n = col_x[x+p];
                break;
              }
            }
            if(x-p >= 0) {
              if(!(col_x[x-p] & 0xFF000000)){
                n = col_x[x-p];
                break;
              }
            }
          }
        }
        if(!(n & 0xFF000000)) {
          int lx = lxpos[x];
          int rx = rxpos[x];
          int yy = ypos[x];
          getPixel(imgl, lx, yy, lcolr, lcolg, lcolb);
          getPixel(imgr, rx, yy, rcolr, rcolg, rcolb);
          int clr = (n >> 16) & 0xFF;
          int clg = (n >> 8) & 0xFF;
          int clb = (n >> 0) & 0xFF;
          int dif_l = abs(clr - lcolr) + abs(clg - lcolg) + abs(clb - lcolb);
          int dif_r = abs(clr - rcolr) + abs(clg - rcolg) + abs(clb - rcolb);
          if(dif_l < dif_r) {
            col_x[x] = ((0xFF & lcolr) << 16) | ((0xFF & lcolg) << 8) | (0xFF & lcolb);

            if(pidx != NULL) {
              int ptr_pos = (x + y*srwidth);
              lu_ptr[ptr_pos] = lx;
              v_ptr[ptr_pos]  = yy;
            }
          } else {
            col_x[x] = ((0xFF & rcolr) << 16) | ((0xFF & rcolg) << 8) | (0xFF & rcolb);

            if(pidx != NULL) {
              int ptr_pos = (x + y*srwidth);
              ru_ptr[ptr_pos] = rx;
              v_ptr[ptr_pos]  = yy;
            }
          }
        }
      }
    }
    // setting color
    float *colv = &(pts.channels[0].values[y*srwidth]);
    for (int x=0; x<srwidth; x++)
      colv[x] = *reinterpret_cast<float*>(&(col_x[x]));
  } // y_loop

  if ( pidx != NULL ) {
    pidx->header = pts.header;
    pidx->indices.resize(srwidth*srheight*3);
    for (int i=0; i<srwidth*srheight; i++)
      {
        pidx->indices[3*i + 0] = lu_ptr[i];
        pidx->indices[3*i + 1] = ru_ptr[i];
        pidx->indices[3*i + 2] = v_ptr[i];
      }
    delete lu_ptr;
    delete ru_ptr;
    delete v_ptr;
  }
  return true;
}
