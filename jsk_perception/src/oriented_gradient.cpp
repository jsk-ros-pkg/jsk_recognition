// @brief calc oriented gradent
// @author Hiroaki Yaguchi, JSK
#include "jsk_perception/oriented_gradient.hpp"
#include <jsk_topic_tools/log_utils.h>

#if ( CV_MAJOR_VERSION >= 4)
#include <opencv2/imgproc/types_c.h>
#endif

namespace jsk_perception {

// @brief calc 8 neighbor oriented gradient image
// @param src source image
// @param dst destination image (HSV, H : orientation, V : intensity)
void calcOrientedGradient(cv::Mat& src, cv::Mat& dst) {
  int width, height;
  cv::Mat gimg;

  width = src.cols;
  height = src.rows;

  cv::cvtColor(src, gimg, CV_BGR2GRAY);

  dst.create(height, width, CV_8UC3);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      cv::Vec3b& px=dst.at<cv::Vec3b>(y, x);
      for (int k = 0; k < 3; k++) {
        px[k] = 0;
      }
    }
  }

  float r2 = sqrt(2.0);

  for (int y = 1; y < height - 1; y++) {
    for (int x = 1; x < width - 1; x++) {
      int m,th;
      float dx,dy;
      float dxx,dyy,dxy,dyx;
      double dth;

      dxx = (float)gimg.at<unsigned char>(y, x + 1)
        - (float)gimg.at<unsigned char>(y, x - 1);
      dyy = (float)gimg.at<unsigned char>(y + 1, x)
        - (float)gimg.at<unsigned char>(y - 1, x);
      dxy = (float)gimg.at<unsigned char>(y + 1, x + 1)
        - (float)gimg.at<unsigned char>(y - 1, x - 1);
      dyx = (float)gimg.at<unsigned char>(y + 1, x - 1)
        - (float)gimg.at<unsigned char>(y - 1, x + 1);

      dx = 0.5 * (dxx + 0.5 * r2 * (dxy - dyx));
      dy = 0.5 * (dyy + 0.5 * r2 * (dxy + dyx));
      // dx = dxx; dy = dyy;

      m = (int)(sqrt(0.5 * (dx * dx + dy * dy)));

      dth = atan2(dy, dx) * 180.0 / M_PI;
      if (dth < 0.0) dth += 180.0;
      if (dth >= 180.0) dth -= 180.0;
      th = (int)dth;

      dst.at<cv::Vec3b>(y, x) = cv::Vec3b(th, 255, m);

    }
  }
}

// @brief calc key points from oriented gradient image
// @param src source image
// @param dst destination image (HSV, H : orientation, V : intensity)
// @param result key points
// @param thres (optional) threshold of intensity
// @param bs (optional) block size
void calcOGKeyPoints(cv::Mat& src,
                     cv::Mat& dst,
                     std::vector<cv::Point>& result,
                     int thres,
                     int bs) {
  calcOrientedGradient(src, dst);
  int width, height;
  width = src.cols;
  height = src.rows;

  result.clear();

  for (int y = bs; y < height - bs; y++) {
    for (int x = bs; x < width - bs; x++) {
      cv::Vec3b& px0 = dst.at<cv::Vec3b>(y, x);
      if (px0[2] > thres) {
        bool iskey = true;
        for (int dx = -bs; dx <= bs; dx++) {
          for (int dy = -bs; dy <= bs; dy++) {
            if (dx == 0 && dy == 0) break;
            cv::Vec3b& px1 = dst.at<cv::Vec3b>(y + dy, x + dx);
            if (px0[2] < px1[2]) iskey = false;
          }
        }
        if (iskey) result.push_back(cv::Point(x, y));
      }

    }
  }

}

// @brief calc 8 neighbor scaled oriented gradient image
// @param src source image
// @param dst destination image (HSV, H : orientation, V : intensity)
// @param scale scale
void calcScaledOrientedGradient(cv::Mat& src, cv::Mat& dst, int scale) {
  cv::Mat gimg;
  cv::Mat intimg, sqintimg, tintimg;

  int width = src.cols;
  int height = src.rows;

  cv::cvtColor(src, gimg, CV_BGR2GRAY);

  dst.create(height, width, CV_8UC3);
  unsigned char *gradbuf = dst.ptr();

  intimg.create(height + 1, width + 1, CV_32S);
  sqintimg.create(height + 1, width + 1, CV_32S);
  tintimg.create(height + 1, width + 1, CV_32S);

  cv::integral(gimg, intimg, sqintimg, tintimg);

  float r2 = sqrt(2.0);
  int xidx, idx;
  int hscale = scale / 2;
  int bsize = hscale * 2 + 1;
  int barea0 = scale * scale;
  int barea1 = scale * bsize;
  int margin = scale;
  for (int y = margin; y < height - margin; y++) {
    xidx = y * width;
    for (int x = margin; x < width - margin; x++) {
      int m, th;
      float dx, dy;
      float dxx, dyy, dxy, dyx;
      double dth;

      dxx = ((float)(intimg.at<int>(y + hscale + 1, x + scale + 1)
                     + intimg.at<int>(y - hscale, x + 1)
                     - intimg.at<int>(y - hscale, x + scale + 1)
                     - intimg.at<int>(y + hscale + 1, x + 1))
             - (float)(intimg.at<int>(y + hscale + 1, x)
                       +intimg.at<int>(y - hscale, x - scale)
                       -intimg.at<int>(y - hscale, x)
                       -intimg.at<int>(y + hscale + 1, x - scale))) / barea1;
      dyy = ((float)(intimg.at<int>(y + scale + 1, x + hscale + 1)
                     + intimg.at<int>(y + 1, x - hscale)
                     - intimg.at<int>(y + scale + 1, x - hscale)
                     - intimg.at<int>(y + 1, x + hscale + 1))
             - (float)(intimg.at<int>(y - scale, x - hscale)
                       +intimg.at<int>(y, x + hscale + 1)
                       -intimg.at<int>(y - scale, x + hscale + 1)
                       -intimg.at<int>(y, x - hscale))) / barea1;

      dxy = ((float)(intimg.at<int>(y + scale + 1, x + scale + 1)
                     + intimg.at<int>(y + 1, x + 1)
                     - intimg.at<int>(y + 1, x + scale + 1)
                     - intimg.at<int>(y + scale + 1, x + 1))
             - (float)(intimg.at<int>(y, x)
                       + intimg.at<int>(y - scale, x - scale)
                       - intimg.at<int>(y, x - scale)
                       - intimg.at<int>(y - scale, x))) / barea0;
      dyx = ((float)(intimg.at<int>(y + scale + 1, x)
                     + intimg.at<int>(y + 1, x - scale)
                     - intimg.at<int>(y + 1, x)
                     - intimg.at<int>(y + scale + 1, x - scale))
             - (float)(intimg.at<int>(y, x + scale + 1)
                       + intimg.at<int>(y - scale, x + 1)
                       - intimg.at<int>(y - scale, x + scale + 1)
                       - intimg.at<int>(y, x + 1))) / barea0;

      dx = 0.5 * (dxx + 0.5 * r2 * (dxy - dyx));
      dy = 0.5 * (dyy + 0.5 * r2 * (dxy + dyx));

      m = (int)(sqrt(0.5 * (dx * dx + dy * dy)));

      dth = atan2(dy, dx) * 180.0 / M_PI;
      if (dth < 0.0) dth += 180.0;
      if (dth >= 180.0) dth -= 180.0;
      th = (int)dth;

      gradbuf[(xidx + x) * 3] = th;
      gradbuf[(xidx + x) * 3 + 1] = 255;
      gradbuf[(xidx + x) * 3 + 2] = m;
    }
  }
}

// @brief calc key points from scaled oriented gradient image
// @param src source image
// @param dst destination image (HSV, H : orientation, V : intensity)
void calcSOGKeyPoints(cv::Mat& src, cv::Mat& dst) {
  cv::Mat gimg;
  cv::Mat intimg, sqintimg, tintimg;

  int width = src.cols;
  int height = src.rows;

  std::vector<std::vector<float> > gradimglist;
  std::vector<int> scalelist;

  cv::cvtColor(src, gimg, CV_BGR2GRAY);

  dst.create(height, width, CV_8UC3);
  unsigned char *gradbuf = dst.ptr();

  intimg.create(height + 1, width + 1, CV_32S);
  sqintimg.create(height + 1, width + 1, CV_32S);
  tintimg.create(height + 1, width + 1, CV_32S);

  cv::integral(gimg, intimg, sqintimg, tintimg);

  // scale=1,1+1*2,1+2*2,...,1+s*2
  int maxscale = 10;
  scalelist.resize(width * height);

  // gradimglist = g(0) gx(0), gy(0), ...
  gradimglist.resize((maxscale + 1) * 3);

  for(int i = 0; i < gradimglist.size(); i++)
    gradimglist[i].resize(width * height);

  // s=1
  float r2 = sqrt(2.0);
  int xidx, idx;
  for (int y = 1; y < height - 1; y++) {
    xidx = y * width;
    for (int x = 1; x < width - 1; x++) {
      float dx, dy;
      float dxx, dyy, dxy, dyx;

      dxx = (float)gimg.at<unsigned char>(y, x + 1)
        - (float)gimg.at<unsigned char>(y, x - 1);
      dyy = (float)gimg.at<unsigned char>(y + 1, x)
        - (float)gimg.at<unsigned char>(y - 1, x);
      dxy = (float)gimg.at<unsigned char>(y + 1, x + 1)
        - (float)gimg.at<unsigned char>(y - 1, x - 1);
      dyx = (float)gimg.at<unsigned char>(y + 1, x - 1)
        - (float)gimg.at<unsigned char>(y - 1, x + 1);

      dx = 0.5 * (dxx + 0.5 * r2 * (dxy - dyx));
      dy = 0.5 * (dyy + 0.5 * r2 * (dxy + dyx));

      idx = xidx + x;
      gradimglist[0][idx] = dx * dx + dy * dy;
      gradimglist[1][idx] = dx;
      gradimglist[2][idx] = dy;
    }
  }

  // s >= 2
  for (int scale = 2; scale <= maxscale; scale++) {

    int hscale = scale / 2;
    int bsize = hscale * 2 + 1;
    int barea0 = scale * scale;
    int barea1 = scale * bsize;
    int margin = scale;

    int sidx = (scale - 1) * 3;
    for (int y = margin; y < height - margin; y++) {
      xidx = y * width;
      for (int x = margin; x < width - margin; x++) {
        float dx, dy;
        float dxx, dyy, dxy, dyx;

        dxx = ((float)(intimg.at<int>(y + hscale + 1, x + scale + 1)
                       + intimg.at<int>(y - hscale, x + 1)
                       - intimg.at<int>(y - hscale, x + scale + 1)
                       - intimg.at<int>(y + hscale + 1, x + 1))
               - (float)(intimg.at<int>(y + hscale + 1, x)
                         + intimg.at<int>(y - hscale, x - scale)
                         - intimg.at<int>(y - hscale, x)
                         - intimg.at<int>(y + hscale + 1, x - scale)))
            / barea1;
        dyy = ((float)(intimg.at<int>(y + scale + 1, x + hscale + 1)
                       + intimg.at<int>(y + 1, x - hscale)
                       - intimg.at<int>(y + scale + 1, x - hscale)
                       - intimg.at<int>(y + 1, x + hscale + 1))
               -(float)(intimg.at<int>(y - scale, x - hscale)
                        +intimg.at<int>(y, x + hscale + 1)
                        -intimg.at<int>(y - scale, x + hscale + 1)
                        -intimg.at<int>(y, x - hscale))) / barea1;


        dxy = ((float)(intimg.at<int>(y + scale + 1, x + scale + 1)
                       + intimg.at<int>(y + 1, x + 1)
                       - intimg.at<int>(y + 1, x + scale + 1)
                       - intimg.at<int>(y + scale + 1, x + 1))
               - (float)(intimg.at<int>(y, x)
                         + intimg.at<int>(y - scale, x - scale)
                         - intimg.at<int>(y, x - scale)
                         - intimg.at<int>(y - scale, x))) / barea0;
        dyx = ((float)(intimg.at<int>(y + scale + 1, x)
                       + intimg.at<int>(y + 1, x - scale)
                       - intimg.at<int>(y + 1, x)
                       - intimg.at<int>(y + scale + 1, x - scale))
               - (float)(intimg.at<int>(y, x + scale + 1)
                         + intimg.at<int>(y - scale, x + 1)
                         - intimg.at<int>(y - scale, x + scale + 1)
                         - intimg.at<int>(y, x + 1))) / barea0;

        dx = 0.5 * (dxx + 0.5 * r2 * (dxy - dyx));
        dy = 0.5 * (dyy + 0.5 * r2 * (dxy + dyx));

        idx = xidx + x;
        gradimglist[sidx][idx] = dx * dx + dy * dy;
        gradimglist[sidx + 1][idx] = dx;
        gradimglist[sidx + 2][idx] = dy;
      }
    }
  }

  // scale estimation
  int ofs = 1 + maxscale;
  for (int y = ofs; y < height - ofs; y++) {
    xidx = y * width;
    for (int x = ofs; x < width - ofs; x++) {
      idx = xidx + x;
      float m, mmax, th;
      int sscale = 0;
      int mscale = sscale;
      float dx, dy;
      mmax = gradimglist[sscale][idx];
      for (int scale = sscale + 1; scale <= maxscale; scale++) {
        m = gradimglist[scale * 3][idx];
        if (mmax < m) {
          mmax = m;
          mscale = scale;
        } else {
          break;
        }
      }
      m = sqrt(mmax);
      dx = gradimglist[mscale * 3 + 1][idx];
      dy = gradimglist[mscale * 3 + 2][idx];
      th = atan2(dy, dx) * 180.0 / M_PI;
      if (th < 0.0) th += 180.0;
      if (th >= 180.0) th -= 180.0;
      scalelist[idx] = mscale;

      gradbuf[idx * 3] = (unsigned char)th;
      gradbuf[idx * 3 + 1] = 255;
      gradbuf[idx * 3 + 2] = (unsigned char)m;
    }
  }

  // keypoint localization
  for (int y = ofs; y < height - ofs; y++) {
    xidx = y * width;
    for (int x = ofs; x < width - ofs; x++) {
      idx = xidx + x;
      int scale = scalelist[idx];
      float m = gradimglist[scale][idx];
      bool iskey = false;
      if (sqrt(m) > 32) {
        iskey = true;
        for (int sc = scale > 0 ? scale - 1 : 0;
             sc < (scale < maxscale ? scale + 1 : maxscale);
             sc++) {
          for (int xx = -1; xx <= 1; xx++) {
            if (m < gradimglist[sc][idx + xx]
                || m < gradimglist[sc][idx + xx + width]
                || m < gradimglist[sc][idx + xx - width]) {
              iskey = false;
              break;
            }
          }
        }
      }

      if (iskey) {
        cv::circle(src,
                   cv::Point(x, y),
                   scale + 1,
                   cv::Scalar(0, 0, 255),
                   1,
                   4);
      } else {
        // gradbuf[idx*3]=0;
        // gradbuf[idx*3+1]=0;
        // gradbuf[idx*3+2]=0;
      }
    }
  }
}


}  // namespace
