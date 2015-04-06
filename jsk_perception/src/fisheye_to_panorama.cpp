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

#include "jsk_perception/fisheye_to_panorama.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <algorithm>
#include <math.h> 

#define PI 3.141592

namespace jsk_perception
{
  void FisheyeToPanorama::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_undistorted_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output", 1);
    pub_undistorted_bilinear_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "output_bilinear", 1);
  }

  void FisheyeToPanorama::subscribe()
  {
    sub_image_ = pnh_->subscribe("input", 1, &FisheyeToPanorama::rectify, this);
  }
  
  void FisheyeToPanorama::unsubscribe()
  {
    sub_image_.shutdown();
  }

  void FisheyeToPanorama::rectify(const sensor_msgs::Image::ConstPtr& image_msg)
  {
    
    cv::Mat distorted = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
    int l = distorted.rows / 2;
    cv::Mat undistorted(l, l*4, CV_8UC3);
    cv::Mat undistorted_bilinear(l, l*4, CV_8UC3);

    for(int i = 0; i < undistorted.rows; ++i){
      for(int j = 0; j < undistorted.cols; ++j){
	
	double radius = l - i;
	double theta = 2.0 * PI * (double)(-j) / (double)(4.0 * l);
	double fTrueX = radius * cos(theta);
	double fTrueY = radius * sin(theta);

	int x = (int)round(fTrueX) + l;
	int y = l - (int)round(fTrueY);
	if (x >= 0 && x < (2 * l) && y >= 0 && y < (2 * l))
	  {
	    for(int c = 0; c < undistorted.channels(); ++c)
	      undistorted.data[ i * undistorted.step + j * undistorted.elemSize() + c ]
		= distorted.data[x * distorted.step + y * distorted.elemSize() + c];
	  }

	fTrueX = fTrueX + (double)l;
	fTrueY = (double)l - fTrueY;

	int iFloorX = (int)floor(fTrueX);
	int iFloorY = (int)floor(fTrueY);
	int iCeilingX = (int)ceil(fTrueX);
	int iCeilingY = (int)ceil(fTrueY);

	if (iFloorX < 0 || iCeilingX < 0 ||
	    iFloorX >= (2 * l) || iCeilingX >= (2 * l) ||
	    iFloorY < 0 || iCeilingY < 0 ||
	    iFloorY >= (2 * l) || iCeilingY >= (2 * l)) continue;

	double fDeltaX = fTrueX - (double)iFloorX;
	double fDeltaY = fTrueY - (double)iFloorY;

	cv::Mat clrTopLeft(1,1, CV_8UC3), clrTopRight(1,1, CV_8UC3), clrBottomLeft(1,1, CV_8UC3), clrBottomRight(1,1, CV_8UC3);
	for(int c = 0; c < undistorted.channels(); ++c){
	  clrTopLeft.data[ c ] = distorted.data[iFloorX * distorted.step + iFloorY * distorted.elemSize() + c];
	  clrTopRight.data[ c ] = distorted.data[iCeilingX * distorted.step + iFloorY * distorted.elemSize() + c];
	  clrBottomLeft.data[ c ] = distorted.data[iFloorX * distorted.step + iCeilingY * distorted.elemSize() + c];
	  clrBottomRight.data[ c ] = distorted.data[iCeilingX * distorted.step + iCeilingY * distorted.elemSize() + c];
	}

	double fTop0 = interpolate(fDeltaX, clrTopLeft.data[0], clrTopRight.data[0]);
	double fTop1 = interpolate(fDeltaX, clrTopLeft.data[1], clrTopRight.data[1]);
	double fTop2 = interpolate(fDeltaX, clrTopLeft.data[2], clrTopRight.data[2]);
	double fBottom0 = interpolate(fDeltaX, clrBottomLeft.data[0], clrBottomRight.data[0]);
	double fBottom1 = interpolate(fDeltaX, clrBottomLeft.data[1], clrBottomRight.data[1]);
	double fBottom2 = interpolate(fDeltaX, clrBottomLeft.data[2], clrBottomRight.data[2]);

	int i0 = (int)round(interpolate(fDeltaY, fTop0, fBottom0));
	int i1 = (int)round(interpolate(fDeltaY, fTop1, fBottom1));
	int i2 = (int)round(interpolate(fDeltaY, fTop2, fBottom2));

	i0 = std::min(255, std::max(i0, 0));
	i1 = std::min(255, std::max(i1, 0));
	i2 = std::min(255, std::max(i2, 0));
	  
	undistorted_bilinear.data[ i * undistorted_bilinear.step + j * undistorted_bilinear.elemSize() + 0] = i0;
	undistorted_bilinear.data[ i * undistorted_bilinear.step + j * undistorted_bilinear.elemSize() + 1] = i1;
	undistorted_bilinear.data[ i * undistorted_bilinear.step + j * undistorted_bilinear.elemSize() + 2] = i2;
      }
    }

    pub_undistorted_image_.publish(
				   cv_bridge::CvImage(
						      image_msg->header,
						      image_msg->encoding,
						      undistorted).toImageMsg());
    pub_undistorted_bilinear_image_.publish(cv_bridge::CvImage(
							       image_msg->header,
							       image_msg->encoding,
							       undistorted_bilinear).toImageMsg());
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::FisheyeToPanorama, nodelet::Nodelet);
