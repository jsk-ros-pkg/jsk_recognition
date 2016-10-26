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
#define BOOST_PARAMETER_MAX_ARITY 7
#include "jsk_pcl_ros/find_object_on_plane.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  void FindObjectOnPlane::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_min_area_rect_image_ = advertise<sensor_msgs::Image>(
      *pnh_, "debug/min_area_rect_image", 1);
    onInitPostProcess();
  }

  void FindObjectOnPlane::subscribe()
  {
    sub_image_.subscribe(*pnh_, "input", 1);
    sub_info_.subscribe(*pnh_, "input/camera_info", 1);
    sub_coefficients_.subscribe(*pnh_, "input/coefficients", 1);
    sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
    sync_->connectInput(sub_image_, sub_info_, sub_coefficients_);
    sync_->registerCallback(boost::bind(&FindObjectOnPlane::find,
                                        this, _1, _2, _3));
  }

  void FindObjectOnPlane::unsubscribe()
  {
    sub_image_.unsubscribe();
    sub_coefficients_.unsubscribe();
  }

  void FindObjectOnPlane::find(
    const sensor_msgs::Image::ConstPtr& image_msg,
    const sensor_msgs::CameraInfo::ConstPtr& info_msg,
    const pcl_msgs::ModelCoefficients::ConstPtr& polygon_3d_coefficient_msg)
  {
    cv::Mat object_image = cv_bridge::toCvShare(image_msg, image_msg->encoding)->image;
    
    image_geometry::PinholeCameraModel model;
    pcl::ModelCoefficients::Ptr polygon_coefficients
      (new pcl::ModelCoefficients);
    pcl_conversions::toPCL(*polygon_3d_coefficient_msg, *polygon_coefficients);
    jsk_recognition_utils::Plane::Ptr plane
      (new jsk_recognition_utils::Plane(polygon_coefficients->values));
    model.fromCameraInfo(info_msg);
    std::vector<cv::Point> all_points;
    for (int j = 0; j < object_image.rows; j++) {
      for (int i = 0; i < object_image.cols; i++) {
        if (object_image.at<uchar>(j, i) == 255) {
          all_points.push_back(cv::Point(i, j));
        }
      }
    }
    cv::RotatedRect obb = cv::minAreaRect(all_points);
    
    cv::Mat min_area_rect_image;
    cv::cvtColor(object_image, min_area_rect_image, CV_GRAY2BGR);
    cv::Point2f vertices2f[4];
    obb.points(vertices2f);
    cv::line(min_area_rect_image, vertices2f[0], vertices2f[1],
             cv::Scalar(0, 0, 255), 4);
    cv::line(min_area_rect_image, vertices2f[1], vertices2f[2],
             cv::Scalar(0, 0, 255), 4);
    cv::line(min_area_rect_image, vertices2f[2], vertices2f[3],
             cv::Scalar(0, 0, 255), 4);
    cv::line(min_area_rect_image, vertices2f[3], vertices2f[0],
             cv::Scalar(0, 0, 255), 4);
    cv::Rect bb = obb.boundingRect();
    std::vector<cv::Point3f> search_points_3d;
    std::vector<cv::Point2f> search_points_2d;
    generateStartPoints(cv::Point2f(bb.x, bb.y),
                        model, polygon_coefficients,
                        search_points_3d, search_points_2d);
    for (size_t i = 0; i < search_points_2d.size(); i++) {
      cv::circle(min_area_rect_image, search_points_2d[i], 5, cv::Scalar(0, 255, 0), 1);
    }
    
    //for (size_t i = 0; i < search_points_3d.size(); i++) {
    double min_area = DBL_MAX;
    double min_angle;
    cv::Point2f min_search_point;
    double min_x;
    double min_y;
    for (size_t i = 0; i < search_points_2d.size(); i++) {
      std::vector<double> angles;
      std::vector<double> max_x;
      std::vector<double> max_y;
      generateAngles(object_image, search_points_2d[i], angles,
                     max_x, max_y,
                     model, plane);
      // draw angles
      for (size_t j = 0; j < angles.size(); j++) {
        double area = drawAngle(min_area_rect_image, search_points_2d[i], angles[j],
                                max_x[j], max_y[j], model, plane, cv::Scalar(0, 255, 0));
        if (area < min_area) {
          min_area = area;
          min_x = max_x[j];
          min_y = max_y[j];
          min_angle = angles[j];
          min_search_point = search_points_2d[i];
        }
      }
    }
    drawAngle(min_area_rect_image, min_search_point, min_angle,
              min_x, min_y, model, plane, cv::Scalar(0, 255, 255));
    // convert the points into 3-D
    pub_min_area_rect_image_.publish(
      cv_bridge::CvImage(image_msg->header,
                         sensor_msgs::image_encodings::BGR8,
                         min_area_rect_image).toImageMsg());
    NODELET_INFO("published");
  }


  void FindObjectOnPlane::generateAngles(
    const cv::Mat& blob_image, const cv::Point2f& test_point,
    std::vector<double>& angles,
    std::vector<double>& max_x,
    std::vector<double>& max_y,
    const image_geometry::PinholeCameraModel& model,
    const jsk_recognition_utils::Plane::Ptr& plane)
  {
    angles.clear();
    const double angle_step = 3;
    std::vector<cv::Point> indices;
    for (int j = 0; j < blob_image.rows; j++) {
        for (int i = 0; i < blob_image.cols; i++) {
          if (blob_image.at<uchar>(j, i) != 0) { // need to check
            indices.push_back(cv::Point(i, j));
          }
        }
    }
    for (double angle = -180; angle < 180; angle += angle_step) {
      Eigen::Vector2f ux(cos(angle/180*M_PI), sin(angle/180*M_PI));
      //Eigen::Vector2f uy(sin(angle/180*M_PI), -cos(angle/180*M_PI));
      cv::Point2d uy_end = getUyEnd(test_point, cv::Point2d(test_point.x + ux[0], test_point.y + ux[1]),
                                    model,
                                    plane);
      Eigen::Vector2f uy(uy_end.x - test_point.x, uy_end.y - test_point.y);

      Eigen::Matrix2f A;
      A << ux[0], uy[0],
        ux[1], uy[1];
      bool excluded = false;
      double max_alpha = -DBL_MAX;
      double max_beta = -DBL_MAX;
      for (size_t i = 0; i < indices.size(); i++) {
        Eigen::Vector2f p_q = Eigen::Vector2f(indices[i].x, indices[i].y) - Eigen::Vector2f(test_point.x, test_point.y);
        Eigen::Vector2f a_b = A.inverse() * p_q;
        double alpha = a_b[0];
        double beta = a_b[1];
        if (alpha < 0 || beta < 0) {
          excluded = true;
          break;
        }
        if (alpha > max_alpha) {
          max_alpha = alpha;
        }
        if (beta > max_beta) {
          max_beta = beta;
        }
        
      }
      if (!excluded) {
        angles.push_back(angle);
        max_x.push_back(max_alpha);
        max_y.push_back(max_beta);
      }
    }
  }

  Eigen::Vector3f FindObjectOnPlane::rayPlaneInteersect(
    const cv::Point3d& ray,
    const jsk_recognition_utils::Plane::Ptr& plane)
  {
    Eigen::Vector3f n = plane->getNormal();
    Eigen::Vector3f d(ray.x, ray.y, ray.z);
    double alpha = - plane->getD() / n.dot(d);
    return alpha * d;
  }
  
  cv::Point2d FindObjectOnPlane::getUyEnd(
    const cv::Point2d& ux_start,
    const cv::Point2d& ux_end,
    const image_geometry::PinholeCameraModel& model,
    const jsk_recognition_utils::Plane::Ptr& plane)
  {
    cv::Point3d start_ray = model.projectPixelTo3dRay(ux_start);
    cv::Point3d end_ray = model.projectPixelTo3dRay(ux_end);
    Eigen::Vector3f ux_start_3d = rayPlaneInteersect(start_ray, plane);
    Eigen::Vector3f ux_end_3d = rayPlaneInteersect(end_ray, plane);
    Eigen::Vector3f ux_3d = ux_end_3d - ux_start_3d;
    Eigen::Vector3f normal = plane->getNormal();
    Eigen::Vector3f uy_3d = normal.cross(ux_3d).normalized();
    Eigen::Vector3f uy_end_3d = ux_start_3d + uy_3d;
    cv::Point2d uy_end = model.project3dToPixel(cv::Point3d(uy_end_3d[0],
                                                            uy_end_3d[1],
                                                            uy_end_3d[2]));
    return uy_end;
  }
  
  double FindObjectOnPlane::drawAngle(
    cv::Mat& out_image, const cv::Point2f& test_point, const double angle,
    const double max_x, const double max_y,
    const image_geometry::PinholeCameraModel& model,
    const jsk_recognition_utils::Plane::Ptr& plane,
    cv::Scalar color)
  {
    Eigen::Vector2f ux(cos(angle/180*M_PI), sin(angle/180*M_PI));
    cv::Point2d uy_end = getUyEnd(
      test_point, cv::Point2d(test_point.x + ux[0], test_point.y + ux[1]),
      model, plane);
    Eigen::Vector2f uy(uy_end.x - test_point.x, uy_end.y - test_point.y);
    //Eigen::Vector2f uy(sin(angle/180*M_PI), -cos(angle/180*M_PI));
    Eigen::Vector2f to_point = ux * max_x + Eigen::Vector2f(test_point.x, test_point.y);
    Eigen::Vector2f to_point2 = uy * max_y + Eigen::Vector2f(test_point.x, test_point.y);
    cv::Point2f to_point_cv(to_point[0], to_point[1]);
    cv::Point2f to_point2_cv(to_point2[0], to_point2[1]);
    cv::Point2f to_point3_cv = to_point_cv + to_point2_cv - test_point;
    cv::line(out_image, test_point, to_point_cv, color, 4);
    cv::line(out_image, test_point, to_point2_cv, color, 4);
    cv::line(out_image, to_point_cv, to_point3_cv, color, 4);
    cv::line(out_image, to_point2_cv, to_point3_cv, color, 4);
    return max_x * max_y;       // TODO: it's not correct!
  }

  void FindObjectOnPlane::generateStartPoints(
    const cv::Point2f& point_2d,
    const image_geometry::PinholeCameraModel& model,
    const pcl::ModelCoefficients::Ptr& coefficients,
    std::vector<cv::Point3f>& search_points_3d,
    std::vector<cv::Point2f>& search_points_2d)
  {
    NODELET_INFO("generateStartPoints");
    jsk_recognition_utils::Plane::Ptr plane
      (new jsk_recognition_utils::Plane(coefficients->values));
    cv::Point3d ray = model.projectPixelTo3dRay(point_2d);
    Eigen::Vector3f projected_point = rayPlaneInteersect(ray, plane);
    const double resolution_step = 0.01;
    const int resolution = 5;
    search_points_3d.clear();
    search_points_2d.clear();
    for (int i = - resolution; i < resolution; ++i) {
      for (int j = - resolution; j < resolution; ++j) {
        double x_diff = resolution_step * i;
        double y_diff = resolution_step * j;
        Eigen::Vector3f moved_point = projected_point + Eigen::Vector3f(x_diff, y_diff, 0);
        Eigen::Vector3f projected_moved_point;
        plane->project(moved_point, projected_moved_point);
        cv::Point3f projected_moved_point_cv(projected_moved_point[0],
                                             projected_moved_point[1],
                                             projected_moved_point[2]);
        search_points_3d.push_back(cv::Point3f(projected_moved_point_cv));
        cv::Point2d p2d = model.project3dToPixel(projected_moved_point_cv);
        search_points_2d.push_back(p2d);
      }
    }
  }
}


#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::FindObjectOnPlane, nodelet::Nodelet);
