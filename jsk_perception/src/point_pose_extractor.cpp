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
 *     disclaimer in the documentation and/or other materials provided
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
#include "jsk_perception/point_pose_extractor.h"
#include <jsk_topic_tools/log_utils.h>

namespace jsk_perception
{
  PointPoseExtractor::~PointPoseExtractor(){
    _sub.shutdown();
    _client.shutdown();
    _pub.shutdown();
    _pub_agg.shutdown();
  }

  void PointPoseExtractor::initialize () {
    std::string _pose_str;
    double template_width;
    double template_height;
    std::string template_filename;
    std::string window_name;

    pnh_->param("child_frame_id", _child_frame_id, std::string("matching"));
    pnh_->param("object_width",  template_width,  0.06);
    pnh_->param("object_height", template_height, 0.0739);
    pnh_->param("relative_pose", _pose_str, std::string("0 0 0 0 0 0 1"));
    std::string default_template_file_name;
    try {
#ifdef ROSPACK_EXPORT
      rospack::ROSPack rp;
      rospack::Package *p = rp.get_pkg("jsk_perception");
      if (p!=NULL) default_template_file_name = p->path + std::string("/sample/opencv-logo2.png");
#else
      rospack::Rospack rp;
      std::vector<std::string> search_path;
      rp.getSearchPathFromEnv(search_path);
      rp.crawl(search_path, 1);
      std::string path;
      if (rp.find("jsk_perception",path)==true) default_template_file_name = path + std::string("/sample/opencv-logo2.png");
#endif
    } catch (std::runtime_error &e) {
    }
    pnh_->param("template_filename", template_filename, default_template_file_name);
    pnh_->param("reprojection_threshold", _reprojection_threshold, 3.0);
    pnh_->param("distanceratio_threshold", _distanceratio_threshold, 0.49);
    pnh_->param("error_threshold", _err_thr, 50.0);
    pnh_->param("theta_step", _th_step, 5.0);
    pnh_->param("phi_step", _phi_step, 5.0);
    pnh_->param("viewer_window", _viewer, true);
    pnh_->param("window_name", window_name, std::string("sample1"));
    pnh_->param("autosize", _autosize, false);
    pnh_->param("publish_null_object_detection", pnod, false);
    pnh_->param("publish_tf", _publish_tf, false);

    // make one template
    cv::Mat template_img;
    template_img = cv::imread (template_filename, 1);
    if ( template_img.empty()) {
      ROS_ERROR ("template picture <%s> cannot read. template picture is not found or uses unsuported format.", template_filename.c_str());
      return;
    }

    _first_sample_change = false;

    // relative pose
    std::vector<double> rv(7);
    std::istringstream iss(_pose_str);
    tf::Transform transform;
    for(int i=0; i<6; i++)
      iss >> rv[i];

    if (iss.eof()) { // use rpy expression
      transform = tf::Transform(tf::createQuaternionFromRPY(rv[3], rv[4], rv[5]),
                                tf::Vector3(rv[0], rv[1], rv[2]));
    } else {  // use quaternion expression
      iss >> rv[6];
      transform = tf::Transform(tf::Quaternion(rv[3], rv[4], rv[5], rv[6]),
                                tf::Vector3(rv[0], rv[1], rv[2]));
    }

    // add the image to template list
    add_new_template(template_img, window_name, transform,
                     template_width, template_height, _th_step, _phi_step);

  } // initialize


  cv::Mat PointPoseExtractor::make_homography(cv::Mat src, cv::Mat rvec, cv::Mat tvec,
                                              double template_width, double template_height, cv::Size &size){

    cv::Point3f coner[4] = {cv::Point3f(-(template_width/2.0), -(template_height/2.0),0),
                            cv::Point3f(template_width/2.0,-(template_height/2.0),0),
                            cv::Point3f(template_width/2.0,template_height/2.0,0),
                            cv::Point3f(-(template_width/2.0),template_height/2.0,0)};
    cv::Mat coner_mat (cv::Size(4, 1), CV_32FC3, coner);
    std::vector<cv::Point2f> coner_img_points;

    cv::Mat zero_distortion_mat = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::projectPoints(coner_mat, rvec, tvec,
                      pcam.intrinsicMatrix(),
                      zero_distortion_mat, // pcam.distortionCoeffs(),
                      coner_img_points);
    float x_min = 10000, x_max = 0;
    float y_min = 10000, y_max = 0;
    for (int i = 0; i < (int)coner_img_points.size(); i++){
      x_min = std::min(x_min, coner_img_points.at(i).x);
      x_max = std::max(x_max, coner_img_points.at(i).x);
      y_min = std::min(y_min, coner_img_points.at(i).y);
      y_max = std::max(y_max, coner_img_points.at(i).y);
    }

    std::vector<cv::Point2f> coner_img_points_trans;
    for (int i = 0; i < (int)coner_img_points.size(); i++){
      cv::Point2f pt_tmp(coner_img_points.at(i).x - x_min,
                         coner_img_points.at(i).y - y_min);
      coner_img_points_trans.push_back(pt_tmp);
    }

    cv::Point2f template_points[4] = {cv::Point2f(0,0),
                                      cv::Point2f(src.size().width,0),
                                      cv::Point2f(src.size().width,src.size().height),
                                      cv::Point2f(0,src.size().height)};
    cv::Mat template_points_mat (cv::Size(4, 1), CV_32FC2, template_points);

    size = cv::Size(x_max - x_min, y_max - y_min);
    return cv::findHomography(template_points_mat, cv::Mat(coner_img_points_trans), 0, 3);
  }


  int PointPoseExtractor::make_warped_images (cv::Mat src, std::vector<cv::Mat> &imgs,
                                              std::vector<cv:: Mat> &Mvec,
                                              double template_width, double template_height,
                                              double th_step, double phi_step){
    
    std::vector<cv::Size> sizevec;

    for (int i = (int)((-3.14/4.0)/th_step); i*th_step < 3.14/4.0; i++){
      for (int j = (int)((-3.14/4.0)/phi_step); j*phi_step < 3.14/4.0; j++){
        double fR3[3], fT3[3];
        cv::Mat rvec(3, 1, CV_64FC1, fR3);
        cv::Mat tvec(3, 1, CV_64FC1, fT3);

        tf::Quaternion quat;
        quat.setEuler(0, th_step*i, phi_step*j);
        fR3[0] = quat.getAxis().x() * quat.getAngle();
        fR3[1] = quat.getAxis().y() * quat.getAngle();
        fR3[2] = quat.getAxis().z() * quat.getAngle();
        fT3[0] = 0;
        fT3[1] = 0;
        fT3[2] = 0.5;

        cv::Mat M;
        cv::Size size;
        M = make_homography(src, rvec, tvec, template_width, template_height, size);
        //Whenever an H matrix cannot be estimated, cv::findHomography returns an empty one.
        if(!M.empty()){
          Mvec.push_back(M);
          sizevec.push_back(size);
        }
      }
    }
    for (int i = 0; i < (int)Mvec.size(); i++){
      cv::Mat dst;
      cv::warpPerspective(src, dst, Mvec.at(i), sizevec.at(i),
                          CV_INTER_LINEAR, IPL_BORDER_CONSTANT, 0);
      imgs.push_back(dst);
    }
    return 0;
  }

  bool PointPoseExtractor::add_new_template(cv::Mat img, std::string typestr, tf::Transform relative_pose,
                                            double template_width, double template_height,
                                            double theta_step=5.0, double phi_step=5.0)
  {
    std::vector<cv::Mat> imgs;
    std::vector<cv::Mat> Mvec;
    make_warped_images(img, imgs, Mvec,
                       template_width, template_height, theta_step, phi_step);

    for (int i = 0; i < (int)imgs.size(); i++){
      std::string type = typestr;
      if(imgs.size() > 1) {
        char chr[20];
        sprintf(chr, "%d", i);
        type += "_" + std::string(chr);
      }

      Matching_Template * tmplt = 
        new Matching_Template(imgs.at(i), type,
                              img.size().width, img.size().height,
                              template_width, template_height,
                              relative_pose, Mvec.at(i),
                              _reprojection_threshold,
                              _distanceratio_threshold,
                              (_viewer ? type : ""), _autosize);
      _templates.push_back(tmplt);
      if( _viewer )
        {
          cv::namedWindow(type, _autosize ? CV_WINDOW_AUTOSIZE : 0);
          cvSetMouseCallback (type.c_str(), &cvmousecb, this);
        }
    }
    return true;
  }

  bool PointPoseExtractor::settemplate_cb (jsk_perception::SetTemplate::Request &req,
                                           jsk_perception::SetTemplate::Response &res){
    if ( !_initialized ) {
      ROS_WARN("SetTemplate service is available only after receiving input ImageFeature0D");
      return false;
    }
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(req.image, enc::BGR8);
    cv::Mat img(cv_ptr->image);

    tf::Transform transform;
    tf::poseMsgToTF(req.relativepose, transform);

    // add the image to template list
    add_new_template(img, req.type, transform,
                     req.dimx, req.dimy, 1.0, 1.0);
    return true;
  }


  void PointPoseExtractor::imagefeature_cb (const posedetection_msgs::ImageFeature0DConstPtr& msg){
    std::vector<cv::KeyPoint> sourceimg_keypoints;
    cv::Mat sourceimg_descriptors;
    std::vector<posedetection_msgs::Object6DPose> vo6p;
    posedetection_msgs::ObjectDetection od;

    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg->image, enc::BGR8);
    cv::Mat src_img(cv_ptr->image);

    // from ros messages to keypoint and pin hole camera model
    features2keypoint (msg->features, sourceimg_keypoints, sourceimg_descriptors);
    pcam.fromCameraInfo(msg->info);
    if ( cv::countNonZero(pcam.intrinsicMatrix()) == 0 ) {
      ROS_FATAL("intrinsic matrix is zero, your camera info looks invalid");
    }
    if ( !_initialized ) {
      // make template images from camera info
      initialize();
      std::cerr << "initialize templates done" << std::endl;
      _initialized = true;
    }

    if ( sourceimg_keypoints.size () < 2 ) {
      ROS_INFO ("The number of keypoints in source image is less than 2");
    }
    else {
      // make KDTree
      cv::flann::Index *ft = new cv::flann::Index(sourceimg_descriptors, cv::flann::KDTreeIndexParams(1));

      // matching and detect object
      ROS_INFO("_templates size: %d", (int)_templates.size());
      for (int i = 0; i < (int)_templates.size(); i++){
        posedetection_msgs::Object6DPose o6p;

        cv::Mat debug_img;
        if (_templates.at(i)->estimate_od(_client, src_img, sourceimg_keypoints, pcam, _err_thr, debug_img, ft, &o6p))
          vo6p.push_back(o6p);

        // for debug Image topic
        cv_bridge::CvImage out_msg;
        out_msg.header   = msg->image.header;
        out_msg.encoding = "bgr8";
        out_msg.image    = debug_img;
        _debug_pub.publish(out_msg.toImageMsg());
      }
      delete ft;
      if (((int)vo6p.size() != 0) || pnod) {
        od.header.stamp = msg->image.header.stamp;
        od.header.frame_id = msg->image.header.frame_id;
        od.objects = vo6p;
        _pub.publish(od);
        _pub_agg.publish(od);
        // Publish result as geometry_msgs/PoseStamped. But it can only contain one object
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header = od.header;
        if ((int)vo6p.size() != 0) {
          pose_msg.pose = od.objects[0].pose;
          _pub_pose.publish(pose_msg);
        }
        // broadcast tf
        if ( this->_publish_tf ) {
          tf::Transform transform;
          tf::poseMsgToTF(pose_msg.pose, transform);
          _br.sendTransform(
                            tf::StampedTransform(
                                                 transform,
                                                 msg->image.header.stamp,
                                                 msg->image.header.frame_id,
                                                 _child_frame_id
                                                 )
                            );
        }
      }
    }
    // BOOST_FOREACH(Matching_Template* mt, _templates) {
    //   std::cerr << "templates_ -> " << mt << std::endl;
    // }
    cv::waitKey( 10 );
  }

  void PointPoseExtractor::onInit()
  {
    DiagnosticNodelet::onInit();
    dynamic_reconfigure::Server<Config> server;
    dynamic_reconfigure::Server<Config>::CallbackType f;
    f = boost::bind(&PointPoseExtractor::dyn_conf_callback, this, boost::placeholders::_1, boost::placeholders::_2);
    server.setCallback(f);

    it = new image_transport::ImageTransport(*pnh_);
    // Use nh_ instead of pnh_ for backward compatibility.
    // See https://github.com/jsk-ros-pkg/jsk_recognition/pull/2779 and 
    // https://github.com/jsk-ros-pkg/jsk_recognition/pull/2778
    _client = nh_->serviceClient<posedetection_msgs::Feature0DDetect>("Feature0DDetect");
    _pub = advertise<posedetection_msgs::ObjectDetection>(*nh_, "ObjectDetection", 10);
    _pub_agg = advertise<posedetection_msgs::ObjectDetection>(*nh_, "ObjectDetection_agg", 10);
    _pub_pose = advertise<geometry_msgs::PoseStamped>(*nh_, "object_pose", 10);
    _debug_pub = it->advertise("debug_image", 1);
    _server = nh_->advertiseService("SetTemplate", &PointPoseExtractor::settemplate_cb, this);
    _initialized = false;

    onInitPostProcess();
  }

  void PointPoseExtractor::subscribe()
  {
    _sub = nh_->subscribe("ImageFeature0D", 1,
                           &PointPoseExtractor::imagefeature_cb, this);
  }

  void PointPoseExtractor::unsubscribe()
  {
    _sub.shutdown();
  }

  /* callback for dynamic reconfigure */
  void PointPoseExtractor::dyn_conf_callback(Config &config,
                                             uint32_t level) {
    std::cout << "id = " << config.template_id << std::endl;
    std::cout << "lvl = " << level << std::endl;
    if((int)_templates.size() <= config.template_id) {
      ROS_WARN("template_id is invalid");
      config.template_id = 0;
      if(_templates.size() != 0)
        config.frame_id = _templates[0]->_matching_frame;
    } else {
      Matching_Template* tmpl = _templates[config.template_id];
      if(config.frame_id == tmpl->_matching_frame) {
        ROS_WARN("update params");
        tmpl->_reprojection_threshold = config.reprojection_threshold;
        tmpl->_distanceratio_threshold = config.distanceratio_threshold;
        _err_thr = config.error_threshold;
      } else {
        ROS_WARN("get params");
        config.frame_id = tmpl->_matching_frame;
        config.reprojection_threshold = tmpl->_reprojection_threshold;
        config.distanceratio_threshold = tmpl->_distanceratio_threshold;
        config.error_threshold = _err_thr;
      }
    }
  }

};  // the end of definition of class PointPoseExtractor

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::PointPoseExtractor, nodelet::Nodelet);
