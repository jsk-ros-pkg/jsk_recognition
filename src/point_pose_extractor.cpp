//
// this program is C++ implementation of posedetectiondb/PointPoseExtraction
//
// Masaho Ishida (ishida@jsk.t.u-tokyo.ac.jp) 2010
//
// this code requirs OpenCV r4351 with patch attached at
// https://code.ros.org/trac/opencv/ticket/819
//
#include <ros/ros.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#pragma message "Compiling " __FILE__ "..."
#include <posedetection_msgs/Feature0DDetect.h>
#include <posedetection_msgs/ImageFeature0D.h>
#include <posedetection_msgs/ObjectDetection.h>
#include <posedetection_msgs/Object6DPose.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/tf.h>
#include <boost/shared_ptr.hpp>
#include <vector>

#include <posedetectiondb/SetTemplate.h>


void features2keypoint (posedetection_msgs::Feature0D features,
			std::vector<cv::KeyPoint>& keypoints,
			cv::Mat& descriptors){
  keypoints.resize(features.scales.size());
  descriptors.create(features.scales.size(),features.descriptor_dim,CV_32FC1);
  std::vector<cv::KeyPoint>::iterator keypoint_it = keypoints.begin();
  for ( int i=0; keypoint_it != keypoints.end(); ++keypoint_it, ++i ) {
    *keypoint_it = cv::KeyPoint(cv::Point2f(features.positions[i*2+0],
					    features.positions[i*2+1]),
				features.descriptor_dim, // size
				features.orientations[i], //angle
				0, // resonse
				features.scales[i] // octave
				);
    for (int j = 0; j < features.descriptor_dim; j++){
      descriptors.at<float>(i,j) =
	features.descriptors[i*features.descriptor_dim+j];
    }
  }
}


class Matching_Template
{
private:
  std::string _matching_frame;
  cv::Mat _template_img;
  std::vector<cv::KeyPoint> _template_keypoints;
  cv::Mat _template_descriptors;
  double _template_width;	// width of template [m]
  double _template_height;	// height of template [m]
  tf::Transform _relativepose;
  std::string _window_name;
  // The maximum allowed reprojection error to treat a point pair as an inlier
  double _reprojection_threshold;
  // threshold on squared ratio of distances between NN and 2nd NN
  double _distanceratio_threshold;

public:
  Matching_Template(){
  }
  Matching_Template(cv::Mat img,
		    std::string matching_frame,
		    double template_width,
		    double template_height,
		    tf::Transform relativepose,
		    double reprojection_threshold,
		    double distanceratio_threshold,
		    std::string window_name,
		    bool autosize){

    _template_img = img.clone();
    _matching_frame = matching_frame;
    _template_width = template_width;
    _template_height = template_height;
    _relativepose = relativepose;
    _window_name = window_name;
    _reprojection_threshold = reprojection_threshold;
    _distanceratio_threshold = distanceratio_threshold;
    _template_keypoints.clear();
    cv::namedWindow(_window_name, autosize ? CV_WINDOW_AUTOSIZE : 0);
  }


  virtual ~Matching_Template(){
  }


  std::string get_window_name(){
    return _window_name;
  }


  bool check_template (){
    if (_template_img.empty() && _template_keypoints.size() == 0 &&
	_template_descriptors.empty()){
      return false;
    }
    else {
      return true;
    }
  }


  int set_template(ros::ServiceClient client){
    posedetection_msgs::Feature0DDetect srv;
    sensor_msgs::CvBridge bridge;

    if ( _template_img.empty()) {
      ROS_ERROR ("template picture is empty.");
      return -1;
    }
    ROS_INFO_STREAM("read template image and call " << client.getService() << " service");
    boost::shared_ptr<IplImage> _template_imgipl =
      boost::shared_ptr<IplImage>(new IplImage (_template_img));
    srv.request.image = *bridge.cvToImgMsg(_template_imgipl.get(), "bgr8");
    if (client.call(srv)) {
      ROS_INFO_STREAM("get features with " << srv.response.features.scales.size() << " descriptoins");
      features2keypoint (srv.response.features, _template_keypoints, _template_descriptors);
      return 0;
    } else {
      ROS_ERROR("Failed to call service Feature0DDetect");
      return 1;
    }
  }


  double log_fac( int n )
  {
    double f = 0;
    int i;
    for( i = 1; i <= n; i++ ){
      f += log( i );
    }
    return f;
  }

  int min_inlier( int n, int m, double p_badsupp, double p_badxform )
  {
    double pi, sum;
    int i, j;
    for( j = m+1; j <= n; j++ ){
      sum = 0;
      for( i = j; i <= n; i++ ){
	pi = (i-m) * log( p_badsupp ) + (n-i+m) * log( 1.0 - p_badsupp ) +
	  log_fac( n - m ) - log_fac( i - m ) - log_fac( n - i );
	sum += exp( pi );
      }
      if( sum < p_badxform )
	break;
    }
    return j;
  }


  bool estimate_od (ros::ServiceClient client, cv::Mat src_img,
		    std::vector<cv::KeyPoint> sourceimg_keypoints,
		    image_geometry::PinholeCameraModel pcam,
		    cv::flann::Index* ft, posedetection_msgs::Object6DPose* o6p){

    if ( _template_keypoints.size()== 0 &&
	 _template_descriptors.empty() ){
      set_template(client);
    }
    if ( _template_keypoints.size()== 0 &&
	 _template_descriptors.empty() ) {
      ROS_ERROR ("Template image was not set.");
      return false;
    }

    // stacked image
    cv::Size stack_size  = cv::Size(MAX(src_img.cols,_template_img.cols),
				    src_img.rows+_template_img.rows);
    cv::Mat stack_img(stack_size,CV_8UC3);
    stack_img = cv::Scalar(0);
    cv::Mat stack_img_tmp(stack_img,cv::Rect(0,0,_template_img.cols,_template_img.rows));
    cv::add(stack_img_tmp, _template_img, stack_img_tmp);
    cv::Mat stack_img_src(stack_img,cv::Rect(0,_template_img.rows,src_img.cols,src_img.rows));
    cv::add(stack_img_src, src_img, stack_img_src);


    // matching
    cv::Mat m_indices(_template_descriptors.rows, 2, CV_32S);
    cv::Mat m_dists(_template_descriptors.rows, 2, CV_32F);
    ft->knnSearch(_template_descriptors, m_indices, m_dists, 2, cv::flann::SearchParams(-1) );

    // matched points
    std::vector<cv::Point2f> pt1, pt2;
    std::vector<int> queryIdxs,trainIdxs;
    for ( unsigned int j = 0; j < _template_keypoints.size(); j++ ) {
      if ( m_dists.at<float>(j,0) < m_dists.at<float>(j,1) * _distanceratio_threshold) {
	queryIdxs.push_back(j);
	trainIdxs.push_back(m_indices.at<int>(j,0));
      }
    }
    if ( queryIdxs.size() == 0 ) {
      ROS_WARN_STREAM("could not found matched points with distanceratio(" <<_distanceratio_threshold << ")");
    } else {
      cv::KeyPoint::convert(_template_keypoints,pt1,queryIdxs);
      cv::KeyPoint::convert(sourceimg_keypoints,pt2,trainIdxs);
    }

    ROS_INFO ("Found %d total matches among %d template keypoints", (int)pt2.size(), (int)_template_keypoints.size());

    cv::Mat H;
    std::vector<uchar> mask((int)pt2.size());

    if ( pt1.size() > 4 ) {
      H = cv::findHomography(cv::Mat(pt1), cv::Mat(pt2), mask, CV_RANSAC, _reprojection_threshold);
    }

    // draw line
    std::vector<cv::Point2f>::iterator pt1_it = pt1.begin();
    std::vector<cv::Point2f>::iterator pt2_it = pt2.begin();
    for (int j = 0 ; pt1_it != pt1.end(); ++pt1_it, ++pt2_it, j++ ) {
      if ( mask.at(j)){
	cv::line(stack_img, *pt1_it, *pt2_it+cv::Point2f(0,_template_img.rows),
		 CV_RGB(0,255,0), 1,8,0);
      }
      else {
	cv::line(stack_img, *pt1_it, *pt2_it+cv::Point2f(0,_template_img.rows),
		 CV_RGB(255,0,255), 1,8,0);
      }
    }
    int inlier_sum = 0;
    for (int k=0; k < (int)mask.size(); k++){
      inlier_sum += (int)mask.at(k);
    }

    if ((cv::countNonZero( H ) == 0) || (inlier_sum < min_inlier((int)pt2.size(), 4, 0.10, 0.01))){
      cv::imshow(_window_name, stack_img);
      return false;
    }

    std::string _type;
    char chr[20];
    sprintf(chr, "%d", inlier_sum);
    _type = _matching_frame + "_" + std::string(chr);

    cv::Point2f corners2d[4] = {cv::Point2f(0,0),
				cv::Point2f(_template_img.cols,0),
				cv::Point2f(_template_img.cols,_template_img.rows),
				cv::Point2f(0,_template_img.rows)};
    cv::Mat corners2d_mat (cv::Size(4, 1), CV_32FC2, corners2d);
    cv::Point3f corners3d[4] = {cv::Point3f(0,0,0),
				cv::Point3f(_template_width,0,0),
				cv::Point3f(_template_width,_template_height,0),
				cv::Point3f(0,_template_height,0)};
    cv::Mat corners3d_mat (cv::Size(4, 1), CV_32FC3, corners3d);

    cv::Mat corners2d_mat_trans;

    cv::perspectiveTransform (corners2d_mat, corners2d_mat_trans, H);

    double fR3[3], fT3[3];
    cv::Mat rvec(3, 1, CV_64FC1, fR3);
    cv::Mat tvec(3, 1, CV_64FC1, fT3);

    cv::solvePnP (corners3d_mat, corners2d_mat_trans, pcam.intrinsicMatrix(),
		  pcam.distortionCoeffs(),
		  rvec, tvec);

    tf::Transform checktf;
    checktf.setOrigin( tf::Vector3(fT3[0], fT3[1], fT3[2] ) );
    double rx = fR3[0], ry = fR3[1], rz = fR3[2];

    tf::Quaternion quat;
    double angle = cv::norm(rvec);
    quat.setRotation(tf::Vector3(rx/angle, ry/angle, rz/angle), angle);
    checktf.setRotation( quat );

    checktf.operator*=(_relativepose);

    checktf.getBasis().getEulerYPR(rz, ry, rx);
    ROS_INFO( "tx: (%0.2lf,%0.2lf,%0.2lf) rx: (%0.2lf,%0.2lf,%0.2lf)",
	      checktf.getOrigin().getX(),
	      checktf.getOrigin().getY(),
	      checktf.getOrigin().getZ(),
	      rx, ry, rz);

    o6p->pose.position.x = checktf.getOrigin().getX();
    o6p->pose.position.y = checktf.getOrigin().getY();
    o6p->pose.position.z = checktf.getOrigin().getZ();
    o6p->pose.orientation.w = checktf.getRotation().w();
    o6p->pose.orientation.x = checktf.getRotation().x();
    o6p->pose.orientation.y = checktf.getRotation().y();
    o6p->pose.orientation.z = checktf.getRotation().z();
    o6p->type = _type;

    // draw lines araound object
    for (int j = 0; j < corners2d_mat_trans.cols; j++){
      cv::Point2f p1(corners2d_mat_trans.at<cv::Point2f>(0,j).x,
		     corners2d_mat_trans.at<cv::Point2f>(0,j).y+_template_img.rows);
      cv::Point2f p2(corners2d_mat_trans.at<cv::Point2f>(0,(j+1)%corners2d_mat_trans.cols).x,
		     corners2d_mat_trans.at<cv::Point2f>(0,(j+1)%corners2d_mat_trans.cols).y+_template_img.rows);
      cv::line (stack_img, p1, p2, CV_RGB(255, 0, 0),
		4, // width
		CV_AA, 0);
    }

    // draw coords
    {
      double cfR3[3], cfT3[3];
      cv::Mat crvec(3, 1, CV_64FC1, cfR3);
      cv::Mat ctvec(3, 1, CV_64FC1, cfT3);
      cfT3[0] = checktf.getOrigin().getX();
      cfT3[1] = checktf.getOrigin().getY();
      cfT3[2] = checktf.getOrigin().getZ();
      cfR3[0] = checktf.getRotation().getAxis().x() * checktf.getRotation().getAngle();
      cfR3[1] = checktf.getRotation().getAxis().y() * checktf.getRotation().getAngle();
      cfR3[2] = checktf.getRotation().getAxis().z() * checktf.getRotation().getAngle();

      cv::Point3f coords[4] = {cv::Point3f(0,0,0),
			       cv::Point3f(0.05,0,0),
			       cv::Point3f(0,0.05,0),
			       cv::Point3f(0,0,0.05)};
      cv::Mat coords_mat (cv::Size(4, 1), CV_32FC3, coords);
      std::vector<cv::Point2f> coords_img_points;
      cv::projectPoints(coords_mat, crvec, ctvec, pcam.intrinsicMatrix(),
			pcam.distortionCoeffs(),
			coords_img_points);
      cv::Point2f p1(coords_img_points.at(0).x,
		     coords_img_points.at(0).y+_template_img.rows);
      cv::Point2f p2(coords_img_points.at(1).x,
		     coords_img_points.at(1).y+_template_img.rows);
      cv::Point2f p3(coords_img_points.at(2).x,
		     coords_img_points.at(2).y+_template_img.rows);
      cv::Point2f p4(coords_img_points.at(3).x,
		     coords_img_points.at(3).y+_template_img.rows);
      cv::line (stack_img, p1, p2, CV_RGB(255, 0, 0), 3, CV_AA, 0);
      cv::line (stack_img, p1, p3, CV_RGB(0, 255, 0), 3, CV_AA, 0);
      cv::line (stack_img, p1, p4, CV_RGB(0, 0, 255), 3, CV_AA, 0);
    }

    // write text on image
    {
      std::string text;
      text = "inlier: " + std::string(chr);
      int x, y;
      double scale;
      scale = 2.0;
      x = stack_img.size().width - 400;
      y = _template_img.size().height * 0.7;
      cv::putText (stack_img, text, cv::Point(x, y),
		   0, scale, CV_RGB(0, 255, 0),
		   2, 8, false);
    }

    cv::imshow(_window_name, stack_img);
    return true;
  }
};  // the end of difinition of class Matching_Template


class PointPoseExtractor
{
  ros::NodeHandle _n;
  ros::Subscriber _sub;
  ros::ServiceServer _server;
  ros::ServiceClient _client;
  ros::Publisher _pub;
  double _reprojection_threshold;
  double _distanceratio_threshold;
  bool _autosize;
  std::vector<Matching_Template> _templates;

public:
  PointPoseExtractor(){
    ros::NodeHandle local_nh("~");

    std::string matching_frame;
    double template_width;
    double template_height;
    std::string template_filename;
    std::string window_name;

    local_nh.param("child_frame_id", matching_frame, std::string("matching"));
    local_nh.param("object_width",  template_width,  0.17);
    local_nh.param("object_height", template_height, 0.24);
    local_nh.param("template_filename", template_filename, std::string("/home/leus/prog/euslib/jsk/img/kellog-front.jpg"));
    local_nh.param("reprojection_threshold", _reprojection_threshold, 3.0);
    local_nh.param("distanceratio_threshold", _distanceratio_threshold, 0.49);
    local_nh.param("child_frame_id", matching_frame, std::string("matching"));
    local_nh.param("window_name", window_name, std::string("Matches"));
    local_nh.param("autosize", _autosize, false);

    _sub = _n.subscribe("ImageFeature0D", 1,
    			&PointPoseExtractor::imagefeature_cb, this);
    _client = _n.serviceClient<posedetection_msgs::Feature0DDetect>("Feature0DDetect");
    _pub = _n.advertise<posedetection_msgs::ObjectDetection>("ObjectDetection", 10);
    _server = _n.advertiseService("SetTemplate", &PointPoseExtractor::settemplate_cb, this);

    cv::Mat template_img;
    template_img = cv::imread (template_filename, 1);
    if ( template_img.empty()) {
      ROS_ERROR ("template picture <%s> cannot read. template picture is not found or uses unsuported format.", template_filename.c_str());
    }

    Matching_Template tmplt(template_img, matching_frame, template_width,
    			    template_height,
    			    tf::Transform(tf::Quaternion(0, 0, 0), tf::Vector3(0, 0, 0)),
    			    _reprojection_threshold,
    			    _distanceratio_threshold,
    			    window_name, _autosize);
    _templates.push_back(tmplt);
  }

  virtual ~PointPoseExtractor(){
    _sub.shutdown();
    _client.shutdown();
    _pub.shutdown();
  }


  bool settemplate_cb (posedetectiondb::SetTemplate::Request &req,
			posedetectiondb::SetTemplate::Response &res){
      IplImage* iplimg;
      cv::Mat img;
      sensor_msgs::CvBridge bridge;
      bridge.fromImage (req.image, "bgr8");
      iplimg = bridge.toIpl();
      img = cv::Mat(iplimg);
      tf::Transform transform(tf::Quaternion(req.relativepose.orientation.x,
					     req.relativepose.orientation.y,
					     req.relativepose.orientation.z,
					     req.relativepose.orientation.w),
			      tf::Vector3(req.relativepose.position.x,
					  req.relativepose.position.y,
					  req.relativepose.position.z));
      Matching_Template tmplt(img, req.type,
			      req.dimx, req.dimy,
			      transform,
			      _reprojection_threshold,
			      _distanceratio_threshold,
			      req.type, _autosize);
      _templates.push_back(tmplt);
      return true;
  }


  void imagefeature_cb (const posedetection_msgs::ImageFeature0DConstPtr& msg){
    std::vector<cv::KeyPoint> sourceimg_keypoints;
    cv::Mat sourceimg_descriptors;
    image_geometry::PinholeCameraModel pcam;
    sensor_msgs::CvBridge bridge;
    std::vector<posedetection_msgs::Object6DPose> vo6p;
    posedetection_msgs::ObjectDetection od;

    bridge.fromImage (msg->image, "bgr8");
    IplImage* src_imgipl;
    src_imgipl = bridge.toIpl();
    cv::Mat src_img(src_imgipl);

    // from ros messages to keypoint and pin hole camera model
    features2keypoint (msg->features, sourceimg_keypoints, sourceimg_descriptors);
    pcam.fromCameraInfo(msg->info);

    if ( sourceimg_keypoints.size () < 2 ) {
      ROS_INFO ("The number of keypoints in source image is less than 2");
    }
    else {
      // make KDTree
      cv::flann::Index *ft = new cv::flann::Index(sourceimg_descriptors, cv::flann::KDTreeIndexParams(1));

      // matching and detect object
      for (int i = 0; i < (int)_templates.size(); i++){
	posedetection_msgs::Object6DPose o6p;

	if (_templates.at(i).estimate_od(_client, src_img, sourceimg_keypoints, pcam, ft, &o6p))
	  vo6p.push_back(o6p);
      }
      delete ft;
      if ((int)vo6p.size() != 0){
	od.header.stamp = msg->image.header.stamp;
	od.header.frame_id = msg->image.header.frame_id;
	od.objects = vo6p;
	_pub.publish(od);
      }
    }
    cv::waitKey( 10 );
  }
};  // the end of definition of class PointPoseExtractor


int main (int argc, char **argv){
  ros::init (argc, argv, "PointPoseExtractor");

  PointPoseExtractor matcher;

  ros::spin();

  return 0;
}
