//
// this program is C++ implementation of posedetectiondb/PointPoseExtraction
//
// Masaho Ishida (ishida@jsk.t.u-tokyo.ac.jp) 2010
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

class PointPoseExtractor
{
  ros::NodeHandle _n;
  ros::Subscriber _sub;
  std::string _matching_frame;
  ros::ServiceClient _client;
  ros::Publisher _pub;
  cv::Mat _template_img;
  std::vector<cv::KeyPoint> _template_keypoints;
  cv::Mat _template_descriptors;
  double _template_width;	// width of template [m]
  double _template_height;	// height of template [m]

  // The maximum allowed reprojection error to treat a point pair as an inlier
  double _reprojection_threshold;
  // threshold on squared ratio of distances between NN and 2nd NN
  double _distanceratio_threshold;

public:
  PointPoseExtractor(){
    ros::NodeHandle local_nh("~");

    double width, height;
    std::string template_filename;

    local_nh.param("child_frame_id", _matching_frame, std::string("matching"));
    local_nh.param("object_width",  width,  0.17);
    local_nh.param("object_height", height, 0.24);
    local_nh.param("template_filename", template_filename, std::string("/home/leus/prog/euslib/jsk/img/kellog-front.jpg"));
    local_nh.param("reprojection_threshold", _reprojection_threshold, 3.0);
    local_nh.param("distanceratio_threshold", _distanceratio_threshold, 0.49);

    local_nh.param("child_frame_id", _matching_frame, std::string("matching"));

    _sub = _n.subscribe("ImageFeature0D", 1,
    			&PointPoseExtractor::imagefeature_cb, this);

    _client = _n.serviceClient<posedetection_msgs::Feature0DDetect>("Feature0DDetect");
    _pub = _n.advertise<posedetection_msgs::ObjectDetection>("ObjectDetection", 10);
    _template_keypoints.clear();

    set_template(template_filename, _template_width, _template_height);
  }

  virtual ~PointPoseExtractor(){
    _sub.shutdown();
    _client.shutdown();
    _pub.shutdown();
  }

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

  int set_template(std::string fname, float width, float height){
    posedetection_msgs::Feature0DDetect srv;
    sensor_msgs::CvBridge bridge;

    _template_img = cv::imread (fname, 1);
    if ( _template_img.empty()) {
      ROS_ERROR ("template picture <%s> cannot read. template picture is not found or uses unsuported format.", fname.c_str());
      return -1;
    }

    ROS_INFO_STREAM("read " << fname << " template image and call " << _client.getService() << " service");
    boost::shared_ptr<IplImage> _template_imgipl =
      boost::shared_ptr<IplImage>(new IplImage (_template_img));
    srv.request.image = *bridge.cvToImgMsg(_template_imgipl.get(), "bgr8");
    if (_client.call(srv)) {
      ROS_INFO_STREAM("get features with " << srv.response.features.scales.size() << " descriptoins");
      features2keypoint (srv.response.features, _template_keypoints, _template_descriptors);
      _template_width = width;
      _template_height = height;
      return 0;
    } else {
      ROS_ERROR("Failed to call service Feature0DDetect");
      return 1;
    }
  }

  void imagefeature_cb (const posedetection_msgs::ImageFeature0DConstPtr& msg){
    std::vector<cv::KeyPoint> sourceimg_keypoints;
    cv::Mat sourceimg_descriptors;
    image_geometry::PinholeCameraModel pcam;
    sensor_msgs::CvBridge bridge;

    if ( _template_keypoints.size()== 0 &&
	 _template_descriptors.empty() ) {
      ROS_ERROR ("Template image was not set.");
      return;
    }

    bridge.fromImage (msg->image, "bgr8");
    IplImage* src_imgipl;
    src_imgipl = bridge.toIpl();
    cv::Mat src_img(src_imgipl);

    features2keypoint (msg->features, sourceimg_keypoints, sourceimg_descriptors);
    pcam.fromCameraInfo(msg->info);

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
    cv::flann::Index *ft = new cv::flann::Index(sourceimg_descriptors, cv::flann::KDTreeIndexParams(1));
    cv::Mat m_indices(_template_descriptors.rows, 2, CV_32S);
    cv::Mat m_dists(_template_descriptors.rows, 2, CV_32F);
    ft->knnSearch(_template_descriptors, m_indices, m_dists, 2, cv::flann::SearchParams(-1) );
    delete ft;

    // matched points
    std::vector<cv::Point2f> pt1, pt2;
    std::vector<int> queryIdxs,trainIdxs;
    for ( unsigned int i = 0; i < _template_keypoints.size(); i++ ) {
      if ( m_dists.at<float>(i,0) < m_dists.at<float>(i,1) * _distanceratio_threshold) {
	queryIdxs.push_back(i);
	trainIdxs.push_back(m_indices.at<int>(i,0));
      }
    }
    cv::KeyPoint::convert(_template_keypoints,pt1,queryIdxs);
    cv::KeyPoint::convert(sourceimg_keypoints,pt2,trainIdxs);

    // draw line
    std::vector<cv::Point2f>::iterator pt1_it = pt1.begin();
    std::vector<cv::Point2f>::iterator pt2_it = pt2.begin();
    for ( ; pt1_it != pt1.end(); ++pt1_it, ++pt2_it ) {
      cv::line(stack_img, *pt1_it, *pt2_it+cv::Point2f(0,_template_img.rows), 
	       CV_RGB(255,0,255), 1,8,0);
    }
    ROS_INFO ("Found %d total matches among %d template keypoints", (int)pt2.size(), (int)_template_keypoints.size());

    cv::Mat H;
    if ( pt1.size() > 4 ) {
      H = cv::findHomography(cv::Mat(pt1), cv::Mat(pt2), CV_RANSAC, _reprojection_threshold);
    }

    if ( !H.empty() ) {
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
                    pcam.distortionCoeffs(), rvec, tvec);

      tf::Transform checktf;
      checktf.setOrigin( tf::Vector3(fT3[0], fT3[1], fT3[2] ) );
      double rx = fR3[0] + M_PI, ry = fR3[1], rz = fR3[2];

      ROS_INFO( "tx: (%0.2lf,%0.2lf,%0.2lf) rx: (%0.2lf,%0.2lf,%0.2lf)",
                fT3[0],fT3[1], fT3[2], fR3[0],fR3[1],fR3[2]);

      tf::Quaternion quat;
      quat.setRPY(rx, ry, rz);
      checktf.setRotation( quat );

      posedetection_msgs::ObjectDetection od;
      std::vector<posedetection_msgs::Object6DPose> vo6p;
      {
	posedetection_msgs::Object6DPose o6p;
	o6p.pose.position.x = fT3[0];
	o6p.pose.position.y = fT3[1];
	o6p.pose.position.z = fT3[2];
	o6p.pose.orientation.w = quat.w();
	o6p.pose.orientation.x = quat.x();
	o6p.pose.orientation.y = quat.y();
	o6p.pose.orientation.z = quat.z();
	vo6p.push_back(o6p);
      }

      od.header.stamp = msg->image.header.stamp;
      od.header.frame_id = msg->image.header.frame_id;
      od.objects = vo6p;
      _pub.publish(od);

      // draw lines araound object
      for (int i = 0; i < corners2d_mat_trans.cols; i++){
	cv::Point2f p1(corners2d_mat_trans.at<cv::Point2f>(0,i).x,
		       corners2d_mat_trans.at<cv::Point2f>(0,i).y+_template_img.rows);
	cv::Point2f p2(corners2d_mat_trans.at<cv::Point2f>(0,(i+1)%corners2d_mat_trans.cols).x,
		       corners2d_mat_trans.at<cv::Point2f>(0,(i+1)%corners2d_mat_trans.cols).y+_template_img.rows);
	cv::line (stack_img, p1, p2, CV_RGB(255, 0, 0),
                  4, // width
                  CV_AA, 0);
      }
    }

    cv::imshow("Matches", stack_img);
    cv::waitKey( 10 );

    return;
  }

};  // the end of definition of class PointPoseExtractor


int main (int argc, char **argv){
  ros::init (argc, argv, "PointPoseExtractor");

  PointPoseExtractor matcher;

  ros::spin();

  return 0;
}

