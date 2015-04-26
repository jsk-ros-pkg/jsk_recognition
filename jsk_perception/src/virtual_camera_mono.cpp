#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <vector>

#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>

class VirtualCameraMono
{
  ros::NodeHandle nh_,private_nh_;
  image_transport::ImageTransport it_,it_priv_;
  image_transport::CameraSubscriber sub_;
  image_transport::CameraPublisher pub_;
  tf::TransformListener tf_listener_;
  image_geometry::PinholeCameraModel cam_model_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Subscriber sub_trans_, sub_poly_;

  tf::StampedTransform trans_; // transform to virtual camera
  geometry_msgs::PolygonStamped poly_; // target polygon to transform image

public:
  VirtualCameraMono() : private_nh_("~"), it_(nh_), it_priv_(private_nh_)
  {
    std::string image_topic = nh_.resolveName("image");
    sub_ = it_.subscribeCamera(image_topic, 1, &VirtualCameraMono::imageCb, this);
    pub_ = it_priv_.advertiseCamera("image", 1);

    // init parameters : TODO replace ros::param
    trans_.frame_id_ = "/elevator_inside_panel";
    trans_.child_frame_id_ = "/virtual_camera_frame";
    trans_.setOrigin(tf::Vector3(0.7,0,0));
    trans_.setRotation(tf::Quaternion(0.707,0,0,-0.707) * tf::Quaternion(0,0.707,0,-0.707));

    poly_.header.frame_id = "/elevator_inside_panel";
    geometry_msgs::Point32 pt;
    pt.x=0, pt.y=1, pt.z=0; poly_.polygon.points.push_back(pt);
    pt.x=0, pt.y=-1, pt.z=0; poly_.polygon.points.push_back(pt);
    pt.x=0, pt.y=-1, pt.z=-1; poly_.polygon.points.push_back(pt);
    pt.x=0, pt.y=1, pt.z=-1; poly_.polygon.points.push_back(pt);

    // parameter subscriber
    sub_trans_ = nh_.subscribe<geometry_msgs::TransformStamped>("view_point", 1, &VirtualCameraMono::transCb, this);
    sub_poly_ = nh_.subscribe<geometry_msgs::PolygonStamped>("target_polygon", 1, &VirtualCameraMono::polyCb, this);

  }

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    // if nobody listens this topic, do not publish
    if(pub_.getNumSubscribers() == 0)
      return;

    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat image;
    try {
      cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
      image = cv_ptr->image;
    }
    catch (cv_bridge::Exception& ex) {
      ROS_ERROR("[virtual_camera_mono] Failed to convert image");
      return;
    }

    cam_model_.fromCameraInfo(info_msg);

    trans_.stamp_ = ros::Time::now();
    tf_broadcaster_.sendTransform(trans_);

    //
    ROS_INFO("transform image.");
    //IplImage *outimage = cvCloneImage(image); // need to release
    cv::Mat outimage = image.clone();
    if (TransformImage(image, outimage, trans_, poly_, cam_model_)) {
      //
      ROS_INFO("publish image and transform.");
      sensor_msgs::CameraInfo virtual_info = *info_msg;
      //sensor_msgs::Image::Ptr img_msg = bridge_.cvToImgMsg(outimage, "bgr8");
      cv_ptr->image = outimage;
      sensor_msgs::Image::Ptr img_msg = cv_ptr->toImageMsg();
      img_msg->header.stamp = trans_.stamp_;
      virtual_info.header.stamp = trans_.stamp_;
      img_msg->header.frame_id = trans_.child_frame_id_;
      virtual_info.header.frame_id = trans_.child_frame_id_;
      pub_.publish(*img_msg, virtual_info);
    }

    // finalize
    //cvReleaseImage(&outimage);
  }

  // subscribe target polygon
  void polyCb(const geometry_msgs::PolygonStampedConstPtr& poly) { poly_ = *poly; }
  // subscribe virtual camera pose
  void transCb(const geometry_msgs::TransformStampedConstPtr& tf) {
    trans_.frame_id_ = tf->header.frame_id;
    trans_.child_frame_id_ = tf->child_frame_id;
    trans_.setOrigin(tf::Vector3(tf->transform.translation.x,
				 tf->transform.translation.y,
				 tf->transform.translation.z));
    trans_.setRotation(tf::Quaternion(tf->transform.rotation.x, tf->transform.rotation.y,
				      tf->transform.rotation.z, tf->transform.rotation.w));
  }

  // poly is plane only
  bool TransformImage(cv::Mat src, cv::Mat dest,
		      tf::StampedTransform& trans, geometry_msgs::PolygonStamped& poly,
		      image_geometry::PinholeCameraModel& cam_model_)
  {
    try {
      // transform polygon to camera coordinate and virtual camera coordinate
      std::vector<tf::Point> target_poly, target_poly_translated;
      for(std::vector<geometry_msgs::Point32>::iterator pit=poly.polygon.points.begin(); pit != poly.polygon.points.end(); pit++) {
	geometry_msgs::PointStamped point, cpoint, vpoint;
	point.point.x = pit->x, point.point.y = pit->y, point.point.z = pit->z;
	point.header = poly.header;
	tf_listener_.transformPoint(cam_model_.tfFrame() ,point, cpoint);
	tf_listener_.transformPoint(trans.frame_id_ ,point, vpoint);

	tf::Vector3 vpt_vec = trans.inverse() * tf::Vector3(vpoint.point.x, vpoint.point.y, vpoint.point.z);

	// push
	target_poly.push_back(tf::Point(cpoint.point.x, cpoint.point.y, cpoint.point.z));
	target_poly_translated.push_back(tf::Point(vpt_vec.x(),vpt_vec.y(),vpt_vec.z()));
      }

      // warp from (cpoint in camera) to (vpoint in virtual camera)
      CvPoint2D32f src_pnt[4], dst_pnt[4];
      for(int i = 0; i < 4; i++) {
	cv::Point3d xyz(target_poly[i].x(),target_poly[i].y(),target_poly[i].z());
	cv::Point3d xyz_trans(target_poly_translated[i].x(),
			      target_poly_translated[i].y(),
			      target_poly_translated[i].z());
	cv::Point2d uv,uv_trans;
	uv = cam_model_.project3dToPixel(xyz);
	src_pnt[i] = cvPoint2D32f (uv.x, uv.y);
	uv_trans = cam_model_.project3dToPixel(xyz_trans);
	dst_pnt[i] = cvPoint2D32f (uv_trans.x, uv_trans.y);
      }

      CvMat* map_matrix = cvCreateMat (3, 3, CV_32FC1);
      cvGetPerspectiveTransform (src_pnt, dst_pnt, map_matrix);
      cv::Mat map_matrix2(map_matrix);
      

      // unrectified?
      //IplImage* rectified = cvCloneImage(src);
      //cv::Mat from_mat = cv::Mat(src), to_mat = cv::Mat(rectified);
      //cam_model_.rectifyImage(from_mat,to_mat);
      //cvWarpPerspective (rectified, dest, map_matrix, CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS, cvScalarAll (0));
      cv::Mat to_mat = src.clone();
      cam_model_.rectifyImage(src, to_mat);
      cv::warpPerspective (src, dest, map_matrix2, dest.size(), cv::INTER_LINEAR);
      //cvReleaseImage(&rectified);
    } catch ( std::runtime_error e ) {
      // ROS_ERROR("%s",e.what());
      return false;
    }
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_camera_mono");

  VirtualCameraMono vcam;

  ros::spin();
}

