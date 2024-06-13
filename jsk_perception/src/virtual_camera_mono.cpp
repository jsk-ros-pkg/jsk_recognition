#include "jsk_perception/virtual_camera_mono.h"

#include <jsk_topic_tools/rosparam_utils.h>
#include <cv_bridge/cv_bridge.h>
#include <string>
#include <vector>

namespace jsk_perception
{
  void VirtualCameraMono::onInit()
  {
    DiagnosticNodelet::onInit();
    pub_ = advertiseCamera(*pnh_, "image", 1);

    dynamic_reconfigure::Server<jsk_perception::VirtualCameraMonoConfig>::CallbackType f =
      boost::bind(&VirtualCameraMono::configCb, this, boost::placeholders::_1, boost::placeholders::_2);
    srv_ = boost::make_shared<dynamic_reconfigure::Server<jsk_perception::VirtualCameraMonoConfig> >(*pnh_);
    srv_->setCallback(f);

    pnh_->param("frame_id", trans_.frame_id_, std::string("/elevator_inside_panel"));
    pnh_->param("child_frame_id", trans_.child_frame_id_, std::string("/virtual_camera_frame"));
    ROS_INFO("VirutalCmaeraMono(%s) frame_id: %s, chid_frame_id: %s", ros::this_node::getName().c_str(), trans_.frame_id_.c_str(), trans_.child_frame_id_.c_str());
    pnh_->param("queue_size", queue_size_, 1);
    std::vector<double> initial_pos, initial_rot;
    if (jsk_topic_tools::readVectorParameter(*pnh_, "initial_pos", initial_pos)) {
      trans_.setOrigin(tf::Vector3(initial_pos[0], initial_pos[1], initial_pos[2]));
    }
    else {
      trans_.setOrigin(tf::Vector3(0.7, 0, 0));
    }
    if (jsk_topic_tools::readVectorParameter(*pnh_, "initial_rot", initial_rot)) {
      trans_.setRotation(tf::Quaternion(initial_rot[0], initial_rot[1], initial_rot[2], initial_rot[3]));
    }
    else {
      trans_.setRotation(tf::Quaternion(0.707, 0, 0, -0.707) * tf::Quaternion(0, 0.707, 0, -0.707));
    }
    ROS_INFO("  initia_pos : %f %f %f", trans_.getOrigin().getX(), trans_.getOrigin().getY(), trans_.getOrigin().getZ());
    ROS_INFO("  initia_rot : %f %f %f %f", trans_.getRotation().getX(), trans_.getRotation().getY(), trans_.getRotation().getZ(), trans_.getRotation().getW());

    poly_.header.frame_id = trans_.frame_id_;
    geometry_msgs::Point32 pt;
    pt.x=0, pt.y=1, pt.z=0; poly_.polygon.points.push_back(pt);
    pt.x=0, pt.y=-1, pt.z=0; poly_.polygon.points.push_back(pt);
    pt.x=0, pt.y=-1, pt.z=-1; poly_.polygon.points.push_back(pt);
    pt.x=0, pt.y=1, pt.z=-1; poly_.polygon.points.push_back(pt);

    // parameter subscriber
    sub_trans_ = nh_->subscribe<geometry_msgs::TransformStamped>("view_point", 1, &VirtualCameraMono::transCb, this);
    sub_poly_ = nh_->subscribe<geometry_msgs::PolygonStamped>("target_polygon", 1, &VirtualCameraMono::polyCb, this);

    onInitPostProcess();
  }

  void VirtualCameraMono::configCb(Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock(mutex_);
    switch (config.interpolation_method) {
      case 0:
        interpolation_method_ = cv::INTER_NEAREST;
        break;
      case 1:
        interpolation_method_ = cv::INTER_LINEAR;
        break;
      case 2:
        interpolation_method_ = cv::INTER_AREA;
        break;
      case 3:
        interpolation_method_ = cv::INTER_CUBIC;
        break;
      case 4:
        interpolation_method_ = cv::INTER_LANCZOS4;
        break;
      default:
        ROS_ERROR("Invalid interpolation method: %d", config.interpolation_method);
        break;
    }
  }

  void VirtualCameraMono::subscribe()
  {
    ROS_INFO("Subscribing to image topic");
    it_.reset(new image_transport::ImageTransport(*nh_));
    sub_ = it_->subscribeCamera("image", queue_size_, &VirtualCameraMono::imageCb, this);
    ros::V_string names = boost::assign::list_of("image");
    jsk_topic_tools::warnNoRemap(names);
  }

  void VirtualCameraMono::unsubscribe()
  {
    ROS_INFO("Unsubscibing from image topic");
    sub_.shutdown();
  }

  void VirtualCameraMono::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg)
  {
    vital_checker_->poke();
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
    ROS_DEBUG("transform image.");
    cv::Mat outimage = image.clone();
    if (TransformImage(image, outimage, trans_, poly_, cam_model_)) {
      ROS_DEBUG("publish image and transform.");
      sensor_msgs::CameraInfo virtual_info = *info_msg;
      cv_ptr->image = outimage;
      sensor_msgs::Image::Ptr img_msg = cv_ptr->toImageMsg();
      img_msg->header.stamp = trans_.stamp_;
      virtual_info.header.stamp = trans_.stamp_;
      img_msg->header.frame_id = trans_.child_frame_id_;
      virtual_info.header.frame_id = trans_.child_frame_id_;
      pub_.publish(*img_msg, virtual_info);
    }
  }

  // subscribe target polygon
  void VirtualCameraMono::polyCb(const geometry_msgs::PolygonStampedConstPtr& poly) { poly_ = *poly; }

  // subscribe virtual camera pose
  void VirtualCameraMono::transCb(const geometry_msgs::TransformStampedConstPtr& tf)
  {
    trans_.frame_id_ = tf->header.frame_id;
    trans_.child_frame_id_ = tf->child_frame_id;
    trans_.setOrigin(tf::Vector3(tf->transform.translation.x,
				 tf->transform.translation.y,
				 tf->transform.translation.z));
    trans_.setRotation(tf::Quaternion(tf->transform.rotation.x, tf->transform.rotation.y,
				      tf->transform.rotation.z, tf->transform.rotation.w));
  }

  // poly is plane only
  bool VirtualCameraMono::TransformImage(cv::Mat src, cv::Mat dest,
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
	tf_listener_.transformPoint(cam_model_.tfFrame(), point, cpoint);
	tf_listener_.transformPoint(trans.frame_id_, point, vpoint);

	tf::Vector3 vpt_vec = trans.inverse() * tf::Vector3(vpoint.point.x, vpoint.point.y, vpoint.point.z);

	// push
	target_poly.push_back(tf::Point(cpoint.point.x, cpoint.point.y, cpoint.point.z));
	target_poly_translated.push_back(tf::Point(vpt_vec.x(),vpt_vec.y(),vpt_vec.z()));
      }

      // warp from (cpoint in camera) to (vpoint in virtual camera)
      cv::Point2f src_pnt[4], dst_pnt[4];
      for(int i = 0; i < 4; i++) {
	cv::Point3d xyz(target_poly[i].x(),target_poly[i].y(),target_poly[i].z());
	cv::Point3d xyz_trans(target_poly_translated[i].x(),
			      target_poly_translated[i].y(),
			      target_poly_translated[i].z());
	cv::Point2d uv,uv_trans;
	uv = cam_model_.project3dToPixel(xyz);
	src_pnt[i] = cv::Point (uv.x, uv.y);
	uv_trans = cam_model_.project3dToPixel(xyz_trans);
	dst_pnt[i] = cv::Point (uv_trans.x, uv_trans.y);
      }

      cv::Mat map_matrix = cv::getPerspectiveTransform (src_pnt, dst_pnt);
      cv::warpPerspective (src, dest, map_matrix, dest.size(), interpolation_method_);
    }
    catch (tf::TransformException e) {
      ROS_WARN_THROTTLE(10, "[virtual_camera_mono] failed to transform image: %s", e.what());
      return false;
    }
    return true;
  }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS (jsk_perception::VirtualCameraMono, nodelet::Nodelet);
