////////////////////////////////////////////////////////////////////////////////
//
// JSK CR(ColorRange) Capture
//

#include <ros/ros.h>
#include <ros/names.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <tf/transform_listener.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

#include "SRCalibratedLib.h"
#include "CRLib.h"

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include "cr_capture/CRCaptureConfig.h"

#include "cr_capture/RawCloudData.h"
#include "cr_capture/PullRawData.h"
#include "cr_capture/PixelIndices.h"

class CRCaptureNode {
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber camera_sub_l_, camera_sub_r_, camera_sub_depth_;
  ros::Publisher cloud_pub_;
  ros::Publisher cloud2_pub_;
  ros::Publisher index_pub_;
  ros::ServiceServer rawdata_service_;

  std::string left_ns_, right_ns_, range_ns_;

  // all subscriber
  message_filters::Subscriber<sensor_msgs::Image> image_conf_sub_;
  message_filters::Subscriber<sensor_msgs::Image> image_intent_sub_;
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub_;
  message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image,
				    sensor_msgs::Image, sensor_msgs::CameraInfo> sync_;

  // parameter
  bool calc_pixelpos;
  bool use_images;
  bool pull_raw_data;

  // libraries
  SRCalibratedLib sr_lib_;
  CRLib cr_lib_;

  // dynamic reconfigure
  typedef dynamic_reconfigure::Server<cr_capture::CRCaptureConfig> ReconfigureServer;
  ReconfigureServer reconfigure_server_;

  //buffer
  cr_capture::RawCloudData raw_cloud_;

public:
  CRCaptureNode () : nh_("~"), it_(nh_), sync_(5)
  {
    // Set up dynamic reconfiguration
    ReconfigureServer::CallbackType f = boost::bind(&CRCaptureNode::config_cb, this, _1, _2);
    reconfigure_server_.setCallback(f);

    // parameter
    nh_.param("max_range", sr_lib_.max_range, 5.0);
    ROS_INFO("max_range : %f", sr_lib_.max_range);

    nh_.param("depth_scale", sr_lib_.depth_scale, 1.0); // not using
    ROS_INFO("depth_scale : %f", sr_lib_.depth_scale);

    // set transformation
    double trans_pos[3];
    double trans_quat[4];
    trans_pos[0] = trans_pos[1] = trans_pos[2] = 0;
    if (nh_.hasParam("translation")) {
      XmlRpc::XmlRpcValue param_val;
      nh_.getParam("translation", param_val);
      if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray && param_val.size() == 3) {
        trans_pos[0] = param_val[0];
        trans_pos[1] = param_val[1];
        trans_pos[2] = param_val[2];
      }
    }
    ROS_INFO("translation : [%f, %f, %f]", trans_pos[0], trans_pos[1], trans_pos[2]);
    trans_quat[0] = trans_quat[1] = trans_quat[2] = 0;
    trans_quat[3] = 1;
    if (nh_.hasParam("rotation")) {
      XmlRpc::XmlRpcValue param_val;
      nh_.getParam("rotation", param_val);
      if (param_val.getType() == XmlRpc::XmlRpcValue::TypeArray && param_val.size() == 4) {
        trans_quat[0] = param_val[0];
        trans_quat[1] = param_val[1];
        trans_quat[2] = param_val[2];
        trans_quat[3] = param_val[3];
      }
    }
    ROS_INFO("rotation : [%f, %f, %f, %f]", trans_quat[0], trans_quat[1],
             trans_quat[2], trans_quat[3]);
    tf::Quaternion btq(trans_quat[0], trans_quat[1], trans_quat[2], trans_quat[3]);
    tf::Vector3 btp(trans_pos[0], trans_pos[1], trans_pos[2]);
    cr_lib_.cam_trans.setOrigin(btp);
    cr_lib_.cam_trans.setRotation(btq);

    nh_.param("use_filter", sr_lib_.use_filter, true);
    ROS_INFO("use_filter : %d", sr_lib_.use_filter);
    sr_lib_.edge1 = 40.0; sr_lib_.edge2 = 80; sr_lib_.dilate_times = 1;

    nh_.param("use_smooth", sr_lib_.use_smooth, false);
    ROS_INFO("use_smooth : %d", sr_lib_.use_smooth);
    if (sr_lib_.use_smooth) {
      nh_.param("smooth_size", sr_lib_.smooth_size, 6);
      ROS_INFO("smooth_size : %d", sr_lib_.smooth_size);
      nh_.param("smooth_depth", sr_lib_.smooth_depth, 0.04);
      ROS_INFO("smooth_depth : %f", sr_lib_.smooth_depth);
      sr_lib_.smooth_depth = (sr_lib_.smooth_depth / sr_lib_.max_range) * 0xFFFF;
      nh_.param("smooth_space", sr_lib_.smooth_space, 6.0);
      ROS_INFO("smooth_space : %f", sr_lib_.smooth_space);
    }

    nh_.param("clear_uncolored_points", cr_lib_.clear_uncolored_points, true);
    ROS_INFO("clear_uncolored_points : %d", cr_lib_.clear_uncolored_points);

    nh_.param("short_range", sr_lib_.short_range, false);
    ROS_INFO("short_range : %d", sr_lib_.short_range);

    //
    // ros node setting
    //
    cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> ("color_pcloud", 1);
    cloud2_pub_ = nh_.advertise<sensor_msgs::PointCloud2> ("color_pcloud2", 1);

    left_ns_ = nh_.resolveName("left");
    right_ns_ = nh_.resolveName("right");
    range_ns_ = nh_.resolveName("range");

    camera_sub_l_ = it_.subscribeCamera(left_ns_ + "/image", 1, &CRCaptureNode::cameraleftCB, this);
    camera_sub_r_ = it_.subscribeCamera(right_ns_ + "/image", 1, &CRCaptureNode::camerarightCB, this);

    // use images for confidence, intensity threshold
    nh_.param("use_images", use_images, false);
    ROS_INFO("use_images : %d", use_images);
    if(use_images) {
      // all subscribe
      nh_.param("intensity_threshold", sr_lib_.intensity_threshold, -1);
      if(sr_lib_.intensity_threshold >= 0) {
	ROS_INFO("intensity_threshold : %d", sr_lib_.intensity_threshold);
      }
      nh_.param("confidence_threshold", sr_lib_.confidence_threshold, -1);
      if(sr_lib_.confidence_threshold >= 0) {
	ROS_INFO("confidence_threshold : %d", sr_lib_.confidence_threshold);
      }

      image_conf_sub_.subscribe(nh_, range_ns_ + "/confidence/image_raw", 1);
      image_intent_sub_.subscribe(nh_, range_ns_ + "/intensity/image_raw", 1);
      image_depth_sub_.subscribe(nh_, range_ns_ + "/distance/image_raw16", 1);
      info_sub_.subscribe(nh_, range_ns_ + "/camera_info", 1);

      sync_.connectInput(image_conf_sub_, image_intent_sub_, image_depth_sub_, info_sub_);
      sync_.registerCallback(boost::bind(&CRCaptureNode::cameraallCB, this, _1, _2, _3, _4));
    } else {
      // not all subscribe
      camera_sub_depth_ = it_.subscribeCamera(range_ns_ + "/image", 1, &CRCaptureNode::camerarangeCB, this);
    }

    // pull raw data service
    nh_.param("pull_raw_data", pull_raw_data, false);
    ROS_INFO("pull_raw_data : %d", pull_raw_data);
    if(pull_raw_data) {
      rawdata_service_ = nh_.advertiseService("pull_raw_data", &CRCaptureNode::pullData, this);
    }

    nh_.param("calc_pixel_color", calc_pixelpos, false); // not using
    ROS_INFO("calc_pixel_color : %d", calc_pixelpos);
    if(calc_pixelpos) {
      index_pub_ = nh_.advertise<cr_capture::PixelIndices>("pixel_indices", 1);
    }
  }

  bool
  pullData (cr_capture::PullRawDataRequest &req,
            cr_capture::PullRawDataResponse &res)
  {
    res.data = raw_cloud_;
    return true;
  }

  void
  config_cb (cr_capture::CRCaptureConfig &config, uint32_t level)
  {
    cr_lib_.clear_uncolored_points = config.clear_uncolored;
    sr_lib_.short_range = config.short_range;

    sr_lib_.use_filter   = config.use_filter;
    sr_lib_.edge1        = config.canny_parameter1;
    sr_lib_.edge2        = config.canny_parameter2;
    sr_lib_.dilate_times = config.dilate_times;

    sr_lib_.use_smooth   = config.use_smooth;
    sr_lib_.smooth_size  = config.smooth_size;
    sr_lib_.smooth_space = config.smooth_space;
    sr_lib_.smooth_depth = config.smooth_depth;
    sr_lib_.smooth_depth = (sr_lib_.smooth_depth / sr_lib_.max_range) * 0xFFFF;

    sr_lib_.intensity_threshold = config.intensity_threshold;
    sr_lib_.confidence_threshold = config.confidence_threshold;
  }

  void cameraleftCB(const sensor_msgs::ImageConstPtr &img,
                    const sensor_msgs::CameraInfoConstPtr &info)
  {

    cr_lib_.setLeftImg(img, info);

    if(pull_raw_data)
    {
      raw_cloud_.left_image = *img;
      raw_cloud_.left_info = *info;
    }
  }

  void
  camerarightCB(const sensor_msgs::ImageConstPtr &img,
                const sensor_msgs::CameraInfoConstPtr &info)
  {

    cr_lib_.setRightImg(img, info);

    if(pull_raw_data)
    {
      raw_cloud_.right_image = *img;
      raw_cloud_.right_info = *info;
    }
  }

  void
  cameraallCB (const sensor_msgs::ImageConstPtr &img_c,
               const sensor_msgs::ImageConstPtr &img_i,
               const sensor_msgs::ImageConstPtr &img_d,
               const sensor_msgs::CameraInfoConstPtr &info)
  {
    sr_lib_.setRengeImg(img_c, img_i, img_d, info);

    if(pull_raw_data) {
      raw_cloud_.intensity = *img_i;
      raw_cloud_.confidence = *img_c;
      raw_cloud_.depth16 = *img_d;
      raw_cloud_.range_info = *info;

      raw_cloud_.header = info->header;
    }

    publishCloud( info->header );
  }

  void
  camerarangeCB (const sensor_msgs::ImageConstPtr &img,
                 const sensor_msgs::CameraInfoConstPtr &info)
  {
    sr_lib_.setRengeImg(img, img, img, info);

    if(pull_raw_data) {
      raw_cloud_.depth16 = *img;
      raw_cloud_.range_info = *info;

      raw_cloud_.header = info->header;
    }

    publishCloud( info->header );
  }

  void
  publishCloud (const std_msgs::Header &header)
  {
    sensor_msgs::PointCloud pts;

    sr_lib_.calc3DPoints(pts);

    if ( calc_pixelpos ) {
      cr_capture::PixelIndices pidx;
      cr_lib_.calcColor(pts, sr_lib_.width(), sr_lib_.height(), &pidx);
      index_pub_.publish(pidx);
      if (pull_raw_data)
        raw_cloud_.pixel_indices = pidx;
    } else {
      cr_lib_.calcColor(pts, sr_lib_.width(), sr_lib_.height());
    }

    // advertise
    if (cloud_pub_.getNumSubscribers() > 0)
    {
      pts.header = header;
      sensor_msgs::PointCloudPtr ptr = boost::make_shared <sensor_msgs::PointCloud> (pts);
      cloud_pub_.publish(ptr);
    }
    if (cloud2_pub_.getNumSubscribers() > 0 || pull_raw_data)
    {
      pts.header = header;
      sensor_msgs::PointCloud2 outbuf;
      if(!sensor_msgs::convertPointCloudToPointCloud2 (pts, outbuf))
      {
	ROS_ERROR ("[cr_capture] Conversion from sensor_msgs::PointCloud2 to sensor_msgs::PointCloud failed!");
	return;
      }
      outbuf.width = sr_lib_.width();
      outbuf.height = sr_lib_.height();
      outbuf.row_step = outbuf.width * outbuf.point_step;
      if(pull_raw_data)
	raw_cloud_.point_cloud = outbuf;

      if (cloud2_pub_.getNumSubscribers() > 0 )
      {
	sensor_msgs::PointCloud2Ptr ptr = boost::make_shared <sensor_msgs::PointCloud2> (outbuf);
	cloud2_pub_.publish(ptr);
      }
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cr_capture");

  CRCaptureNode cap_node;

  ros::spin();
  return 0;
}
