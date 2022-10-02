#ifndef JSK_PERCEPTION_VIRTUAL_CAMERA_MONO_H_
#define JSK_PERCEPTION_VIRTUAL_CAMERA_MONO_H_

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <jsk_perception/VirtualCameraMonoConfig.h>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <boost/assign.hpp>
#include <boost/thread.hpp>
#include <geometry_msgs/PolygonStamped.h>
#if ( CV_MAJOR_VERSION >= 4)
#else
#include <opencv/cv.hpp>
#include <opencv/highgui.h>
#endif

namespace jsk_perception
{
  class VirtualCameraMono: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    typedef VirtualCameraMonoConfig Config;
    VirtualCameraMono() : DiagnosticNodelet("VirtualCameraMono") {}

  protected:

    virtual void onInit();
    virtual void configCb (Config &config, uint32_t level);    
    virtual void subscribe();
    virtual void unsubscribe();
    virtual void imageCb (const sensor_msgs::ImageConstPtr& image_msg,
                          const sensor_msgs::CameraInfoConstPtr& info_msg);
    virtual void polyCb (const geometry_msgs::PolygonStampedConstPtr& poly);
    virtual void transCb (const geometry_msgs::TransformStampedConstPtr& tf);
    virtual bool TransformImage(cv::Mat src, cv::Mat dest,
		      tf::StampedTransform& trans, geometry_msgs::PolygonStamped& poly,
                                                   image_geometry::PinholeCameraModel& cam_model_);

    image_transport::CameraSubscriber sub_;
    image_transport::CameraPublisher pub_;
    ros::Subscriber sub_trans_, sub_poly_;
    
    boost::shared_ptr<dynamic_reconfigure::Server<Config> > srv_;
    boost::shared_ptr<image_transport::ImageTransport> it_;
    tf::TransformListener tf_listener_;
    image_geometry::PinholeCameraModel cam_model_;
    tf::TransformBroadcaster tf_broadcaster_;

    tf::StampedTransform trans_; // transform to virtual camera
    geometry_msgs::PolygonStamped poly_; // target polygon to transform image
    int interpolation_method_;
    int queue_size_;
    
  private:
    
  };
}

#endif
