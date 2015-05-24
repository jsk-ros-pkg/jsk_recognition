// -*- mode: C++ -*-

#ifndef LOG_POLAR_NODELET_H_
#define LOG_POLAR_NODELET_H_

#include "resized_image_transport/image_processing_nodelet.h"
#include "resized_image_transport/LogPolarConfig.h"

namespace resized_image_transport
{
  class LogPolar : public resized_image_transport::ImageProcessing
  {
  public:
  protected:
    typedef dynamic_reconfigure::Server<LogPolarConfig> ReconfigureServer;
    boost::shared_ptr<ReconfigureServer> reconfigure_server_;

    bool inverse_log_polar_;
    double log_polar_scale_;

  protected:
    void onInit();
    void initReconfigure();
    void initParams();
    void config_cb (LogPolarConfig &config, uint32_t level);
    void process(const sensor_msgs::ImageConstPtr &src_img, const sensor_msgs::CameraInfoConstPtr &src_info,
                 sensor_msgs::ImagePtr &dst_img, sensor_msgs::CameraInfo &dst_info);
  };
}
#endif
