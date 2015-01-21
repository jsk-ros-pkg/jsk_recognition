// -*- mode: C++ -*-

#ifndef IMAGE_RESIZER_NODELET_H_
#define IMAGE_RESIZER_NODELET_H_

#include "resized_image_transport/image_processing_nodelet.h"
#include "resized_image_transport/ImageResizerConfig.h"

namespace resized_image_transport
{
  class ImageResizer : public resized_image_transport::ImageProcessing
  {
  public:
  protected:
    typedef dynamic_reconfigure::Server<ImageResizerConfig> ReconfigureServer;
    ReconfigureServer reconfigure_server_;
    int interpolation_;

  protected:
    void onInit();
    void initReconfigure();
    void initParams();
    void config_cb (ImageResizerConfig &config, uint32_t level);
    void process(const sensor_msgs::ImageConstPtr &src_img, const sensor_msgs::CameraInfoConstPtr &src_info,
		 sensor_msgs::ImagePtr &dst_img, sensor_msgs::CameraInfo &dst_info);
  };
}
#endif
