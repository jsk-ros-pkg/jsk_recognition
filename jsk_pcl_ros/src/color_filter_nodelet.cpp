#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/color_filter_nodelet.h"

namespace pcl_ros
{
  void RGBColorFilter::config_callback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);

    if (impl_.getRedMax() != config.r_limit_max)
      impl_.setRedMax(config.r_limit_max);

    if (impl_.getGreenMax() != config.g_limit_max)
      impl_.setGreenMax(config.g_limit_max);

    if (impl_.getBlueMax() != config.b_limit_max)
      impl_.setBlueMax(config.b_limit_max);

    if (impl_.getRedMin() != config.r_limit_min)
      impl_.setRedMin(config.r_limit_min);

    if (impl_.getGreenMin() != config.g_limit_min)
      impl_.setGreenMin(config.g_limit_min);

    if (impl_.getBlueMin() != config.b_limit_min)
      impl_.setBlueMin(config.b_limit_min);

    impl_.updateCondition();
  }

  void HSVColorFilter::config_callback (Config &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);

    if (impl_.getHueMax() != config.h_limit_max)
      impl_.setHueMax(config.h_limit_max);

    if (impl_.getSaturationMax() != config.s_limit_max)
      impl_.setSaturationMax(config.s_limit_max);

    if (impl_.getValueMax() != config.v_limit_max)
      impl_.setValueMax(config.v_limit_max);

    if (impl_.getHueMin() != config.h_limit_min)
      impl_.setHueMin(config.h_limit_min);

    if (impl_.getSaturationMin() != config.s_limit_min)
      impl_.setSaturationMin(config.s_limit_min);

    if (impl_.getValueMin() != config.v_limit_min)
      impl_.setValueMin(config.v_limit_min);

    if (impl_.getUseHue() != config.use_h)
      impl_.setUseHue(config.use_h);

    impl_.updateCondition();
  }
}

typedef pcl_ros::RGBColorFilter RGBColorFilter;
typedef pcl_ros::HSVColorFilter HSVColorFilter;

PLUGINLIB_DECLARE_CLASS (jsk_pcl, RGBColorFilter, RGBColorFilter, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (jsk_pcl, HSVColorFilter, HSVColorFilter, nodelet::Nodelet);
