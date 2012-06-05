#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/color_filter_nodelet.h"


namespace pcl_ros
{
    void ColorFilter::onInit ()
    {
        Filter::onInit ();
        double h_max, h_min, s_max, s_min, v_max, v_min;
        bool filter_limit;
        pnh_->param<double>("h_limit_max", h_max, 2 * M_PI);
        pnh_->param<double>("s_limit_max", s_max, 1.0);
        pnh_->param<double>("v_limit_max", v_max, 1.0);
        pnh_->param<double>("h_limit_min", h_min, 0.0);
        pnh_->param<double>("s_limit_min", s_min, 0.0);
        pnh_->param<double>("v_limit_min", v_min, 0.0);
        pnh_->param<bool>("filter_limit_negative", filter_limit, false);

        impl_.setHueMax(h_max);
        impl_.setSaturationMax(s_max);
        impl_.setValueMax(v_max);
        impl_.setHueMin(h_min);
        impl_.setSaturationMin(s_min);
        impl_.setValueMin(v_min);
        impl_.setFilterLimitsNegative(filter_limit);
    }
    
    bool ColorFilter::child_init (ros::NodeHandle &nh, bool &has_service)
    {
        has_service = true;
        srv_ = boost::make_shared <dynamic_reconfigure::Server<jsk_pcl_ros::ColorFilterConfig> > (nh);
        dynamic_reconfigure::Server<jsk_pcl_ros::ColorFilterConfig>::CallbackType f = boost::bind (&ColorFilter::config_callback, this, _1, _2);
        srv_->setCallback (f);

        return (true);
    }
    
    void ColorFilter::config_callback (jsk_pcl_ros::ColorFilterConfig &config, uint32_t level)
    {
        if (impl_.getFilterLimitsNegative () != config.filter_limit_negative)
            impl_.setFilterLimitsNegative (config.filter_limit_negative);
        
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
    }

    void RGBColorFilter::config_callback (Config &config, uint32_t level)
    {
        if (impl_.getFilterLimitsNegative () != config.filter_limit_negative)
            impl_.setFilterLimitsNegative (config.filter_limit_negative);
    
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
    }

    void HSVColorFilter::config_callback (Config &config, uint32_t level)
    {
        if (impl_.getFilterLimitsNegative () != config.filter_limit_negative)
            impl_.setFilterLimitsNegative (config.filter_limit_negative);
    
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
    }
}
typedef pcl_ros::ColorFilter ColorFilter;
typedef pcl_ros::RGBColorFilter RGBColorFilter;
typedef pcl_ros::HSVColorFilter HSVColorFilter;

PLUGINLIB_DECLARE_CLASS (jsk_pcl, ColorFilter, ColorFilter, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (jsk_pcl, RGBColorFilter, RGBColorFilter, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (jsk_pcl, HSVColorFilter, HSVColorFilter, nodelet::Nodelet);
