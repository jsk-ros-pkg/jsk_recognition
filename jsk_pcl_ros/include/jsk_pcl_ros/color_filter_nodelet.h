// -*- mode: c++ -*-
#ifndef COLOR_FILTER_NODELET_H_
#define COLOR_FILTER_NODELET_H_

// PCL includes
#include "color_filter.h"

#include "pcl_ros/filters/filter.h"
#include "jsk_pcl_ros/ColorFilterConfig.h"
#include "jsk_pcl_ros/RGBColorFilterConfig.h"
#include "jsk_pcl_ros/HSVColorFilterConfig.h"

namespace pcl_ros
{

    class RGBColorFilter : public Filter
    {
    protected:
        typedef jsk_pcl_ros::RGBColorFilterConfig Config;
        boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
        ros::Publisher indices_pub_;
        
        inline void
        filter (const PointCloud2::ConstPtr &input,
                const IndicesPtr &indices, PointCloud2 &output)
            {
                
                impl_.setInputCloud (input);
                impl_.setIndices (indices);
                pcl::PointIndices output_indices;
                impl_.filterIndices (output, output_indices);
                indices_pub_.publish(output_indices);
            }
        
        bool child_init (ros::NodeHandle &nh, bool &has_service)
            {
                has_service = true;
                srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh);
                dynamic_reconfigure::Server<Config>::CallbackType f =
                    boost::bind (&RGBColorFilter::config_callback, this, _1, _2);
                srv_->setCallback (f);
                indices_pub_ = nh.advertise<pcl::PointIndices>("output_indices",
                                                               max_queue_size_);
                
                return true;
            }
        
        void config_callback (Config &config, uint32_t level);


        pcl::RGBColorFilter<sensor_msgs::PointCloud2> impl_;
    public:
        void onInit ()
            {
                Filter::onInit ();
                double r_max, r_min, g_max, g_min, b_max, b_min;
                bool filter_limit;
                pnh_->param<double>("r_limit_max", r_max, 255);
                pnh_->param<double>("g_limit_max", g_max, 255);
                pnh_->param<double>("b_limit_max", b_max, 255);
                pnh_->param<double>("r_limit_min", r_min, 0);
                pnh_->param<double>("g_limit_min", g_min, 0);
                pnh_->param<double>("b_limit_min", b_min, 0);
                pnh_->param<bool>("filter_limit_negative", filter_limit, false);
                
                impl_.setRedMax(r_max);
                impl_.setGreenMax(g_max);
                impl_.setBlueMax(b_max);
                impl_.setRedMin(r_min);
                impl_.setGreenMin(g_min);
                impl_.setBlueMin(b_min);
                impl_.setFilterLimitsNegative(filter_limit);
            }
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    class HSVColorFilter : public Filter
    {
    protected:
        typedef jsk_pcl_ros::HSVColorFilterConfig Config;
        boost::shared_ptr <dynamic_reconfigure::Server<Config> > srv_;
        ros::Publisher indices_pub_;
        
        inline void
        filter (const PointCloud2::ConstPtr &input,
                const IndicesPtr &indices, PointCloud2 &output)
            {
                impl_.setInputCloud (input);
                impl_.setIndices (indices);
                pcl::PointIndices output_indices;
                impl_.filterIndices (output, output_indices);
                indices_pub_.publish(output_indices);
            }
        
        bool child_init (ros::NodeHandle &nh, bool &has_service)
            {
                has_service = true;
                srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (nh);
                dynamic_reconfigure::Server<Config>::CallbackType f =
                    boost::bind (&HSVColorFilter::config_callback, this, _1, _2);
                srv_->setCallback (f);
                indices_pub_ = nh.advertise<pcl::PointIndices>("output_indices",
                                                               max_queue_size_);
                return true;
            }
        
        void config_callback (Config &config, uint32_t level);


        pcl::HSVColorFilter<sensor_msgs::PointCloud2> impl_;
    public:
        void onInit ()
            {
                Filter::onInit ();
                double h_max, h_min, s_max, s_min, v_max, v_min;
                bool filter_limit, use_h;
                pnh_->param<double>("h_limit_max", h_max, 1.0);
                pnh_->param<double>("s_limit_max", s_max, 1.0);
                pnh_->param<double>("v_limit_max", v_max, 1.0);
                pnh_->param<double>("h_limit_min", h_min, 0);
                pnh_->param<double>("s_limit_min", s_min, 0);
                pnh_->param<double>("v_limit_min", v_min, 0);
                pnh_->param<bool>("filter_limit_negative", filter_limit, false);
                pnh_->param<bool>("use_h", use_h, true);
                
                impl_.setHueMax(h_max);
                impl_.setSaturationMax(s_max);
                impl_.setValueMax(v_max);
                impl_.setHueMin(h_min);
                impl_.setSaturationMin(s_min);
                impl_.setValueMin(v_min);
                impl_.setFilterLimitsNegative(filter_limit);
            }
        
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    
    
    // deprecated
    class ColorFilter : public Filter
    {
    protected:
        boost::shared_ptr <dynamic_reconfigure::Server<jsk_pcl_ros::ColorFilterConfig> > srv_;
        inline void
        filter (const PointCloud2::ConstPtr &input, const IndicesPtr &indices, 
                PointCloud2 &output)
            {
                impl_.setInputCloud (input);
                impl_.setIndices (indices);
                impl_.filter (output);
            }
        bool child_init (ros::NodeHandle &nh, bool &has_service);
        void config_callback (jsk_pcl_ros::ColorFilterConfig &config, uint32_t level);
        
        pcl::ColorFilter<sensor_msgs::PointCloud2> impl_;
    public:
        void onInit ();
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}

#endif
