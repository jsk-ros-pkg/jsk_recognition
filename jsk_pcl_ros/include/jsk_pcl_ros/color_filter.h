// -*- mode: c++ -*-
#ifndef __COLOR_FILTER_H__
#define __COLOR_FILTER_H__

#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include "pcl/io/io.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "jsk_pcl_ros/color_converter.h"

namespace pcl
{
    
    template <typename PointT>
    class PointIndicesFilter : public Filter<PointT>
    {
    protected:
        using Filter<PointT>::initCompute;
        using Filter<PointT>::deinitCompute;
        using Filter<PointT>::input_;
        using Filter<PointT>::filter_name_;
        using Filter<PointT>::filter_field_name_;
        using Filter<PointT>::filter_limit_min_;
        using Filter<PointT>::filter_limit_max_;
        using Filter<PointT>::filter_limit_negative_;
        using Filter<PointT>::getClassName;
    public:
        // virtual inline
        // void filter(PointIndices &output)
        //     {
        //         if (!initCompute ()) return;
                
        //         // Copy header at a minimum
        //         output.header = input_->header;
                
        //         // Apply the actual filter
        //         //applyIndicesFilter (output);
                
        //         deinitCompute ();
        //     }
    };

    template <typename PointT>
    struct RGBColorFilterType
    {
        typedef typename pcl::PointCloud<PointT> PCLPointCloud;
        typedef typename pcl::PointCloud<PointT> OutputType;
        typedef PointT PCLPointT;
        typedef typename pcl::traits::fieldList<PointT>::type FieldList;
        static inline void convert
        (const typename OutputType::ConstPtr input, PCLPointCloud* outcloud)
            {
                outcloud = input;
            }
        static inline void outputConvert
        (PCLPointCloud* output_cloud, OutputType* output)
            {
                output = output_cloud;
            }
    };

    template <>
    struct RGBColorFilterType <sensor_msgs::PointCloud2>
    {
        typedef pcl::PointCloud<pcl::PointRGB> PCLPointCloud;
        typedef sensor_msgs::PointCloud2 OutputType;
        typedef PointRGB PCLPointT;
        typedef pcl::traits::fieldList<PointRGB>::type FieldList;
        static inline void convert
        (const  OutputType::ConstPtr input, PCLPointCloud* outcloud)
            {
                // check the input is valid or not
                std::vector<sensor_msgs::PointField> fields;
                int distance_idx = pcl::getFieldIndex (*input, "rgb");
                if (distance_idx == -1)
                {
                    //ROS_INFO("dropping message, because it does not have rgb field");
                    outcloud->width = 0;
                    outcloud->height = 0;
                    outcloud->points.clear();
                }
                else
                {
                    fromROSMsg(*input, *outcloud);
                }
            }
        
        static inline void outputConvert
        (PCLPointCloud* output_cloud, OutputType* output)
            {
                toROSMsg(*output_cloud, *output);
            }
    };
    
    template <typename PointT>
    class RGBColorFilter : public PointIndicesFilter<PointT>
    {
    protected:
        using PointIndicesFilter<PointT>::input_;
        using PointIndicesFilter<PointT>::filter_name_;
        using PointIndicesFilter<PointT>::filter_field_name_;
        using PointIndicesFilter<PointT>::filter_limit_min_;
        using PointIndicesFilter<PointT>::filter_limit_max_;
        using PointIndicesFilter<PointT>::filter_limit_negative_;
        using PointIndicesFilter<PointT>::getClassName;
        using Filter<PointT>::initCompute;
        using Filter<PointT>::deinitCompute;
        
        typedef typename RGBColorFilterType<PointT>::OutputType PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;
        typedef typename RGBColorFilterType<PointT>::PCLPointCloud PCLPointCloud;
        typedef typename RGBColorFilterType<PointT>::PCLPointT PCLPointT;
        typedef typename RGBColorFilterType<PointT>::FieldList FieldList;
        
        unsigned char r_min_, r_max_, b_min_, b_max_, g_min_, g_max_;
        
        inline bool checkRGBRange
        (unsigned char r, unsigned char g, unsigned char b)
            {
                return (r_min_ < r && r < r_max_ &&
                        g_min_ < g && g < g_max_ &&
                        b_min_ < b && b < b_max_);
            }
        
    public:
        RGBColorFilter ()
            {
                r_min_ = b_min_ = g_min_ = 0;
                r_max_ = b_max_ = g_max_ = 255;
                filter_name_ = "RGBColorFilter";
            }
        virtual ~RGBColorFilter (){};
        
        inline void setRedMin (unsigned char r_min) {r_min_ = r_min;}
        inline void setRedMax (unsigned char r_max) {r_max_ = r_max;}
        inline void setGreenMin (unsigned char g_min) {g_min_ = g_min;}
        inline void setGreenMax (unsigned char g_max) {g_max_ = g_max;}
        inline void setBlueMin (unsigned char b_min) {b_min_ = b_min;}
        inline void setBlueMax (unsigned char b_max) {b_max_ = b_max;}
        inline unsigned char getRedMin() { return r_min_; }
        inline unsigned char getRedMax() { return r_max_; }
        inline unsigned char getGreenMin() { return g_min_; }
        inline unsigned char getGreenMax() { return g_max_; }
        inline unsigned char getBlueMin() { return b_min_; }
        inline unsigned char getBlueMax() { return b_max_; }
        
        // TODO: support keep_dense_ parameter
        inline virtual void applyFilter (PointCloud &output)
            {
                PCLPointCloud input_converted, output_converted;
                RGBColorFilterType<PointT>::convert(input_, &input_converted);
                size_t nr_p = 0;
                output_converted.points.resize(input_converted.points.size());
                for ( size_t i = 0; i < input_converted.points.size(); i++)
                {
                    PCLPointT point = input_converted.points[i];
                    RGBValue rgb;
                    rgb.float_value = (point.rgb);
                    unsigned char r = rgb.Red, g = rgb.Green, b = rgb.Blue;
                    if (filter_limit_negative_)
                    {
                        
                        if (checkRGBRange(r, g, b))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            ++nr_p;
                        }
                    }
                    else
                    {
                        if (!(checkRGBRange(r, g, b)))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            ++nr_p;
                        }
                    }
                }
                output_converted.height = 1;
                output_converted.width = nr_p;
                output_converted.points.resize(nr_p);
                RGBColorFilterType<PointT>::outputConvert(&output_converted, &output);
                output.header = input_->header;
                output.is_dense = input_->is_dense;
            }
        
        inline virtual void applyFilter (PointCloud& output,
                                         PointIndices& output_indices)
            {
                PCLPointCloud input_converted, output_converted;
                RGBColorFilterType<PointT>::convert(input_, &input_converted);
                size_t nr_p = 0;
                output_converted.points.resize(input_converted.points.size());
                for ( size_t i = 0; i < input_converted.points.size(); i++)
                {
                    PCLPointT point = input_converted.points[i];
                    RGBValue rgb;
                    rgb.float_value = (point.rgb);
                    unsigned char r = rgb.Red, g = rgb.Green, b = rgb.Blue;
                    if (filter_limit_negative_)
                    {
                        
                        if (checkRGBRange(r, g, b))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            output_indices.indices.push_back(i);
                            ++nr_p;
                        }
                    }
                    else
                    {
                        if (!(checkRGBRange(r, g, b)))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            output_indices.indices.push_back(i);
                            ++nr_p;
                        }
                    }
                }
                output_converted.height = 1;
                output_converted.width = nr_p;
                output_converted.points.resize(nr_p);
                RGBColorFilterType<PointT>::outputConvert(&output_converted, &output);
                output.header = input_->header;
                output_indices.header = input_->header;
                output.is_dense = input_->is_dense;
            }
        
        virtual inline
        void filterIndices (PointCloud& output, PointIndices& output_indices)
            {
                if (!initCompute ()) return;
                
                // Copy header at a minimum
                output.header = input_->header;
                output_indices.header = input_->header;
                // Apply the actual filter
                //applyIndicesFilter (output);
                applyFilter(output, output_indices);
                deinitCompute ();
            }
        
    };
    
    template <typename PointT>
    struct HSVColorFilterType
    {
        typedef typename pcl::PointCloud<PointT> PCLPointCloud;
        typedef typename pcl::PointCloud<PointT> OutputType;
        typedef PointT PCLPointT;
        typedef typename pcl::traits::fieldList<PointT>::type FieldList;
        static inline void convert
        (const typename OutputType::ConstPtr input, PCLPointCloud* outcloud)
            {
                outcloud = input;
            }
        static inline void outputConvert
        (PCLPointCloud* output_cloud, OutputType* output)
            {
                output = output_cloud;
            }
    };

    template <>
    struct HSVColorFilterType <sensor_msgs::PointCloud2>
    {
        typedef pcl::PointCloud<pcl::PointHSV> PCLPointCloud;
        typedef sensor_msgs::PointCloud2 OutputType;
        typedef PointHSV PCLPointT;
        typedef pcl::traits::fieldList<PointHSV>::type FieldList;
        static inline void convert
        (const  OutputType::ConstPtr input, PCLPointCloud* outcloud)
            {
                // check the input is valid or not
                std::vector<sensor_msgs::PointField> fields;
                int distance_idx = pcl::getFieldIndex (*input, "hue");
                
                if (distance_idx == -1)
                {
                    //ROS_DEBUG("dropping message, because it does not have hue field");
                    outcloud->width = 0;
                    outcloud->height = 0;
                    outcloud->points.clear();
                }
                else
                {
                    fromROSMsg(*input, *outcloud); // input -> outcloud
                }
            }
        
        static inline void outputConvert
        (PCLPointCloud* output_cloud, OutputType* output)
            {
                
                    toROSMsg(*output_cloud, *output);
            }
    };
    
    template <typename PointT>
    class HSVColorFilter : public Filter<PointT>
    {
    protected:
        using Filter<PointT>::input_;
        using Filter<PointT>::filter_name_;
        using Filter<PointT>::filter_field_name_;
        using Filter<PointT>::filter_limit_min_;
        using Filter<PointT>::filter_limit_max_;
        using Filter<PointT>::filter_limit_negative_;
        using Filter<PointT>::getClassName;
        using Filter<PointT>::initCompute;
        using Filter<PointT>::deinitCompute;
        
        typedef typename HSVColorFilterType<PointT>::OutputType PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;
        typedef typename HSVColorFilterType<PointT>::PCLPointCloud PCLPointCloud;
        typedef typename HSVColorFilterType<PointT>::PCLPointT PCLPointT;
        typedef typename HSVColorFilterType<PointT>::FieldList FieldList;

        inline bool checkHSVRange
        (float h, float s, float v)
            {
                //ROS_INFO("h: %f, s: %f, v:%f", h, s, v);
                bool ret = true;
                if (use_h_) {
                  if ( (h_min_ < h_max_) ) {
                    if ( !(h_min_ < h && h < h_max_) )
                      ret = false;
                  } else { // h_min_ > h_max_
                    if( !( h_max_ > h || h > h_min_) )
                      ret = false;
                  }
                }
                if (!(s_min_ < s && s < s_max_))
                  ret = false;
                if (!(v_min_ < v && v < v_max_))
                  ret = false;
                return ret;
            }
        
    public:
        HSVColorFilter () : h_min_(0), h_max_(1.0), s_min_(0), s_max_(1.0),
                            v_min_(0), v_max_(1.0), use_h_(true)
            {
                filter_name_ = "HSVColorFilter";
            }
        
        virtual ~HSVColorFilter () {};
        inline void setHueMin (float h_min) {h_min_ = h_min;}
        inline void setHueMax (float h_max) {h_max_ = h_max;}
        inline void setSaturationMin (float s_min) {s_min_ = s_min;}
        inline void setSaturationMax (float s_max) {s_max_ = s_max;}
        inline void setValueMin (float v_min) {v_min_ = v_min;}
        inline void setValueMax (float v_max) {v_max_ = v_max;}
        inline float getHueMin() { return h_min_; }
        inline float getHueMax() { return h_max_; }
        inline float getSaturationMin() { return s_min_; }
        inline float getSaturationMax() { return s_max_; }
        inline float getValueMin() { return v_min_; }
        inline float getValueMax() { return v_max_; }
        inline void setUseHue (bool use_h) {use_h_ = use_h;};
        inline bool getUseHue (void) {return use_h_;};   
        
        // TODO: support keep_dense_ parameter
        virtual inline void applyFilter (PointCloud &output)
            {
                PCLPointCloud input_converted, output_converted;
                HSVColorFilterType<PointT>::convert(input_, &input_converted);
                size_t nr_p = 0;
                output_converted.points.resize(input_converted.points.size());
                
                for ( size_t i = 0; i < input_converted.points.size(); i++)
                {
                    PCLPointT point = input_converted.points[i];
                    float h = point.hue, s = point.saturation, v = point.value;
                    //ROS_INFO("x: %f, y: %f, z: %f", point.x, point.y, point.z);
                    if (filter_limit_negative_)
                    {
                        if (checkHSVRange(h, s, v))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            ++nr_p;
                        }
                        
                    }
                    else
                    {
                        if (!(checkHSVRange(h, s, v)))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            ++nr_p;
                        }
                    }
                }
                
                output_converted.height = 1;
                output_converted.width = nr_p;
                output_converted.points.resize (output_converted.width * output_converted.height);
                HSVColorFilterType<PointT>::outputConvert(&output_converted, &output);
                output.header = input_->header;
                output.is_dense = input_->is_dense;
            }

        virtual inline void applyFilter (PointCloud &output,
                                         PointIndices& output_indices)
            {
                PCLPointCloud input_converted, output_converted;
                HSVColorFilterType<PointT>::convert(input_, &input_converted);
                size_t nr_p = 0;
                output_converted.points.resize(input_converted.points.size());
                
                for ( size_t i = 0; i < input_converted.points.size(); i++)
                {
                    PCLPointT point = input_converted.points[i];
                    float h = point.hue, s = point.saturation, v = point.value;
                    //ROS_INFO("x: %f, y: %f, z: %f", point.x, point.y, point.z);
                    if (filter_limit_negative_)
                    {
                        if (checkHSVRange(h, s, v))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            output_indices.indices.push_back(i);
                            ++nr_p;
                        }
                        
                    }
                    else
                    {
                        if (!(checkHSVRange(h, s, v)))
                        {
                            pcl::for_each_type <FieldList>
                                (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                                 (input_converted.points[i],
                                  output_converted.points[nr_p]));
                            output_indices.indices.push_back(i);
                            ++nr_p;
                        }
                    }
                }
                
                output_converted.height = 1;
                output_converted.width = nr_p;
                output_converted.points.resize (output_converted.width * output_converted.height);
                HSVColorFilterType<PointT>::outputConvert(&output_converted, &output);
                output.header = input_->header;
                output.is_dense = input_->is_dense;
            }
        
        virtual inline
        void filterIndices (PointCloud& output, PointIndices& output_indices)
            {
                if (!initCompute ()) return;
                
                // Copy header at a minimum
                output.header = input_->header;
                output_indices.header = input_->header;
                // Apply the actual filter
                //applyIndicesFilter (output);
                applyFilter(output, output_indices);
                deinitCompute ();
            }
        
    protected:
        float h_min_, h_max_, s_min_, s_max_, v_min_, v_max_;
        bool use_h_;
    };

    // deprecated helper class not to make the specialized ColorFilter
    template <typename PointType>
    struct ColorFilterTypeResolver
    {
        typedef typename pcl::PointCloud<PointType> OutputPCLType;
        typedef typename pcl::PointCloud<PointType> OutputArgType;
        typedef PointType PointT;
        typedef typename pcl::traits::fieldList<PointType>::type FieldList;
        static inline void convert
        (const typename OutputPCLType::ConstPtr input, OutputPCLType* outcloud)
            {
                outcloud = input;
            }
    };

    template <>
    struct ColorFilterTypeResolver<sensor_msgs::PointCloud2>
    {
        typedef pcl::PointCloud<PointRGB> OutputPCLType;
        typedef sensor_msgs::PointCloud2 OutputArgType;
        typedef PointRGB PointT;
        typedef pcl::traits::fieldList<PointRGB>::type FieldList;
        
        static inline void convert
        (const sensor_msgs::PointCloud2::ConstPtr input,
         OutputPCLType* outcloud)
            {
                // check the input is valid or not
                std::vector<sensor_msgs::PointField> fields;
                int distance_idx = pcl::getFieldIndex (*input, "rgb");
                if (distance_idx == -1)
                {
                    //ROS_INFO("dropping message, because it does not have rgb field");
                    outcloud->width = 0;
                    outcloud->height = 0;
                    outcloud->points.clear();
                }
                else
                {
                    fromROSMsg(*input, *outcloud); // input -> outcloud
                }
            }
    };
    
    template <typename PointT>
    class ColorFilter: public Filter<PointT>
    {
        using Filter<PointT>::input_;
        using Filter<PointT>::filter_name_;
        using Filter<PointT>::filter_field_name_;
        using Filter<PointT>::filter_limit_min_;
        using Filter<PointT>::filter_limit_max_;
        using Filter<PointT>::filter_limit_negative_;
        using Filter<PointT>::getClassName;
        
        typedef typename ColorFilterTypeResolver<PointT>::OutputArgType PointCloud;
        typedef typename PointCloud::Ptr PointCloudPtr;
        typedef typename PointCloud::ConstPtr PointCloudConstPtr;
        
    public:
        ColorFilter() :
            h_min_(0.0), h_max_(M_PI * 2.0),
            s_min_(0.0), s_max_(1.0),
            v_min_(0.0), v_max_(1.0)
            {
                filter_name_ = "ColorFilter";
            };
        virtual ~ColorFilter() { };
        
        inline void setHueMin (double h_min) {h_min_ = h_min;};
        inline void setHueMax (double h_max) {h_max_ = h_max;};
        inline void setSaturationMin (double s_min) {s_min_ = s_min;};
        inline void setSaturationMax (double s_max) {s_max_ = s_max;};
        inline void setValueMin (double v_min) {v_min_ = v_min;};
        inline void setValueMax (double v_max) {v_max_ = v_max;};
        inline double getHueMin (void) {return h_min_;};
        inline double getHueMax (void) {return h_max_;};
        inline double getSaturationMin (void) {return s_min_;};
        inline double getSaturationMax (void) {return s_max_;};
        inline double getValueMin() { return v_min_; }
        inline double getValueMax() { return v_max_; }

        virtual void applyFilter (PointCloud &output);
    protected:
        //typedef typename pcl::traits::fieldList<PointT>::type FieldList;
        typedef typename ColorFilterTypeResolver<PointT>::FieldList FieldList;
        typedef typename ColorFilterTypeResolver<PointT>::PointT PCLPointT;
        typedef typename ColorFilterTypeResolver<PointT>::OutputPCLType PCLCloudType;
        double h_min_;
        double h_max_;
        double s_min_;
        double s_max_;
        double v_min_;
        double v_max_;

        virtual std::string getClassName () const
            { return ("ColorFilter"); }
    };

    // utility
    inline std::vector<double>
    RGB2HSV_deprecated (const double r, const double g, const double b)
    {

        double ma = .0, mi = .0, h = .0, s = .0, v = .0;
        if ( r == g && g == b)
        {
            ma = r;
            mi = r;
            h = 0.0;
            v = r;
        }
        else if ( r > g && r > b )
        {
            ma = r;
            mi = std::min(g, b);
            h = 60.0 * (g - b) / (ma - mi) + 0.0;
        }
        else if ( g > b && g >= r )
        {
            ma = g;
            mi = std::min(b, r);
            h = 60.0 * (b - r ) / (ma - mi) + 120.0;
        }
        else if ( b >= r && b >= g )
        {
            ma = b;
            mi = std::min(r, g);
            h = 60.0 * (r - g) / (ma - mi) + 240.0;
        }
    
        if ( ma == 0.0 )
            return std::vector<double>(3, 0.0);
        s = ( ma - mi ) / ma;
        v = ma;
        std::vector<double> ret(3, 0);
        ret[0] = h / 180 * M_PI;    // convert to radian
        ret[1] = s;
        ret[2] = v;
        return ret;
    }

    template <typename PointT>
    bool HSVRangeCheck (const PointT &point,
                        const double h_min_, const double h_max_,
                        const double s_min_, const double s_max_,
                        const double v_min_, const double v_max_)
    {
        ROS_WARN("ColorFilter is deprecated! use {RGB,HSV}ColorFilter instead.");
        const float* rgb_ptr = &(point.rgb);
        const unsigned int rgb = *(unsigned int*)(rgb_ptr);
        const unsigned char r = (rgb >> 16) & 0xFF;
        const unsigned char g = (rgb >> 8) & 0xFF;
        const unsigned char b = (rgb >> 0) & 0xFF;
        std::vector<double> hsv = RGB2HSV_deprecated(r / 256.0, g / 256.0, b / 256.0);
        if ( h_max_ > hsv[0] &&
             h_min_ < hsv[0] &&
             s_max_ > hsv[1] &&
             s_min_ < hsv[1] &&
             v_max_ > hsv[2] &&
             v_min_ < hsv[2] )
            return true;
        else
            return false;
    }

    // implementation
    template <typename PointT>
    void ColorFilter<PointT>::applyFilter (PointCloud &output2)
    {
        PCLCloudType converted_input;
        PCLCloudType output;
        ColorFilterTypeResolver<PointT>::convert (input_, &converted_input);
        
        size_t nr_p = 0;
        for ( size_t i = 0; i < converted_input.points.size(); i++ )
        {
            PCLPointT point;
            point = converted_input.points[i];
            if (filter_limit_negative_)
            {
                if (!HSVRangeCheck<PCLPointT>(point,
                                              h_min_, h_max_,
                                              s_min_, s_max_,
                                              v_min_, v_max_))
                {
                    // inside of HSV range
                    pcl::for_each_type <FieldList>
                        (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                         (converted_input.points[i], output.points[nr_p]));
                    nr_p++;
                }
            }
            else
            {
                if (HSVRangeCheck<PCLPointT>(point,
                                             h_min_, h_max_,
                                             s_min_, s_max_,
                                             v_min_, v_max_))
                {
                    // inside of HSV range
                    pcl::for_each_type <FieldList>
                        (pcl::NdConcatenateFunctor <PCLPointT, PCLPointT>
                         (converted_input.points[i], output.points[nr_p]));
                    nr_p++;
                }
            }
        }
        output.width = nr_p;
        output.points.resize (output.width * output.height);
        
    }
}

#endif
