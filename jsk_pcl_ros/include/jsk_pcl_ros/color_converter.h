// -*- mode: c++ -*-
#ifndef __COLOR_CONVERTER_H__
#define __COLOR_CONVERTER_H__

#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl/io/io.h"
#include <pcl/features/feature.h>
#include <pcl/ros/point_traits.h>
#include <pcl/ros/for_each_type.h>

#include <jsk_pcl_ros/point_types.h>

namespace pcl
{
    // utility
    typedef union
    {
        struct /*anonymous*/
        {
            unsigned char Blue; // Blue channel
            unsigned char Green; // Green channel
            unsigned char Red; // Red channel
        };
        float float_value;
        long long_value;
    } RGBValue;

    template <typename InPointT, typename OutPointT>
    class ColorConverter : public Feature<InPointT, OutPointT>
    {
        
    public:
        // type declaration
        using Feature<InPointT, OutPointT>::feature_name_;
        using Feature<InPointT, OutPointT>::getClassName;
        using Feature<InPointT, OutPointT>::indices_;
        using Feature<InPointT, OutPointT>::input_;
        using Feature<InPointT, OutPointT>::surface_;
        
        typedef typename Feature<InPointT, OutPointT>::PointCloudOut OutPointCloud;
        typedef typename Feature<InPointT, OutPointT>::PointCloudIn InPointCloud;
        typedef typename InPointCloud::Ptr InPointCloudPtr;
        typedef typename InPointCloud::ConstPtr InPointCloudConstPtr;
        
        typedef typename OutPointCloud::Ptr OutPointCloudPtr;
        typedef typename OutPointCloud::ConstPtr OutPointCloudConstPtr;
        
        ColorConverter () { feature_name_ = "ColorConverter";}
        ~ColorConverter () { }
    protected:
        
        using Feature<InPointT, OutPointT>::fake_indices_;
        virtual void computeFeature (OutPointCloud& output) = 0;
        
        bool fake_surface_;
        bool
        initCompute ()
            {
                // Check if input was set
                if (!input_)
                    return (false);

                // If no point indices have been given, construct a set
                // of indices for the entire input point cloud
                if (!indices_)
                {
                    fake_indices_ = true;
                    std::vector<int> *indices = NULL;
                    try
                    {
                        indices = new std::vector<int> (input_->points.size ());
                    }
                    catch (std::bad_alloc)
                    {
                        ROS_ERROR ("[initCompute] Failed to allocate %zu indices.",
                                   input_->points.size ());
                    }
                    for (size_t i = 0; i < indices->size (); ++i)
                        (*indices)[i] = i; 
                    indices_.reset (indices);
                }
                return (true);
            }

        /** \brief This method should get called after finishing
            the actual computation. */
        bool
        deinitCompute ()
            {
                // Reset the indices
                if (fake_indices_)
                {
                    indices_.reset ();
                    fake_indices_ = false;
                }
                return (true);
            }
        void RGB2HSV(int r, int g, int b, float& fh, float& fs, float& fv)
            {
                // mostly copied from opencv-svn/modules/imgproc/src/color.cpp
                // revision is 4351
                //int bidx = blueIdx, scn = srccn;
                const int hsv_shift = 12;
        
                static const int div_table[] = {
                    0, 1044480, 522240, 348160, 261120, 208896, 174080, 149211,
                    130560, 116053, 104448, 94953, 87040, 80345, 74606, 69632,
                    65280, 61440, 58027, 54973, 52224, 49737, 47476, 45412,
                    43520, 41779, 40172, 38684, 37303, 36017, 34816, 33693,
                    32640, 31651, 30720, 29842, 29013, 28229, 27486, 26782,
                    26112, 25475, 24869, 24290, 23738, 23211, 22706, 22223,
                    21760, 21316, 20890, 20480, 20086, 19707, 19342, 18991,
                    18651, 18324, 18008, 17703, 17408, 17123, 16846, 16579,
                    16320, 16069, 15825, 15589, 15360, 15137, 14921, 14711,
                    14507, 14308, 14115, 13926, 13743, 13565, 13391, 13221,
                    13056, 12895, 12738, 12584, 12434, 12288, 12145, 12006,
                    11869, 11736, 11605, 11478, 11353, 11231, 11111, 10995,
                    10880, 10768, 10658, 10550, 10445, 10341, 10240, 10141,
                    10043, 9947, 9854, 9761, 9671, 9582, 9495, 9410,
                    9326, 9243, 9162, 9082, 9004, 8927, 8852, 8777,
                    8704, 8632, 8561, 8492, 8423, 8356, 8290, 8224,
                    8160, 8097, 8034, 7973, 7913, 7853, 7795, 7737,
                    7680, 7624, 7569, 7514, 7461, 7408, 7355, 7304,
                    7253, 7203, 7154, 7105, 7057, 7010, 6963, 6917,
                    6872, 6827, 6782, 6739, 6695, 6653, 6611, 6569,
                    6528, 6487, 6447, 6408, 6369, 6330, 6292, 6254,
                    6217, 6180, 6144, 6108, 6073, 6037, 6003, 5968,
                    5935, 5901, 5868, 5835, 5803, 5771, 5739, 5708,
                    5677, 5646, 5615, 5585, 5556, 5526, 5497, 5468,
                    5440, 5412, 5384, 5356, 5329, 5302, 5275, 5249,
                    5222, 5196, 5171, 5145, 5120, 5095, 5070, 5046,
                    5022, 4998, 4974, 4950, 4927, 4904, 4881, 4858,
                    4836, 4813, 4791, 4769, 4748, 4726, 4705, 4684,
                    4663, 4642, 4622, 4601, 4581, 4561, 4541, 4522,
                    4502, 4483, 4464, 4445, 4426, 4407, 4389, 4370,
                    4352, 4334, 4316, 4298, 4281, 4263, 4246, 4229,
                    4212, 4195, 4178, 4161, 4145, 4128, 4112, 4096
                };
                //int hr = hrange, hscale = hr == 180 ? 15 : 21;
                int hr = 180, hscale = 15;
                int h, s, v = b;
                int vmin = b, diff;
                int vr, vg;
                    
                v = std::max<int>(v, g);
                v = std::max<int>(v, r);
                vmin = std::min<int>(vmin, g);
                vmin = std::min<int>(vmin, r);
                
                diff = v - vmin;
                vr = v == r ? -1 : 0;
                vg = v == g ? -1 : 0;
                    
                s = diff * div_table[v] >> hsv_shift;
                h = (vr & (g - b)) +
                    (~vr & ((vg & (b - r + 2 * diff))
                            + ((~vg) & (r - g + 4 * diff))));
                h = (h * div_table[diff] * hscale +
                     (1 << (hsv_shift + 6))) >> (7 + hsv_shift);
                
                h += h < 0 ? hr : 0;
                fh = h / 180.0;
                fs = s / 255.0;
                fv = v / 255.0;
            }

        void HSV2RGB(float h, float s, float v, float &r, float &g, float &b)
            {
                if( s == 0 )
                    b = g = r = v;
                else
                {
                    static const int sector_data[][3]=
                        {{1,3,0}, {1,0,2}, {3,0,1}, {0,2,1},
                         {0,1,3}, {2,1,0}};
                    //const float hscale = 6.f/ 180.0;
                    const float hscale = 6.f / 1.0;
                    float tab[4];
                    int sector;
                    h *= hscale;
                    if( h < 0 )
                        do h += 6; while( h < 0 );
                    else if( h >= 6 )
                        do h -= 6; while( h >= 6 );
                    sector = std::floor(h);
                    h -= sector;
            
                    tab[0] = v;
                    tab[1] = v*(1.f - s);
                    tab[2] = v*(1.f - s*h);
                    tab[3] = v*(1.f - s*(1.f - h));
            
                    b = tab[sector_data[sector][0]];
                    g = tab[sector_data[sector][1]];
                    r = tab[sector_data[sector][2]];
                }
            }
    

    public:
        void
        compute (OutPointCloud& output)
            {
                // Copy the header
                output.header = input_->header;

                if (!initCompute ()) 
                {
                    ROS_ERROR ("[pcl::%s::compute] Init failed.",
                               getClassName ().c_str ());
                    output.width = output.height = 0;
                    output.points.clear ();
                    return;
                }

                // If the dataset is empty, just return
                if (input_->points.empty ())
                {
                    output.width = output.height = 0;
                    output.points.clear ();
                    deinitCompute ();
                    return;
                }
                
                // If no search surface has been defined, use the input
                // dataset as the search surface itself
                if (!surface_)
                {
                    fake_surface_ = true;
                    surface_ = input_;
                }

                
                // Resize the output dataset
                if (output.points.size () != indices_->size ())
                    output.points.resize (indices_->size ());
                // Check if the output is dense or not
                if (indices_->size () != input_->points.size ())
                {
                    output.width    = indices_->size ();
                    output.height   = 1;
                    output.is_dense = false;
                }
                else
                {
                    output.width    = input_->width;
                    output.height   = input_->height;
                    output.is_dense = input_->is_dense;
                }

                // Perform the actual feature computation
                computeFeature (output);

                deinitCompute ();

                // Reset the surface
                if (fake_surface_)
                {
                    surface_.reset ();
                    fake_surface_ = false;
                }
            }
        
    };

    template <typename InPointT, typename OutPointT>
    class RGB2HSVConverter : public ColorConverter<InPointT, OutPointT>
    {
        
        using ColorConverter<InPointT, OutPointT>::indices_;
        using ColorConverter<InPointT, OutPointT>::input_;
        using ColorConverter<InPointT, OutPointT>::feature_name_;
        using ColorConverter<InPointT, OutPointT>::getClassName;
        using ColorConverter<InPointT, OutPointT>::use_indices_;
    public:
        typedef typename ColorConverter<InPointT, OutPointT>::InPointCloud InPointCloud;
        typedef typename InPointCloud::Ptr InPointCloudPtr;
        typedef typename InPointCloud::ConstPtr InPointCloudConstPtr;
        typedef typename ColorConverter<InPointT, OutPointT>::OutPointCloud OutPointCloud;
        typedef typename OutPointCloud::Ptr OutPointCloudPtr;
        typedef typename OutPointCloud::ConstPtr OutPointCloudConstPtr;
        
        RGB2HSVConverter () { feature_name_ = "RGB2HSVConverter";}
        ~RGB2HSVConverter () { }
    protected:
        using ColorConverter<InPointT, OutPointT>::RGB2HSV;
        
        void computeFeature (OutPointCloud &output)
            {
                //n *= 3;
                // computation starts from here!
                output.points.resize (input_->points.size ());
                output.width  = input_->width;
                output.height = input_->height;
                output.is_dense = input_->is_dense;
                //ROS_INFO("size: %d", input_->points.size());
                for (size_t i = 0; i < input_->points.size(); i++)
                {
                    InPointT point = input_->points[i];
                    RGBValue rgb;
                    rgb.float_value = (point.rgb);
                    int r = rgb.Red;
                    int b = rgb.Blue;
                    int g = rgb.Green;
                    float fh, fs, fv;                    
                    RGB2HSV(r, g, b, fh, fs, fv);
                    //ROS_INFO("h: %f, s: %f, v: %f", fh, fs, fv);
                    output.points[i].hue = fh;
                    output.points[i].saturation = fs;
                    output.points[i].value = fv;
                    // ROS_INFO("h: %f, s: %f, v: %f",
                    //          output.points[i].hue,
                    //          output.points[i].saturation,
                    //          output.points[i].value);
                        
                }
            }
    };
    
    
    template <typename InPointT, typename OutPointT>
    class HSV2RGBConverter : public ColorConverter<InPointT, OutPointT>
    {
        
        typedef typename ColorConverter<InPointT, OutPointT>::InPointCloud InPointCloud;
        typedef typename InPointCloud::Ptr InPointCloudPtr;
        typedef typename InPointCloud::ConstPtr InPointCloudConstPtr;
        typedef typename ColorConverter<InPointT, OutPointT>::OutPointCloud OutPointCloud;
        typedef typename OutPointCloud::Ptr OutPointCloudPtr;
        typedef typename OutPointCloud::ConstPtr OutPointCloudConstPtr;
    public:
        using ColorConverter<InPointT, OutPointT>::feature_name_;
        using ColorConverter<InPointT, OutPointT>::getClassName;
        using ColorConverter<InPointT, OutPointT>::use_indices_;
        using ColorConverter<InPointT, OutPointT>::indices_;
        using ColorConverter<InPointT, OutPointT>::input_;
        
        HSV2RGBConverter () { feature_name_ = "HSV2RGBConverter";}
        ~HSV2RGBConverter () { }
    protected:
        using ColorConverter<InPointT, OutPointT>::HSV2RGB;
        
        void computeFeature (OutPointCloud &output)
            {
                output.points.resize (input_->points.size ());
                output.width  = input_->width;
                output.height = input_->height;
                output.is_dense = input_->is_dense;
                for (size_t i = 0; i < input_->points.size(); i++)
                {
                    float h = input_->points[i].hue;
                    float s = input_->points[i].saturation;
                    float v = input_->points[i].value;
                    
                    float b, g, r;
                    HSV2RGB(h, s, v, r, g, b);
                    RGBValue rgb;
                    rgb.Blue = b * 255;
                    rgb.Green = g * 255;
                    rgb.Red = r * 255;
                    output.points[i].rgb = rgb.float_value;

                }
            }
    };

}


#endif
