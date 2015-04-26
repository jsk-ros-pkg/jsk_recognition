#include "pcl/pcl_base.h"
#include "pcl/point_types.h"
#include "jsk_pcl_ros/color_converter.h"

#include "pcl/kdtree/tree_types.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"

#include <boost/smart_ptr/make_shared.hpp>

int main (int argc, char** argv) {
    pcl::PointCloud<pcl::PointXYZRGB> p_rgb, p_rgb2;
    pcl::PointCloud<pcl::PointXYZHSV> p_hsv;
    // create RGB pointcloud
    for(int r=0;r < 10; r++) {
        for(int g=0;g < 10; g++) {
            for(int b=0;b < 10; b++) {
                pcl::PointXYZRGB pt;
                pcl::RGBValue v;
                pt.x = r*25;
                pt.y = g*25;
                pt.z = b*25;
                v.Blue = r*25;
                v.Green = g*25;
                v.Red = b*25;
                pt.rgb = v.float_value;
                p_rgb.points.push_back(pt);
            }}}
    // create cv::Mat 1x1000!
    
    
    p_rgb.is_dense = true;
    p_rgb.width = 1000;
    p_rgb.height = 1;
  
    pcl::RGB2HSVConverter<pcl::PointXYZRGB, pcl::PointXYZHSV> rgb2hsv;
    pcl::HSV2RGBConverter<pcl::PointXYZHSV, pcl::PointXYZRGB> hsv2rgb;
  
    pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr tree
        = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZRGB> > ();
  
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr tree2
        = boost::make_shared<pcl::KdTreeFLANN<pcl::PointXYZHSV> > ();
  
    rgb2hsv.setSearchMethod (tree); // dummy
    rgb2hsv.setKSearch (50);        // dummy
  
    hsv2rgb.setSearchMethod (tree2); // dummy
    hsv2rgb.setKSearch (50);         // dummy
  
    rgb2hsv.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> > (p_rgb));
    rgb2hsv.compute(p_hsv);
  
    hsv2rgb.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZHSV> > (p_hsv));
    hsv2rgb.compute(p_rgb2);

    for(int i = 0 ; i < 1000; i++ )
    {
        pcl::PointXYZRGB rgb_p= p_rgb.points[i];
        pcl::PointXYZRGB rgb2_p= p_rgb2.points[i];
        pcl::PointXYZHSV hsv_p= p_hsv.points[i];
      
        pcl::RGBValue v, v2;
        v.float_value = rgb_p.rgb;
        v2.float_value = rgb2_p.rgb;
      
        std::cout << "[" << (int)v.Red << ", " << (int)v.Green << ", " << (int)v.Blue << "] -> "
                  << "[" << hsv_p.hue << ", " << hsv_p.saturation << ", "
                  << hsv_p.value << "] -> "
                  << "[" << (int)v2.Red << ", "
                  << (int)v2.Green << ", "
                  << (int)v2.Blue << "]"
                  << std::endl;
    }
}
