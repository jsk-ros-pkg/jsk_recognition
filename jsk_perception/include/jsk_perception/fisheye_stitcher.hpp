/* Copied from https://github.com/drNoob13/fisheyeStitcher/tree/45d65907b63c300523c6143794124411920cdbb9, 2020/02/05 */
/*
MIT License

Copyright (c) 2018-2020 Tuan Phan Minh Ho

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

//----------------------------------------------------------------------------//
//                                                                            //
// This file is part of the fisheye stitcher project.                         //
// Copyright (c) 2018-2020 Tuan Phan Minh Ho <drnoob2013@gmail.com>           //
// https://github.com/drNoob13/fisheyeStitcher                                //
//                                                                            //
//----------------------------------------------------------------------------//
#ifndef FISHEYE_STITCHER_HPP
#define FISHEYE_STITCHER_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <vector> 
#include <tuple>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp> // for imshow
#include <opencv2/calib3d.hpp> // for findHomography
#include "opencv2/stitching/detail/seam_finders.hpp" // seam_finder
#include <opencv2/core/utility.hpp>

#define     MAX_FOVD      195.0f

// Polynomial Coefficients
#define    P1_    -7.5625e-17
#define    P2_     1.9589e-13
#define    P3_    -1.8547e-10
#define    P4_     6.1997e-08
#define    P5_    -6.9432e-05
#define    P6_     0.9976

namespace stitcher
{

class FisheyeStitcher
{
public:
    FisheyeStitcher(int width, int height, float in_fovd, 
                    bool enb_light_compen, bool enb_refine_align,
                    bool save_unwarped, std::string map_path);
    ~FisheyeStitcher();
    cv::Mat stitch(const cv::Mat& image1, const cv::Mat& image2);

private:
    cv::Mat unwarp(const cv::Mat &in_img);
    std::tuple<double, double> fish2Eqt(const double x_dest, 
                                        const double y_dest, 
                                        const double W_rad);
    void fish2Map();
    void createMask();
    cv::Mat deform( const cv::Mat &in_img);
    void genScaleMap();
    cv::Mat compenLightFO(const cv::Mat &in_img);
    void createBlendMask();
    void init();

    cv::Point2f findMatchLoc(const cv::Mat &Ref, 
                             const cv::Mat &Tmpl, 
                             const std::string &img_window, 
                             const bool disable_display);

    std::tuple<std::vector<cv::Point2f>, std::vector<cv::Point2f> > 
        createControlPoints(const cv::Point2f &matchLocLeft, 
            const cv::Point2f &matchLocRight, const int row_start, 
            const int row_end, const int p_wid, const int p_x1, 
            const int p_x2, const int p_x2_ref);

    cv::Mat blendRight(const cv::Mat &bg1, const cv::Mat &bg2);
    cv::Mat blendLeft(const cv::Mat &bg1, const cv::Mat &bg2);
    cv::Mat blend(const cv::Mat &left_img, const cv::Mat &right_img_aligned);

    // Parameters
    int m_hs_org; // height of the input image (2xfisheyes), e.g. 1920
    int m_ws_org; // width of the input image (2xfisheyes), e.g. 3840
    int m_ws;     // width of one fisheye image, e.g. 1920
    int m_hs;     // height of one fisheye image, e.g. 1920
    int m_ws2;    // m_ws / 2 
    int m_hs2;    // m_hs / 2
    int m_hd;     // height of destination pano image
    int m_wd;     // width of destination pano image
    int m_wd2;    // m_wd / 2
    int m_hd2;    // m_hd / 2 
    float m_in_fovd;
    float m_inner_fovd; // used in creating mask
    bool m_enb_light_compen;
    bool m_disable_light_compen;
    bool m_enb_refine_align;
    bool m_save_unwarped;
    std::string m_map_path;  // path of MLS grids
    cv::Mat m_map_x; // used in deformation
    cv::Mat m_map_y; // used in deformation
    cv::Mat m_cir_mask;
    cv::Mat m_inner_cir_mask;
    cv::Mat m_binary_mask;
    std::vector<int> m_blend_post;
    cv::Mat m_scale_map;
    cv::Mat m_mls_map_x;
    cv::Mat m_mls_map_y;

};  // class

}   // namespace

#endif  // FISHEYE_STITCHER_HPP
