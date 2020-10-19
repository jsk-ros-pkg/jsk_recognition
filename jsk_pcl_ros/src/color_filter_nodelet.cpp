/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "jsk_pcl_ros/color_filter.h"

#include <pluginlib/class_list_macros.h>

namespace jsk_pcl_ros
{

  /*** RGB ***/

  void RGBColorFilter::onInit()
  {
    r_max_ = 255;
    r_min_ = 0;
    g_max_ = 255;
    g_min_ = 0;
    b_max_ = 255;
    b_min_ = 0;

    ColorFilter::onInit();
  }

  void RGBColorFilter::updateCondition()
  {
    ConditionPtr condp (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    int r_max, r_min, g_max, g_min, b_max, b_min;
    if ( r_max_ >= r_min_ ) {
      r_max = r_max_;
      r_min = r_min_;
    }
    else {
      r_max = r_min_;
      r_min = r_max_;
    }
    if ( g_max_ >= g_min_ ) {
      g_max = g_max_;
      g_min = g_min_;
    }
    else {
      g_max = g_min_;
      g_min = g_max_;
    }
    if ( b_max_ >= b_min_ ) {
      b_max = b_max_;
      b_min = b_min_;
    }
    else {
      b_max = b_min_;
      b_min = b_max_;
    }
    
    { 
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("r", pcl::ComparisonOps::LE, r_max));
      ComparisonPtr ge (new Comparison ("r", pcl::ComparisonOps::GE, r_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }
    { 
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("g", pcl::ComparisonOps::LE, g_max));
      ComparisonPtr ge (new Comparison ("g", pcl::ComparisonOps::GE, g_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }
    {
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("b", pcl::ComparisonOps::LE, b_max));
      ComparisonPtr ge (new Comparison ("b", pcl::ComparisonOps::GE, b_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }

    filter_instance_.setCondition (condp);
  }

  void RGBColorFilter::configCallback(jsk_pcl_ros::RGBColorFilterConfig &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);
    r_max_ = config.r_limit_max;
    r_min_ = config.r_limit_min;
    g_max_ = config.g_limit_max;
    g_min_ = config.g_limit_min;
    b_max_ = config.b_limit_max;
    b_min_ = config.b_limit_min;
    updateCondition();
  }

  void RGBColorFilter::convertToColorSpace(float &x, float &y, float &z,
                                           unsigned char r, unsigned char g, unsigned char b)
  {
    x = r/255.0;
    y = g/255.0;
    z = b/255.0;
  }

  /*** HSI ***/
  void HSIColorFilter::onInit()
  {
    h_max_ =  127;
    h_min_ = -128;
    s_max_ = 255;
    s_min_ = 0;
    i_max_ = 255;
    i_min_ = 0;

    ColorFilter::onInit();
  }

  void HSIColorFilter::updateCondition()
  {
    ConditionPtr condp (new pcl::ConditionAnd<pcl::PointXYZRGB> ());

    int h_max, h_min, s_max, s_min, i_max, i_min;
    if ( h_max_ >= h_min_ ) {
      h_max = h_max_;
      h_min = h_min_;
    }
    else {
      h_max = h_min_;
      h_min = h_max_;
    }
    if ( s_max_ >= s_min_ ) {
      s_max = s_max_;
      s_min = s_min_;
    }
    else {
      s_max = s_min_;
      s_min = s_max_;
    }
    if ( i_max_ >= i_min_ ) {
      i_max = i_max_;
      i_min = i_min_;
    }
    else {
      i_max = i_min_;
      i_min = i_max_;
    }

    { 
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("h", pcl::ComparisonOps::LE, h_max));
      ComparisonPtr ge (new Comparison ("h", pcl::ComparisonOps::GE, h_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }
    { 
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("s", pcl::ComparisonOps::LE, s_max));
      ComparisonPtr ge (new Comparison ("s", pcl::ComparisonOps::GE, s_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }
    {
      ConditionPtr cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
      ComparisonPtr le (new Comparison ("i", pcl::ComparisonOps::LE, i_max));
      ComparisonPtr ge (new Comparison ("i", pcl::ComparisonOps::GE, i_min));
      cond->addComparison (le);
      cond->addComparison (ge);
      condp->addCondition(cond);
    }

    filter_instance_.setCondition (condp);
  }

  void HSIColorFilter::configCallback(jsk_pcl_ros::HSIColorFilterConfig &config, uint32_t level)
  {
    boost::mutex::scoped_lock lock (mutex_);
    h_max_ = config.h_limit_max;
    h_min_ = config.h_limit_min;
    s_max_ = config.s_limit_max;
    s_min_ = config.s_limit_min;
    i_max_ = config.i_limit_max;
    i_min_ = config.i_limit_min;
    updateCondition();
  }

  void HSIColorFilter::convertToColorSpace(float &x, float &y, float &z,
                                           unsigned char r, unsigned char g, unsigned char b)
  {
    // http://pointcloudlibrary.github.io/documentation/conditional__removal_8hpp_source.html
    // definitions taken from http://en.wikipedia.org/wiki/HSL_and_HSI
    float hx = (2.0f * r - g - b) / 4.0f;  // hue x component -127 to 127
    float hy = static_cast<float> (g - b) * 111.0f / 255.0f; // hue y component -111 to 111
    int8_t h_ = static_cast<std::int8_t> (std::atan2(hy, hx) * 128.0f / M_PI);

    int32_t i = (r+g+b)/3; // 0 to 255
    uint8_t i_ = static_cast<std::uint8_t> (i);

    std::int32_t m;  // min(r,g,b)
    m = (r < g) ? r : g;
    m = (m < b) ? m : b;

    uint8_t s_ = static_cast<std::uint8_t> ((i == 0) ? 0 : 255 - (m * 255) / i); // saturation 0 to 255
    //
    x = (s_ * cos(h_ * M_PI / 128.0))/255.0; // h between -128 to 128
    y = (s_ * sin(h_ * M_PI / 128.0))/255.0;
    z = i_/255.0;
  }

  /*** ColorFilter ***/
  template <class PackedComparison, typename Config>
  void ColorFilter<PackedComparison, Config>::filter(const sensor_msgs::PointCloud2ConstPtr &input,
                              const PCLIndicesMsg::ConstPtr& indices)
  {

    boost::mutex::scoped_lock lock (mutex_);
    pcl::PointCloud<pcl::PointXYZRGB> tmp_in, tmp_out;
    sensor_msgs::PointCloud2 out;
    fromROSMsg(*input, tmp_in);

    filter_instance_.setInputCloud (tmp_in.makeShared());
    if (indices) {
      pcl::IndicesPtr vindices;
      vindices.reset(new std::vector<int> (indices->indices));
      filter_instance_.setIndices(vindices);
    }
    filter_instance_.filter(tmp_out);

    if ( color_space_pub_.getNumSubscribers() > 0 ){
      /// publish color spaces
      if ( color_space_msg_.data.size() != tmp_in.points.size()) {
        color_space_msg_.data.resize(16*tmp_in.points.size());
        color_space_msg_.width = tmp_in.points.size();
        color_space_msg_.height = 1;
        color_space_msg_.point_step = 16;
        color_space_msg_.row_step = tmp_in.points.size();
      }
      for (unsigned int j=0; j<tmp_in.points.size(); j++) {
        uint32_t rgb_data_ = *reinterpret_cast<int*> (&tmp_in.points[j].rgb);
        unsigned char r_ = (rgb_data_>>16) & 0x0000ff;
        unsigned char g_ = (rgb_data_>>8)  & 0x0000ff;
        unsigned char b_ = (rgb_data_)     & 0x0000ff;

        float x_, y_, z_;
        convertToColorSpace(x_, y_, z_, r_, g_, b_); // convert rgb to xyz to construct color space

        // skip (set black and move point to origin) if no depth data present
        if ( std::isnan(tmp_in.points[j].x) && std::isnan(tmp_in.points[j].y) && std::isnan(tmp_in.points[j].z)  ) {
          x_ = y_ = z_ = 0;
          r_ = g_ = b_ = 0;
        }
        memcpy((void *)(&(color_space_msg_.data[j*16+0])), (const void *)&x_, sizeof(float));
        memcpy((void *)(&(color_space_msg_.data[j*16+4])), (const void *)&y_, sizeof(float));
        memcpy((void *)(&(color_space_msg_.data[j*16+8])), (const void *)&z_, sizeof(float));
        // skip (set gray) if the point is filtered
        if ( std::isnan(tmp_out.points[j].x) && std::isnan(tmp_out.points[j].y) && std::isnan(tmp_out.points[j].z) ) {
          unsigned char gray_ = 16 + (r_/3 + g_/3 + b_/3)*(255-16)/255;
          r_ = g_ = b_ = gray_;
        }
        // set data to color_space_msg
        unsigned char rgb_packed_[4] = {r_, g_, b_, 0};
        memcpy((void *)(&(color_space_msg_.data[j*16+12])), (const void *)rgb_packed_, 4*sizeof(unsigned char));
      }
      color_space_pub_.publish(color_space_msg_);
    }

    if (tmp_out.points.size() > 0) {
      toROSMsg(tmp_out, out);
      pub_.publish(out);
    }
  }

  template <class PackedComparison, typename Config>
  void ColorFilter<PackedComparison, Config>::filter(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    filter(input, PCLIndicesMsg::ConstPtr());
  }

  template <class PackedComparison, typename Config>
  void ColorFilter<PackedComparison, Config>::onInit()
  {
    ConnectionBasedNodelet::onInit();

    color_space_pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("color_space", 1);
    color_space_msg_.header.frame_id = "/map";
    color_space_msg_.fields.resize(4);
    color_space_msg_.fields[0].name = "x";
    color_space_msg_.fields[0].offset = 0;
    color_space_msg_.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
    color_space_msg_.fields[0].count = 1;
    color_space_msg_.fields[1].name = "y";
    color_space_msg_.fields[1].offset = 4;
    color_space_msg_.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
    color_space_msg_.fields[1].count = 1;
    color_space_msg_.fields[2].name = "z";
    color_space_msg_.fields[2].offset = 8;
    color_space_msg_.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
    color_space_msg_.fields[2].count = 1;
    color_space_msg_.fields[3].name = "rgb";
    color_space_msg_.fields[3].offset = 12;
    color_space_msg_.fields[3].datatype = sensor_msgs::PointField::UINT32;
    color_space_msg_.fields[3].count = 1;

    updateCondition();
    bool keep_organized;
    pnh_->param("keep_organized", keep_organized, false);
    pnh_->param("use_indices", use_indices_, false);
    pub_ = advertise<sensor_msgs::PointCloud2>(*pnh_, "output", 1);

    filter_instance_ = pcl::ConditionalRemoval<pcl::PointXYZRGB>(true);
    filter_instance_.setKeepOrganized(keep_organized);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<Config> > (*pnh_);
    typename dynamic_reconfigure::Server<Config>::CallbackType f =
      boost::bind (&ColorFilter::configCallback, this, _1, _2);
    srv_->setCallback (f);

    onInitPostProcess();
  }

  template <class PackedComparison, typename Config>
  void ColorFilter<PackedComparison, Config>::subscribe()
  {
    sub_input_.subscribe(*pnh_, "input", 1);
    if (use_indices_) {
      sync_ = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(10);
      sub_indices_.subscribe(*pnh_, "indices", 1);
      sync_->connectInput(sub_input_, sub_indices_);
      sync_->registerCallback(boost::bind(&ColorFilter::filter, this, _1, _2));
      //sub_input_ = pnh_->subscribe("input", 1, &RGBColorFilter::filter, this);
    }
    else {
      sub_input_.registerCallback(&ColorFilter::filter, this);
    }
  }

  template <class PackedComparison, typename Config>
  void ColorFilter<PackedComparison, Config>::unsubscribe()
  {
    sub_input_.unsubscribe();
    if (use_indices_) {
      sub_indices_.unsubscribe();
    }
  }
  
}

typedef jsk_pcl_ros::RGBColorFilter RGBColorFilter;
typedef jsk_pcl_ros::HSIColorFilter HSIColorFilter;
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::RGBColorFilter, nodelet::Nodelet);
PLUGINLIB_EXPORT_CLASS (jsk_pcl_ros::HSIColorFilter, nodelet::Nodelet);
