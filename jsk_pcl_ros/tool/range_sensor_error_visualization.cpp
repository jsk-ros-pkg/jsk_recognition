// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, JSK Lab
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

/**
 * @brief
 * Visualize error of sensor_msgs/LaserScan
 */

#define BOOST_PARAMETER_MAX_ARITY 7

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <jsk_recognition_msgs/PlotData.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/accumulators/statistics/count.hpp>

/**
 * @brief
 * class to store sensor value and compute mean, error and stddev and so on.
 */
class OneDataStat
{
public:
  typedef boost::shared_ptr<OneDataStat> Ptr;
  typedef boost::accumulators::accumulator_set<
    double,
    boost::accumulators::stats<boost::accumulators::tag::count,
                               boost::accumulators::tag::mean,
                               boost::accumulators::tag::min,
                               boost::accumulators::tag::max,
                               boost::accumulators::tag::variance> > Accumulator;
  virtual void addData(float r)
  {
    acc_(r);
  }

  virtual double mean() const { return boost::accumulators::mean(acc_); }
  virtual double min() const { return boost::accumulators::min(acc_); }
  virtual double max() const { return boost::accumulators::max(acc_); }
  virtual double count() const { return boost::accumulators::count(acc_); }
  virtual double variance() const { return boost::accumulators::variance(acc_); }
  virtual double stddev() const { return sqrt(variance()); }
  
protected:
  Accumulator acc_;
private:
};

/**
 * @brief
 * wrapper function for mean method for boost::function
 */
double mean(const OneDataStat& d) 
{
  return d.mean();
}

/**
 * @brief
 * wrapper function for min method for boost::function
 */
double min(const OneDataStat& d) 
{
  return d.min();
}

/**
 * @brief
 * wrapper function for max method for boost::function
 */
double max(const OneDataStat& d) 
{
  return d.max();
}

/**
 * @brief
 * wrapper function for count method for boost::function
 */
double count(const OneDataStat& d)
{
  return d.count();
}

/**
 * @brief
 * wrapper function for variance method for boost::function
 */
double variance(const OneDataStat& d)
{
  return d.variance();
}

/**
 * @brief
 * wrapper function for stddev method for boost::function
 */
double stddev(const OneDataStat& d)
{
  return d.stddev();
}


/**
 * @brief
 * generate a float image according to accessor.
 */
void generateImage(const std::vector<OneDataStat::Ptr>& stats,
                   cv::Mat& image,
                   boost::function<double(OneDataStat&)> accessor)
{
  // Scan values in order to compute min and max
  typedef boost::accumulators::accumulator_set<
    double,
    boost::accumulators::stats<boost::accumulators::tag::min,
                               boost::accumulators::tag::max> > MinMaxAccumulator;
  MinMaxAccumulator acc;
  for (size_t i = 0; i < stats.size(); i++) {
    acc(accessor(*stats[i]));
  }
  
  double min = boost::accumulators::min(acc);
  double max = boost::accumulators::max(acc);

  for (size_t i = 0; i < stats.size(); i++) {
    double v = accessor(*stats[i]);
    double relative_value = (v - min) / (max - min);
    image.at<float>(0, i) = relative_value;
  }
}

/**
 * @brief
 * global variable to store stat information.
 */
std::vector<OneDataStat::Ptr> g_data;

/**
 * @brief
 * publisher for mean, variance and standard deviation images
 */
ros::Publisher pub_mean, pub_variance, pub_stddev;

/**
 * @brief
 * publisher for variance and standard deviation values for plotting
 */
ros::Publisher pub_variance_plot, pub_stddev_plot;

/**
 * @brief
 * publish a float image.
 */

void publishImage(const cv::Mat& image, ros::Publisher& pub)
{
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  pub.publish(cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::TYPE_32FC1, image).toImageMsg());
}

/**
 * @brief
 * callback function
 */
void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  if (g_data.size() == 0) {
    // first time, g_data is not initialized
    for (size_t i = 0; i < msg->ranges.size(); i++) {
      g_data.push_back(OneDataStat::Ptr(new OneDataStat()));
    }
  }
  if (g_data.size() != msg->ranges.size()) {
    ROS_ERROR("Size of g_data and ~input/ranges are not same");
    return;
  }

  for (size_t i = 0; i < msg->ranges.size(); i++) {
    g_data[i]->addData(msg->ranges[i]);
  }

  // Visualize current error and so on
  if (g_data[0]->count() > 2) {
    // compute only if it receives two more images.
    // because no stats can be computed with one message.
    cv::Mat mean_image(1, msg->ranges.size(), CV_32FC1);
    cv::Mat variance_image(1, msg->ranges.size(), CV_32FC1);
    cv::Mat stddev_image(1, msg->ranges.size(), CV_32FC1);
    generateImage(g_data, mean_image, &mean);
    generateImage(g_data, variance_image, &variance);
    generateImage(g_data, stddev_image, &stddev);
    publishImage(mean_image, pub_mean);
    publishImage(variance_image, pub_variance);
    publishImage(stddev_image, pub_stddev);

    jsk_recognition_msgs::PlotData plot_data;
    jsk_recognition_msgs::PlotData plot_stddev_data;
    for (size_t i = 0; i < g_data.size(); i++) {
      double x = g_data[i]->mean();
      double y = g_data[i]->variance();
      double stddev = g_data[i]->stddev();
      if (x > 1.0 && x < 40 && stddev < 0.100) {            // 10cm
        plot_data.xs.push_back(x);
        plot_data.ys.push_back(y);
        plot_stddev_data.xs.push_back(x);
        plot_stddev_data.ys.push_back(stddev);
      }
    }
    pub_variance_plot.publish(plot_data);
    pub_stddev_plot.publish(plot_stddev_data);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "range_sensor_error_visualization");
  ros::NodeHandle pnh("~");

  // Setup publishers before subscribe topic
  pub_mean = pnh.advertise<sensor_msgs::Image>("output/mean", 1);
  pub_variance = pnh.advertise<sensor_msgs::Image>("output/variance", 1);
  pub_stddev = pnh.advertise<sensor_msgs::Image>("output/stddev", 1);
  pub_variance_plot = pnh.advertise<jsk_recognition_msgs::PlotData>("output/variance_plot", 1);
  pub_stddev_plot = pnh.advertise<jsk_recognition_msgs::PlotData>("output/stddev_plot", 1);
  ros::Subscriber sub = pnh.subscribe("input", 1, &callback);
  ros::spin();

}

