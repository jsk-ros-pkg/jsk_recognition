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

#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <ctime>
#include <boost/random.hpp>
#include <string>
#include <typeinfo>
#include <sstream>
#include<stdio.h>
#include <sys/time.h>

void usage(char** argv)
{
  std::cerr << "Usage: " << argv[0] << " Iteration Number-of-points Max-iterations Gaussian-noise-variance Outlier-threshold" << std::endl;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr generatePoints(const int num_points, const double variance)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  boost::mt19937 gen = boost::mt19937(static_cast<unsigned long>(time(0)));
  boost::normal_distribution<> dst(0.0, sqrt(variance));
  boost::variate_generator<
    boost::mt19937&,
    boost::normal_distribution<> > rand(gen, dst);
   
  const double dx = 0.01;
  cloud->points.resize(num_points * num_points);
  for (size_t i = 0; i < num_points; i++) {
    for (size_t j = 0; j < num_points; j++) {
      pcl::PointXYZ p;
      p.x = i * dx;
      p.y = j * dx;
      p.z = rand();
      cloud->points[i * num_points + j] = p;
    }
  }
  return cloud;
}

int toInt(char* argv)
{
  std::stringstream ss(argv);
  int val = 0;
  ss >> val;
  return val;
}

double toDouble(char* argv)
{
  std::stringstream ss(argv);
  double val = 0;
  ss >> val;
  return val;
}

int main(int argc, char** argv)
{
  if (argc != 6) {
    usage(argv);
    return 1;
  }
  int iteration = toInt(argv[1]);
  int number_of_points = toInt(argv[2]);
  int max_iterations = toInt(argv[3]);
  double variance = toDouble(argv[4]);
  double outlier_threshold = toDouble(argv[5]);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = generatePoints(number_of_points, variance);
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients (true);
  seg.setRadiusLimits(0.0, std::numeric_limits<double>::max ()); // 0.3?
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (outlier_threshold);
  seg.setInputCloud(cloud);
  seg.setMaxIterations (max_iterations);
  seg.setModelType (pcl::SACMODEL_PLANE);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  struct timeval s, e;
  gettimeofday(&s, NULL);
  for (size_t i = 0; i < iteration; i++) {
    seg.segment (*inliers, *coefficients);
  }
  gettimeofday(&e, NULL);
  // Total, each, iteration, number_of_points, max_iterations, variance, outlier_threshold
  double time = (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6;
  printf("%lf,%lf,%d,%d,%d,%f,%f\n", time, time / iteration,
         iteration, number_of_points * number_of_points, max_iterations, variance, outlier_threshold);
  std::cerr << "Took " << time << " sec (" << time / iteration << " sec for each)" << std::endl;
  return 0;
}
