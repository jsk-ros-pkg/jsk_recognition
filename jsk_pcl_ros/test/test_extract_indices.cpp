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
 * Publish static pointcloud and indices.
 * They should be passed to jsk_pcl/ExtractIndices nodelet (running outside of this node)
 * and this node checks the result of jsk_pcl/ExtractIndices
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_msgs/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>

class ExtractIndicesTest: public testing::Test
{
protected:
  virtual void SetUp()
  {
    // generate a pointcloud
    generateTestData();
    // setup ROS
    ros::NodeHandle n("~");
    pub_cloud_ = n.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_even_indices_ = n.advertise<pcl_msgs::PointIndices>("output/even_indices", 1);
    pub_odd_indices_ = n.advertise<pcl_msgs::PointIndices>("output/odd_indices", 1);
    sub_even_result_ = n.subscribe("input/even_result", 1, &ExtractIndicesTest::evenCallback, this);
    sub_odd_result_ = n.subscribe("input/odd_result", 1, &ExtractIndicesTest::oddCallback, this);
    sub_even_organized_result_ = n.subscribe("input/even_organized_result", 1,
                                             &ExtractIndicesTest::evenOrganizedCallback, this);
    sub_odd_organized_result_ = n.subscribe("input/odd_organized_result",
                                            1, &ExtractIndicesTest::oddOrganizedCallback, this);

    // wait until
    ros::Time start_time = ros::Time::now();
    while (!isReady()) {
      publishData();
#if ROS_VERSION_MINIMUM(1,14,0) // melodic
      /*
        on melodic, it hugs after first test.
        not sure why and if we move SetUp code to class function, then it will raise
           terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
           what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
        error on exit.
        related ? -> https://github.com/ros/ros_comm/issues/838
       */
#else
      ros::spinOnce();
#endif
      ros::Duration(0.5).sleep();
    }
    ros::Time end_time = ros::Time::now();
    ROS_INFO("took %f sec to retrieve result", (end_time - start_time).toSec());
  }

  virtual void generateTestData()
  {
    // cloud
    original_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < 1000; i++) {
      pcl::PointXYZ p;
      p.x = i; p.y = 0; p.z = 0;
      original_cloud_->points.push_back(p);
      if (i % 2 == 0) {
        even_indices_msg_.indices.push_back(i);
      }
      else {
        odd_indices_msg_.indices.push_back(i);
      }
    }
    pcl::toROSMsg(*original_cloud_, original_cloud_msg_);
  }
  
  virtual bool isReady()
  {
    boost::mutex::scoped_lock lock(mutex_);
    ROS_INFO("even_cloud_ %d, odd_cloud_ %d, even_organized_cloud_ %d, odd_organized_cloud_ %d",
             !!even_cloud_, !!odd_cloud_, !!even_organized_cloud_, !!odd_organized_cloud_);
    return (even_cloud_ && odd_cloud_ && even_organized_cloud_ && odd_organized_cloud_);
  }

  virtual void publishData()
  {
    ros::Time now = ros::Time::now();
    original_cloud_msg_.header.stamp = now;
    even_indices_msg_.header.stamp = now;
    odd_indices_msg_.header.stamp = now;
    pub_cloud_.publish(original_cloud_msg_);
    pub_even_indices_.publish(even_indices_msg_);
    pub_odd_indices_.publish(odd_indices_msg_);
  }
  
  virtual void evenCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    even_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *even_cloud_);
  }
  
  virtual void oddCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    odd_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *odd_cloud_);
  }

  virtual void evenOrganizedCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    even_organized_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *even_organized_cloud_);
  }
  
  virtual void oddOrganizedCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
  {
    boost::mutex::scoped_lock lock(mutex_);
    odd_organized_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud_msg, *odd_organized_cloud_);
  }

  boost::mutex mutex_;
  ros::Subscriber sub_even_result_, sub_odd_result_;
  ros::Subscriber sub_even_organized_result_, sub_odd_organized_result_;
  ros::Publisher pub_cloud_, pub_even_indices_, pub_odd_indices_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr original_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr even_cloud_, odd_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr even_organized_cloud_, odd_organized_cloud_;
  sensor_msgs::PointCloud2 original_cloud_msg_;
  pcl_msgs::PointIndices even_indices_msg_, odd_indices_msg_;
};

TEST_F(ExtractIndicesTest, testEvenIndices)
{
  EXPECT_EQ(even_cloud_->points.size(), even_indices_msg_.indices.size());
  for (size_t i = 0; i < even_cloud_->points.size(); i++) {
    EXPECT_FLOAT_EQ(even_cloud_->points[i].x, i * 2.0);
  }
}

TEST_F(ExtractIndicesTest, testOddIndices)
{
  EXPECT_EQ(odd_cloud_->points.size(), odd_indices_msg_.indices.size());
  for (size_t i = 0; i < odd_cloud_->points.size(); i++) {
    EXPECT_FLOAT_EQ(odd_cloud_->points[i].x, i * 2.0 + 1.0);
  }
}

TEST_F(ExtractIndicesTest, testEvenOrganizedIndices)
{
  EXPECT_EQ(even_organized_cloud_->points.size(), original_cloud_->points.size());
  size_t non_nan_count = 0;
  for (size_t i = 0; i < even_organized_cloud_->points.size(); i++) {
    if (!std::isnan(even_organized_cloud_->points[i].x)) {
      EXPECT_FLOAT_EQ(even_organized_cloud_->points[i].x, i);
      non_nan_count++;
    }
  }
  EXPECT_EQ(non_nan_count, even_indices_msg_.indices.size());
}

TEST_F(ExtractIndicesTest, testOddOrganizedIndices)
{
  EXPECT_EQ(odd_organized_cloud_->points.size(), original_cloud_->points.size());
  size_t non_nan_count = 0;
  for (size_t i = 0; i < odd_organized_cloud_->points.size(); i++) {
    if (!std::isnan(odd_organized_cloud_->points[i].x)) {
      EXPECT_FLOAT_EQ(odd_organized_cloud_->points[i].x, i);
      non_nan_count++;
    }
  }
  EXPECT_EQ(non_nan_count, odd_indices_msg_.indices.size());
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_extract_indices");
#if ROS_VERSION_MINIMUM(1,14,0) // melodic
  ros::AsyncSpinner spinner(1);
  spinner.start();
#endif
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
