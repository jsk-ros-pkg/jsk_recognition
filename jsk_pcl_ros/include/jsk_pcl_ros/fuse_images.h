/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Kentaro Wada and JSK Lab
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
 *   * Neither the name of Kentaro Wada and JSK Lab nor the names of its
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

#ifndef JSK_PCL_ROS_FUSE_DEPTH_IMAGES_H_
#define JSK_PCL_ROS_FUSE_DEPTH_IMAGES_H_

#include <vector>

#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/pass_through.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

namespace jsk_pcl_ros
{
  class FuseImages: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    FuseImages(const std::string& name, const std::string& encoding):
      DiagnosticNodelet(name), encoding_(encoding) {}
  protected:
    virtual void onInit();
    virtual void subscribe();
    virtual void unsubscribe();
    virtual bool validateInput(const sensor_msgs::Image::ConstPtr& in,
                               const int height_expected, const int width_expected, std::vector<cv::Mat>& inputs);

    ros::Publisher pub_out_;

    bool approximate_sync_;
    int queue_size_;
    bool averaging_;
    std::string encoding_;

    /** \brief Null passthrough filter, used for pushing empty elements in the
      * synchronizer */
    message_filters::PassThrough<sensor_msgs::Image> nf_;

    /** \brief A vector of message filters. */
    std::vector<boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > > filters_;

    /** \brief Synchronizer.
      * \note This will most likely be rewritten soon using the DynamicTimeSynchronizer.
      */
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> > > async_;
    boost::shared_ptr<message_filters::Synchronizer<message_filters::sync_policies::ExactTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image,
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> > > sync_;

    /** \brief Input point cloud callback.
      * Because we want to use the same synchronizer object, we push back
      * empty elements with the same timestamp.
      */
    inline void
    input_callback (const sensor_msgs::Image::ConstPtr &input)
    {
      sensor_msgs::Image imgmsg;
      imgmsg.header.stamp = input->header.stamp;
      nf_.add(boost::make_shared<sensor_msgs::Image> (imgmsg));
    }

    virtual void inputCb(const sensor_msgs::Image::ConstPtr &in1, const sensor_msgs::Image::ConstPtr &in2,
                         const sensor_msgs::Image::ConstPtr &in3, const sensor_msgs::Image::ConstPtr &in4,
                         const sensor_msgs::Image::ConstPtr &in5, const sensor_msgs::Image::ConstPtr &in6,
                         const sensor_msgs::Image::ConstPtr &in7, const sensor_msgs::Image::ConstPtr &in8);

    virtual cv::Mat fuseInputs(std::vector<cv::Mat> inputs) {};
  private:
  };

  class FuseDepthImages: public FuseImages
  {
  public:
    FuseDepthImages(): FuseImages("FuseDepthImages", sensor_msgs::image_encodings::TYPE_32FC1) {};
  protected:
    virtual cv::Mat fuseInputs(std::vector<cv::Mat> inputs);
  };

  class FuseRGBImages: public FuseImages
  {
  public:
    FuseRGBImages(): FuseImages("FuseRGBImages", sensor_msgs::image_encodings::RGB8) {};
  protected:
    virtual cv::Mat fuseInputs(std::vector<cv::Mat> inputs);
  };

} // namespace jsk_pcl_ros

#endif // JSK_PCL_ROS_FUSE_DEPTH_IMAGES_H_
