/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ryohei Ueda and JSK Lab
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
 *   * Neither the name of the Willow Garage nor the names of its
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

#include <pluginlib/class_list_macros.h>
#include "jsk_pcl_ros/color_converter_nodelet.h"

namespace pcl_ros
{
    void
    RGB2HSVColorConverter::computePublish
    (const PointCloudInConstPtr &cloud,
     const PointCloudInConstPtr &surface,
     const IndicesPtr &indices)
    {
        impl_.setInputCloud (cloud);
        impl_.setIndices (indices);
        PointCloudOut output;
        impl_.compute (output);
        output.header = cloud->header;
        pub_output_.publish (output.makeShared ());
    }
    
    void HSV2RGBColorConverter::computePublish
    (const PointCloudInConstPtr &cloud,
     const PointCloudInConstPtr &surface,
     const IndicesPtr &indices)
    {
        impl_.setInputCloud (cloud);
        impl_.setIndices (indices);
        PointCloudOut output;
        impl_.compute (output);
        output.header = cloud->header;
        pub_output_.publish (output.makeShared ());
    }
    
}

typedef pcl_ros::RGB2HSVColorConverter RGB2HSVColorConverter;
typedef pcl_ros::HSV2RGBColorConverter HSV2RGBColorConverter;

PLUGINLIB_DECLARE_CLASS (jsk_pcl, RGB2HSVColorConverter,
                         RGB2HSVColorConverter, nodelet::Nodelet);
PLUGINLIB_DECLARE_CLASS (jsk_pcl, HSV2RGBColorConverter,
                         HSV2RGBColorConverter, nodelet::Nodelet);
