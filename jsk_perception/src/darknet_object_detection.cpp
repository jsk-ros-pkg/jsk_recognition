// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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
 *
 * Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>
 *
 *********************************************************************/

#include <jsk_perception/darknet_object_detection.h>

namespace enc = sensor_msgs::image_encodings;

namespace jsk_perception {

  DarknetObjectDetection::~DarknetObjectDetection()
  {
    free_network(network_);
    free(boxes_);
    free_ptrs((void**)probs_, max_box_num_);
  }

  void DarknetObjectDetection::onInit()
  {
    ConnectionBasedNodelet::onInit();

    pnh_.reset(new ros::NodeHandle(getMTPrivateNodeHandle()));

    std::string weight_path, config_path, label_path;
    pnh_->param<std::string>("network_weight_path", weight_path, "");
    pnh_->param<std::string>("network_config_path", config_path, "");
    pnh_->param<std::string>("label_path", label_path, "");

    pnh_->param<float>("detection_threshold", detection_threshold_, 0.2f);
    pnh_->param<float>("nms", nms_, 0.4f);
    pnh_->param<int>("mean", mean_, 3);

    // load labels if available
    if (boost::filesystem::exists(boost::filesystem::path(label_path)))
    {
      std::ifstream ifs(label_path.c_str());
      std::string buf;
      while(ifs && std::getline(ifs, buf))
        labels_.push_back(buf);
      ROS_INFO_STREAM(labels_.size() << " labels loaded.");
    }

    // initialize darknet
    if (!boost::filesystem::exists(boost::filesystem::path(weight_path)))
    {
      ROS_FATAL("No network weight file found at '%s'.", weight_path.c_str());
      return;
    }

    if (!boost::filesystem::exists(boost::filesystem::path(config_path)))
    {
      ROS_FATAL("No network config file found at '%s'.", config_path.c_str());
      return;
    }

    char* cpath = (char*)calloc(std::strlen(config_path.c_str())+1, sizeof(char));
    char* wpath = (char*)calloc(std::strlen(weight_path.c_str())+1, sizeof(char));
    std::strcpy(cpath, config_path.c_str());
    std::strcpy(wpath, weight_path.c_str());

    ROS_INFO_STREAM(">>> Network initializing...");

    network_ = parse_network_cfg(cpath);
    load_weights(&network_, wpath);
    free(cpath);
    free(wpath);
    layer layer = network_.layers[network_.n-1];
    set_batch_network(&network_, 1);
    srand(2222222);
    max_box_num_ = layer.w * layer.h * layer.n;

    boxes_ = (box*)calloc(max_box_num_, sizeof(box));
    probs_ = (float**)calloc(max_box_num_, sizeof(float*));
    for (int i = 0; i < max_box_num_; ++i) {
      probs_[i] = (float*)calloc(layer.classes, sizeof(float));
    }

    ROS_INFO_STREAM("<<< Network initialized");

    average_pred_.reset(new AverageArray(mean_, layer.classes));

    it_.reset(new image_transport::ImageTransport(*pnh_));

    pub_rects_ = advertise<jsk_recognition_msgs::LabeledRectArray>(*pnh_, "output", 1);

    onInitPostProcess();
  }

  void DarknetObjectDetection::subscribe()
  {
    sub_image_ = it_->subscribe("input", 1, &DarknetObjectDetection::imageCb, this);
  }

  void DarknetObjectDetection::unsubscribe()
  {
    sub_image_.shutdown();
  }

  void DarknetObjectDetection::imageCb(const sensor_msgs::Image::ConstPtr &msg)
  {
    layer layer = network_.layers[network_.n-1];

    IplImage cv_img;

    try {
      cv_img = cv_bridge::toCvShare(msg, enc::RGB8)->image;
    } catch (cv_bridge::Exception &e) {
      ROS_ERROR("Failed to convert image to IplImage");
      return;
    }
    image dn_img = ipl_to_image(&cv_img);
    image dn_resized = resize_image(dn_img, network_.w, network_.h);
    float *pred = network_predict(network_, (float*)dn_resized.data);

    average_pred_->add(pred);
    if (average_pred_->canAverage()) {
      average_pred_->averageArray(pred);
      layer.output = pred;
    }

    get_region_boxes(layer, 1, 1, detection_threshold_,
                     probs_, boxes_, 0, 0);

    if (nms_) {
      do_nms_sort(boxes_, probs_, max_box_num_, layer.classes, nms_);
    }

    jsk_recognition_msgs::LabeledRectArray pub_msg;
    pub_msg.header = msg->header;
    processResult(&dn_img, boxes_, probs_, pub_msg);
    pub_rects_.publish(pub_msg);
    free_image(dn_img);
    free_image(dn_resized);
  }

  void DarknetObjectDetection::processResult(image* dn_img, box* boxes, float** probs, jsk_recognition_msgs::LabeledRectArray &msg)
  {
    for (int i = 0; i < max_box_num_; ++i)
    {
      jsk_recognition_msgs::LabeledRect rect;
      int label = max_index(probs[i], (int)sizeof(probs[i]));
      float prob = probs[i][label];

      if (prob < detection_threshold_) continue;

      rect.label.index = (uint32_t)label;
      rect.label.likelihood = prob;
      if (label < labels_.size())
        rect.label.name = labels_[label];
      rect.rect.x = (boxes[i].x - boxes[i].w / 2) * dn_img->w;
      rect.rect.y = (boxes[i].y - boxes[i].h / 2) * dn_img->h;
      rect.rect.width = boxes[i].w * dn_img->w;
      rect.rect.height = boxes[i].h * dn_img->h;
      msg.rects.push_back(rect);
    }
  }
} // namespace jsk_perception

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::DarknetObjectDetection, nodelet::Nodelet)
