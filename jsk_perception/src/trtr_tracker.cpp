// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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

#include <jsk_perception/trtr_tracker.h>

using namespace trtr;
using namespace jsk_perception;

void TrTrTrackerRos::onInit()
{
  DiagnosticNodelet::onInit();

  // ros pub-sub
  pnh_->param("approximate_sync", approximate_sync_, false);
  pnh_->param("queue_size", queue_size_, 100);

  // inferene model files
  std::string encoder_model_file;
  pnh_->param ("encoder_model_file", encoder_model_file, std::string("model.trt"));

  std::string decoder_model_file;
  pnh_->param ("decoder_model_file", decoder_model_file, std::string("model.trt"));

  // hyper-parameter for postprocess
  // following default values are fine-tuned with various tracker benchmark.
  double cosine_window_factor, size_lpf_factor, score_threshold;
  int cosine_window_step;
  pnh_->param ("cosine_window_factor", cosine_window_factor, 0.4);
  pnh_->param ("cosine_window_step", cosine_window_step, 3);
  pnh_->param ("size_lpf_factor", size_lpf_factor, 0.8);
  pnh_->param ("score_threshold", score_threshold, 0.05);

  int device; // 0: cpu, 1: gpu
  pnh_->param ("device", device, 1);


#if !defined(TENSORRT) && !defined(ONNXRT)
  ROS_FATAL("do not have tensorRT nor onnx, can not do inference");
  return;
  #endif

#if !defined(TENSORRT)
  if(device == 1)
    {
      ROS_FATAL("can only support CPU model");
      return;
    }
  #endif

#if !defined(ONNXRT)
  if(device == 0)
    {
      ROS_FATAL("can only support GPU model");
      return;
    }
  #endif

  // initialize tracker
#ifdef ONNXRT
  if(device == 0)  tracker_ = std::make_shared<ONNX>();
#endif

#ifdef TENSORRT
  if(device == 1)  tracker_ = std::make_shared<TensorRT>();
#endif

  if(!tracker_->init(encoder_model_file, decoder_model_file, score_threshold, cosine_window_factor, cosine_window_step, size_lpf_factor)) return;


  // subscribers to set initial tracking window.
  image_to_init_sub_.subscribe(*pnh_, "input", 1);
  rect_to_init_sub_.subscribe(*pnh_, "input/rect", 1);

  if (approximate_sync_)
    {
      async_ = boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(queue_size_);
      async_->connectInput(image_to_init_sub_, rect_to_init_sub_);
      async_->registerCallback(boost::bind(&TrTrTrackerRos::setInitialRect, this, _1, _2));
    }
  else
    {
      sync_ = boost::make_shared<message_filters::Synchronizer<ExactSyncPolicy> >(queue_size_);
      sync_->connectInput(image_to_init_sub_, rect_to_init_sub_);
      sync_->registerCallback(boost::bind(&TrTrTrackerRos::setInitialRect, this, _1, _2));
    }

  image_pub_ = advertiseImage(*pnh_, "output/vis", 1);
  rect_pub_ = advertise<jsk_recognition_msgs::RectArray>(*pnh_, "output/rect", 1);
  it_ = boost::make_shared<image_transport::ImageTransport>(*pnh_);

  onInitPostProcess();
}

void TrTrTrackerRos::subscribe()
{
  // subscribers to process the tracking
  image_sub_ = it_->subscribe("input", 1, &TrTrTrackerRos::getTrackingResult, this);
}

void TrTrTrackerRos::unsubscribe()
{
  image_sub_.shutdown();
}

void TrTrTrackerRos::setInitialRect(const sensor_msgs::Image::ConstPtr& image_msg,
                                    const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  cv::Mat image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;

  // TODO: currently only support one target
  std::vector<float> bbox;
  bbox.push_back(rect_msg->rects.at(0).x);
  bbox.push_back(rect_msg->rects.at(0).y);
  bbox.push_back(rect_msg->rects.at(0).width);
  bbox.push_back(rect_msg->rects.at(0).height);

  tracker_->reset(bbox, image);
  rect_initialized_ = true;

  ROS_INFO("set the initialze rect to update trtr encoder part");
}

void TrTrTrackerRos::getTrackingResult(const sensor_msgs::ImageConstPtr& msg)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (!rect_initialized_)
    return;

  cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  tracker_->track(image);

  cv::rectangle(image,
                tracker_->getBboxLT(), tracker_->getBboxRB(),
                cv::Scalar(0,255,255),3,4);


  // Publish all.
  image_pub_.publish(cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8, image).toImageMsg());

  jsk_recognition_msgs::RectArray rect_msg;
  rect_msg.header = msg->header;
  jsk_recognition_msgs::Rect rect;
  rect.x = tracker_->getBboxLT().x;
  rect.y = tracker_->getBboxLT().y;
  rect.width = tracker_->getBboxRB().x - tracker_->getBboxLT().x;
  rect.height = tracker_->getBboxRB().y -tracker_->getBboxLT().y;
  rect_msg.rects.push_back(rect);
  rect_pub_.publish(rect_msg);
}

bool Base::init(std::string encoder_model_file, std::string decoder_model_file, float score_threshold, float cosine_window_factor, int cosine_window_step, float size_lpf_factor)
{
  score_threshold_ = score_threshold;
  cosine_window_factor_ = cosine_window_factor;
  cosine_window_step_ = cosine_window_step;
  size_lpf_factor_ = size_lpf_factor;

  // image preprocess normalize
  img_mean_ = cv::Vec3f(0.485, 0.456, 0.406);
  img_std_ = cv::Vec3f(0.229, 0.224, 0.225);

  if(!loadModels(encoder_model_file, decoder_model_file)) return false;

  // craete default position embedding
  for(int i = 0; i < transformer_dim_ / 2; i++)
    pos_emded_dim_t_.push_back(std::pow((float)10000, 2 * (i / 2) / (float)(transformer_dim_ / 2)));

  // template position embedding
  std::vector<int> template_pos_embed_sizes;
  template_pos_embed_sizes.push_back(template_feat_size_);
  template_pos_embed_sizes.push_back(template_feat_size_);
  template_pos_embed_sizes.push_back(transformer_dim_);
  default_template_pos_embed_ = cv::Mat(template_pos_embed_sizes, CV_32F);
  makePositionEmbedding((float*)default_template_pos_embed_.data, template_feat_size_);

  // search position embedding
  std::vector<int> search_pos_embed_sizes;
  search_pos_embed_sizes.push_back(search_feat_size_);
  search_pos_embed_sizes.push_back(search_feat_size_);
  search_pos_embed_sizes.push_back(transformer_dim_);
  default_search_pos_embed_ = cv::Mat(search_pos_embed_sizes, CV_32F);
  makePositionEmbedding((float*)default_search_pos_embed_.data, search_feat_size_);

  // create cosine window for distance penalty
  createCosineWindow();

  return true;
}

void Base::reset(const std::vector<float>& bbox, const cv::Mat& img)
{
  cx_ = bbox.at(0) + bbox.at(2) / 2;
  cy_ = bbox.at(1) + bbox.at(3) / 2;
  w_ = bbox.at(2);
  h_ = bbox.at(3);

  auto chan_avg = cv::mean(img);
  float s_z;
  float scale_z;
  getSiamFCLikeScale(template_img_size_, w_, h_, s_z, scale_z);

  cv::Mat template_img = cv::Mat::zeros(cv::Size(template_img_size_, template_img_size_), img.type());

  std::vector<float> crop_bbox(0);
  crop_bbox.push_back(cx_ - s_z/2);
  crop_bbox.push_back(cy_ - s_z/2);
  crop_bbox.push_back(s_z);
  crop_bbox.push_back(s_z);
  centerCropping(img, crop_bbox, template_img_size_, chan_avg, template_img);

  // get mask
  std::vector<float> template_mask{0.0, 0.0, (float)template_img_size_, (float)template_img_size_};
  if(cx_ < s_z/2) template_mask.at(0) = (s_z/2 - cx_) * scale_z;
  if(cy_ < s_z/2) template_mask.at(1) = (s_z/2 - cy_) * scale_z;
  if(cx_ + s_z/2 > img.cols) template_mask.at(2) -= (cx_ + s_z/2 - img.cols) * scale_z;
  if(cy_ + s_z/2 > img.rows) template_mask.at(3) -= (cy_ + s_z/2 - img.rows) * scale_z;
  for (auto& v: template_mask) v *= (float)template_feat_size_ / template_img_size_;

  encoderInference(template_img, template_mask);
}

void Base::track(const cv::Mat& img)
{
  float s_z;
  float scale_z;
  getSiamFCLikeScale(template_img_size_, w_, h_, s_z, scale_z);
  cv::Mat search_img = cv::Mat::zeros(cv::Size(search_img_size_, search_img_size_), img.type());
  float s_x = search_img_size_ / scale_z;

  // get mask
  std::vector<float> search_mask{0.0, 0.0, (float)search_img_size_, (float)search_img_size_};
  if(cx_ < s_x/2) search_mask.at(0) = (s_x/2 - cx_) * scale_z;
  if(cy_ < s_x/2) search_mask.at(1) = (s_x/2 - cy_) * scale_z;
  if(cx_ + s_x/2 > img.cols) search_mask.at(2) -= (cx_ + s_x/2 - img.cols) * scale_z;
  if(cy_ + s_x/2 > img.rows) search_mask.at(3) -= (cy_ + s_x/2 - img.rows) * scale_z;
  for (auto& v: search_mask) v *= (float)search_feat_size_ / search_img_size_;

  cv::Scalar chan_avg(0,0,0);
  if(cx_ - s_x/2 < 1 || cy_ - s_x/2 < 1 || cx_ + s_x/2 > img.cols - 1 || cy_ + s_x/2 > img.rows - 1)
    chan_avg = cv::mean(img);

  std::vector<float> crop_bbox(0);
  crop_bbox.push_back(cx_ - s_x/2);
  crop_bbox.push_back(cy_ - s_x/2);
  crop_bbox.push_back(s_x);
  crop_bbox.push_back(s_x);

  centerCropping(img, crop_bbox, search_img_size_, chan_avg, search_img);

  decoderInference(search_img, search_mask);

  // do not update tracker result if the inference score is too low
  double max;
  cv::minMaxLoc(heatmap_, nullptr, &max);
  if(max < score_threshold_) return;

  // postprocess
  // Note: we remove size change penalty whcih is introduced in the original implementation

  // 1. add cosine window (distance penalty)
  int best_window_step = 0;
  float window_factor = cosine_window_factor_;
  cv::Mat post_heatmap;
  float best_score = 0;
  cv::Point max_loc; // store in (x: col, y: row)
  for(int i = 0; i < cosine_window_step_; i++)
    {
      post_heatmap = heatmap_ * (1 -  window_factor) + cosine_window_ * window_factor;
      cv::minMaxLoc(post_heatmap, nullptr, nullptr, nullptr, &max_loc);

      best_score = heatmap_.at<float>(max_loc.y, max_loc.x);
      if(best_score > score_threshold_)
        break;
      else
        window_factor = std::max(window_factor - cosine_window_factor_ / cosine_window_step_, (float)0);
    }
  //std::cout << best_score << ", "<< max_loc << std::endl;

  cv::Vec2f target_wh = cv::Mat2f(bbox_wh_)(max_loc) * search_img_size_ / scale_z;
  cv::Vec2f bbox_ct = (cv::Vec2f(max_loc.x, max_loc.y) + cv::Mat2f(bbox_reg_)(max_loc)) * search_img_size_ / float(search_feat_size_);

  //std::cout << bbox_ct << std::endl;
  cv::Vec2f ct_delta = (bbox_ct - cv::Vec2f(search_img_size_, search_img_size_) / 2) / scale_z;
  float cx = cx_ + ct_delta[0];
  float cy = cy_ + ct_delta[1];

  // 2. lpf for size
  float lpf = best_score * size_lpf_factor_;
  if (size_lpf_factor_ == 0) lpf = 1.0;
  float w = w_ * (1 - lpf) + target_wh[0] * lpf;
  float h = h_ * (1 - lpf) + target_wh[1] * lpf;

  // 3. handle boundary
  float x1 = std::max((float)0, cx - w / 2);
  float y1 = std::max((float)0, cy - h / 2);
  float x2 = std::min((float)img.cols, cx + w / 2);
  float y2 = std::min((float)img.rows, cy + h / 2);
  cx_ = (x1 + x2) / 2;
  cy_ = (y1 + y2) / 2;
  w_ = (x2 - x1);
  h_ = (y2 - y1);
}

void Base::createCosineWindow()
{
  // https://numpy.org/doc/stable/reference/generated/numpy.hanning.html
  cosine_window_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32F);
  for(int i = 0; i < search_feat_size_; i++)
    {
      for(int j = 0; j < search_feat_size_; j++)
        {
          float v_i = 0.5 - 0.5 * cosf(2 * M_PI * i / (search_feat_size_ - 1));
          float v_j = 0.5 - 0.5 * cosf(2 * M_PI * j / (search_feat_size_ - 1));

          cosine_window_.at<float>(i,j) = v_i * v_j;
        }
    }
}

void Base::getSiamFCLikeScale(const int& template_img_size,  const float& w, const float& h, float& s_z, float& scale_z)
{
  const float context_amount = 0.5;
  float wc_z = w + context_amount * (w + h);
  float hc_z = h + context_amount * (w + h);
  s_z = sqrt(wc_z * hc_z);
  scale_z = template_img_size / s_z;
}

void Base::centerCropping(const cv::Mat& img, const std::vector<float>& bbox, const int& out_sz, const cv::Scalar& padding, cv::Mat& cropped_img)
{
  float a = (out_sz-1) / bbox[2]; // TODO: check the one pixel operation i.e., -1
  float b = (out_sz-1) / bbox[3];
  float c = -a * bbox[0];
  float d = -b * bbox[1];
  cv::Mat mapping = (cv::Mat_<float>(2,3) << a, 0, c, 0, b, d);
  cv::warpAffine(img, cropped_img, mapping, cropped_img.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT, padding);
}

void Base::makePositionEmbedding(float* p, int feat_size, std::vector<int> mask_bounds)
{
  if(mask_bounds.empty()) mask_bounds = std::vector<int>{0, 0, feat_size, feat_size};

  float factor_x =  2 * (float)M_PI / ((mask_bounds.at(2) - mask_bounds.at(0)) + 1e-6);
  float factor_y =  2 * (float)M_PI / ((mask_bounds.at(3) - mask_bounds.at(1)) + 1e-6);
#pragma omp parallel for
  for (int i = 0; i < feat_size; i++)
    {
      for (int j = 0; j < feat_size; j++)
        {
          for (int k = 0; k < 4; k++)
            {
              for(int l = 0; l < transformer_dim_ / 4; l++)
                {
                  int offset = i * feat_size * transformer_dim_ + j * transformer_dim_;
                  switch(k)
                    {
                    case 0:
                      {
                        int index = std::min(std::max(0, i + 1 - mask_bounds.at(1)), mask_bounds.at(3) - mask_bounds.at(1)); // TODO: should be 0 if i > mask_bounds.at(3)
                        if(j < mask_bounds.at(0) || j > mask_bounds.at(2)-1) index = 0;
                        offset += l * 2;
                        p[offset] = sinf(index * factor_y / pos_emded_dim_t_.at(l*2));
                        break;
                      }
                    case 1:
                      {
                        int index = std::min(std::max(0, i + 1 - mask_bounds.at(1)), mask_bounds.at(3) - mask_bounds.at(1)); // TODO: should be 0 if i > mask_bounds.at(3)
                        if(j < mask_bounds.at(0) || j > mask_bounds.at(2)-1) index = 0;
                        offset +=  l * 2 + 1;
                        p[offset] = cosf(index * factor_y / pos_emded_dim_t_.at(l*2 + 1));
                        break;
                      }
                    case 2:
                      {
                        int index = std::min(std::max(0, j + 1 - mask_bounds.at(0)), mask_bounds.at(2) - mask_bounds.at(0)); // TODO: should be 0 if i > mask_bounds.at(2)
                        if(i < mask_bounds.at(1) || i > mask_bounds.at(3)-1) index = 0;
                        offset += l * 2 + transformer_dim_ / 2;
                        p[offset] = sinf(index * factor_x / pos_emded_dim_t_.at(l*2));
                        break;
                      }
                    case 3:
                      {
                        int index = std::min(std::max(0, j + 1 - mask_bounds.at(0)), mask_bounds.at(2) - mask_bounds.at(0)); // TODO: should be 0 if i > mask_bounds.at(2)
                        if(i < mask_bounds.at(1) || i > mask_bounds.at(3)-1) index = 0;
                        offset += l * 2 + transformer_dim_ / 2 + 1;
                        p[offset] = cosf(index * factor_x / pos_emded_dim_t_.at(l*2 + 1));
                        break;
                      }
                    }
                }
            }
        }
    }
}

#ifdef TENSORRT
bool TensorRT::loadModels(std::string encoder_model_file, std::string decoder_model_file)
{
  encoder_engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(loadEngine(encoder_model_file), InferDeleter());
  if(!encoder_engine_) return false;
  encoder_context_ = std::shared_ptr<nvinfer1::IExecutionContext>(encoder_engine_->createExecutionContext(), InferDeleter());
  encoder_buffers_ = std::make_shared<samplesCommon::BufferManager>(encoder_engine_);

  decoder_engine_ = std::shared_ptr<nvinfer1::ICudaEngine>(loadEngine(decoder_model_file), InferDeleter());
  if(!decoder_engine_) return false;
  decoder_context_ = std::shared_ptr<nvinfer1::IExecutionContext>(decoder_engine_->createExecutionContext(), InferDeleter());
  decoder_buffers_ = std::make_shared<samplesCommon::BufferManager>(decoder_engine_);

  for(int i = 0; i < encoder_engine_->getNbBindings(); i++)
    {
      if(encoder_engine_->getBindingName(i) == std::string("template_image"))
        {
          template_img_size_ = encoder_engine_->getBindingDimensions(i).d[1];
        }
      if(encoder_engine_->getBindingName(i) == std::string("template_pos_embed"))
        {
          template_feat_size_ = encoder_engine_->getBindingDimensions(i).d[1];
          transformer_dim_ = encoder_engine_->getBindingDimensions(i).d[3];
        }
    }

  for(int i = 0; i < decoder_engine_->getNbBindings(); i++)
    {
      if(decoder_engine_->getBindingName(i) == std::string("search_image"))
        {
          search_img_size_ = decoder_engine_->getBindingDimensions(i).d[1];
        }
      if(decoder_engine_->getBindingName(i) == std::string("search_pos_embed"))
        {
          search_feat_size_ = decoder_engine_->getBindingDimensions(i).d[1];
        }
    }

  return true;
}

nvinfer1::ICudaEngine* TensorRT::loadEngine(const std::string& model_file)
{
  std::ifstream engineFile(model_file, std::ios::binary);
  if (!engineFile)
    {
      ROS_ERROR_STREAM("Error opening model file: " << model_file);
      return nullptr;
    }

  engineFile.seekg(0, engineFile.end);
  long int fsize = engineFile.tellg();
  engineFile.seekg(0, engineFile.beg);

  std::vector<char> engineData(fsize);
  engineFile.read(engineData.data(), fsize);
  if (!engineFile)
    {
      ROS_ERROR_STREAM("Error loading engine from model file: " << model_file);
      return nullptr;
    }

  TrtUniquePtr<nvinfer1::IRuntime> runtime{nvinfer1::createInferRuntime(logger_)};
  return runtime->deserializeCudaEngine(engineData.data(), fsize, nullptr);
}

void TensorRT::encoderInference(const cv::Mat& img, const std::vector<float>& bounds)
{
  processInput(img, bounds, std::string("template"));

  encoder_buffers_->copyInputToDevice();
  encoder_context_->executeV2(encoder_buffers_->getDeviceBindings().data());
  encoder_buffers_->copyOutputToHost();

  std::string binding_name("encoder_memory");
  std::memcpy(decoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->size(binding_name));

  binding_name = std::string("template_pos_embed");
  std::memcpy(decoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->getHostBuffer(binding_name), encoder_buffers_->size(binding_name));
}

void TensorRT::decoderInference(const cv::Mat& img, const std::vector<float>& bounds)
{
  processInput(img, bounds, std::string("search"));

  decoder_buffers_->copyInputToDevice();
  decoder_context_->executeV2(decoder_buffers_->getDeviceBindings().data());
  decoder_buffers_->copyOutputToHost();

  heatmap_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC1, decoder_buffers_->getHostBuffer("pred_heatmap"));
  bbox_reg_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC2, decoder_buffers_->getHostBuffer("pred_bbox_reg"));
  bbox_wh_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC2, decoder_buffers_->getHostBuffer("pred_bbox_wh"));
}

void TensorRT::processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type)
{
  std::shared_ptr<samplesCommon::BufferManager>  buffers;
  if(image_type == std::string("template"))
    buffers = encoder_buffers_;
  if(image_type == std::string("search"))
    buffers = decoder_buffers_;

  float* hostDataBuffer = static_cast<float*>(buffers->getHostBuffer(image_type + std::string("_image")));
  assert(hostDataBuffer != nullptr);

  int cols = img.cols;
  int rows = img.rows;
  int channels = img.channels();

  img.forEach<cv::Vec3b>([&](const cv::Vec3b &p, const int position[]) {
      int offset =  (position[0] * rows + position[1]) * channels;
      hostDataBuffer[offset] = (p[0] / 255.0 - img_mean_[0]) / img_std_[0];
      hostDataBuffer[offset + 1] = (p[1] / 255.0 - img_mean_[1]) / img_std_[1];
      hostDataBuffer[offset + 2] = (p[2] / 255.0 - img_mean_[2]) / img_std_[2];
    });

  /* position embedding */
  int feat_size = 0;
  if(image_type == std::string("template")) feat_size = template_feat_size_;
  if(image_type == std::string("search")) feat_size = search_feat_size_;
  std::string binding_name = image_type + std::string("_pos_embed");

  if(bounds[0] >= 1 || bounds[1] >= 1 || bounds[2] < feat_size - 1 || bounds[3] < feat_size - 1)
    {
      std::vector<int> discretized_mask{(int)std::ceil(bounds[0]), (int)std::ceil(bounds[1]), (int)std::floor(bounds[2]), (int)std::floor(bounds[3])};
      makePositionEmbedding(static_cast<float*>(buffers->getHostBuffer(binding_name)), feat_size, discretized_mask);
    }
  else
    {
      if(image_type == std::string("template"))
        std::memcpy(buffers->getHostBuffer(binding_name), (void*)default_template_pos_embed_.data, buffers->size(binding_name)); // TODO: directly use buffer pointer

      if(image_type == std::string("search"))
        std::memcpy(buffers->getHostBuffer(binding_name), (void*)default_search_pos_embed_.data, buffers->size(binding_name)); // TODO: directly use buffer pointer
    }
}
#endif

#ifdef ONNXRT
bool ONNX::loadModels(std::string encoder_model_file, std::string decoder_model_file)
{
  // https://github.com/microsoft/onnxruntime/blob/master/csharp/test/Microsoft.ML.OnnxRuntime.EndToEndTests.Capi/CXX_Api_Sample.cpp
  env_ = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "TrTr Tracker");
  Ort::SessionOptions options;

  encoder_session_ = std::make_shared<Ort::Session>(env_, encoder_model_file.c_str(), options);
  decoder_session_ = std::make_shared<Ort::Session>(env_, decoder_model_file.c_str(), options);

  size_t num_input_nodes = encoder_session_->GetInputCount();
  for (int i = 0; i < num_input_nodes; i++)
    {
      std::string input_name(encoder_session_->GetInputName(i, allocator_));

      Ort::TypeInfo type_info = encoder_session_->GetInputTypeInfo(i);
      auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

      std::vector<int64_t> input_node_dims;
      input_node_dims = tensor_info.GetShape();

      if(input_name == std::string("template_image"))
        {
          template_img_size_ = (int)input_node_dims.at(1);
        }
      if(input_name == std::string("template_pos_embed"))
        {
          template_feat_size_ = (int)input_node_dims.at(1);
          transformer_dim_ = (int)input_node_dims.at(3);
        }

      encoder_input_names_.push_back(encoder_session_->GetInputName(i, allocator_));
      encoder_input_index_map_.insert(std::make_pair(input_name, i));
      encoder_input_tensors_.push_back(Ort::Value::CreateTensor<float>(allocator_, input_node_dims.data(), input_node_dims.size()));

    }

  size_t num_output_nodes = encoder_session_->GetOutputCount();
  for (int i = 0; i < num_output_nodes; i++)
    {
      std::string output_name(encoder_session_->GetOutputName(i, allocator_));

      Ort::TypeInfo type_info = encoder_session_->GetOutputTypeInfo(i);
      auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

      std::vector<int64_t> output_node_dims;
      output_node_dims = tensor_info.GetShape();

      encoder_output_names_.push_back(encoder_session_->GetOutputName(i, allocator_));
      encoder_output_index_map_.insert(std::make_pair(output_name, i));
    }

  num_input_nodes = decoder_session_->GetInputCount();
  for (int i = 0; i < num_input_nodes; i++)
    {
      std::string input_name(decoder_session_->GetInputName(i, allocator_));

      Ort::TypeInfo type_info = decoder_session_->GetInputTypeInfo(i);
      auto tensor_info = type_info.GetTensorTypeAndShapeInfo();

      std::vector<int64_t> input_node_dims;
      input_node_dims = tensor_info.GetShape();


      if(input_name == std::string("search_image"))
        search_img_size_ = (int)input_node_dims.at(1);
      if(input_name == std::string("search_pos_embed"))
        search_feat_size_ = (int)input_node_dims.at(1);

      decoder_input_names_.push_back(decoder_session_->GetInputName(i, allocator_));
      decoder_input_index_map_.insert(std::make_pair(input_name, i));
      decoder_input_tensors_.push_back(Ort::Value::CreateTensor<float>(allocator_, input_node_dims.data(), input_node_dims.size()));
    }

  num_output_nodes = decoder_session_->GetOutputCount();
  for (int i = 0; i < num_output_nodes; i++)
    {
      std::string output_name(decoder_session_->GetOutputName(i, allocator_));

      Ort::TypeInfo type_info = decoder_session_->GetOutputTypeInfo(i);
      auto tensor_info = type_info.GetTensorTypeAndShapeInfo();
      std::vector<int64_t> output_node_dims;
      output_node_dims = tensor_info.GetShape();

      decoder_output_names_.push_back(decoder_session_->GetOutputName(i, allocator_));
      decoder_output_index_map_.insert(std::make_pair(output_name, i));
    }

  return true;
}

void ONNX::encoderInference(const cv::Mat& img, const std::vector<float>& bounds)
{
  processInput(img, bounds, std::string("template"));

  auto output_tensors = encoder_session_->Run(Ort::RunOptions{nullptr},
                                              encoder_input_names_.data(),
                                              encoder_input_tensors_.data(),
                                              encoder_input_names_.size(),
                                              encoder_output_names_.data(),
                                              encoder_output_names_.size());


  int index = decoder_input_index_map_.at(std::string("encoder_memory"));
  int size = transformer_dim_ * template_feat_size_ * template_feat_size_ * sizeof(float);
  std::memcpy(decoder_input_tensors_.at(index).GetTensorMutableData<void>(),
              output_tensors.front().GetTensorMutableData<void>(),
              size);

  int encoder_index = encoder_input_index_map_.at(std::string("template_pos_embed"));
  int decoder_index = decoder_input_index_map_.at(std::string("template_pos_embed"));
  size = transformer_dim_ * template_feat_size_ * template_feat_size_ * sizeof(float);
  std::memcpy(decoder_input_tensors_.at(decoder_index).GetTensorMutableData<void>(),
              encoder_input_tensors_.at(encoder_index).GetTensorMutableData<void>(),
              size);
}

void ONNX::decoderInference(const cv::Mat& img, const std::vector<float>& bounds)
{
  processInput(img, bounds, std::string("search"));

  auto output_tensors = decoder_session_->Run(Ort::RunOptions{nullptr},
                                              decoder_input_names_.data(),
                                              decoder_input_tensors_.data(),
                                              decoder_input_names_.size(),
                                              decoder_output_names_.data(),
                                              decoder_output_names_.size());

  heatmap_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC1,
                     output_tensors.at(decoder_output_index_map_.at(std::string("pred_heatmap"))).GetTensorMutableData<float>());

  bbox_reg_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC2,
                      output_tensors.at(decoder_output_index_map_.at(std::string("pred_bbox_reg"))).GetTensorMutableData<float>());
  bbox_wh_ = cv::Mat(cv::Size(search_feat_size_, search_feat_size_), CV_32FC2,
                     output_tensors.at(decoder_output_index_map_.at(std::string("pred_bbox_wh"))).GetTensorMutableData<float>());
}

void ONNX::processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type)
{

  float* p;
  if(image_type == std::string("template"))
    {
      p = encoder_input_tensors_.at(encoder_input_index_map_.at(image_type + std::string("_image"))).GetTensorMutableData<float>();
    }
  if(image_type == std::string("search"))
    p = decoder_input_tensors_.at(decoder_input_index_map_.at(image_type + std::string("_image"))).GetTensorMutableData<float>();


  int cols = img.cols;
  int rows = img.rows;
  int channels = img.channels();

  img.forEach<cv::Vec3b>([&](const cv::Vec3b &v, const int position[]) {
      int offset =  (position[0] * rows + position[1]) * channels;
      p[offset] = (v[0] / 255.0 - img_mean_[0]) / img_std_[0];
      p[offset + 1] = (v[1] / 255.0 - img_mean_[1]) / img_std_[1];
      p[offset + 2] = (v[2] / 255.0 - img_mean_[2]) / img_std_[2];
    });


  /* position embedding */
  int feat_size = 0;
  if(image_type == std::string("template")) feat_size = template_feat_size_;
  if(image_type == std::string("search")) feat_size = search_feat_size_;
  std::string input_name = image_type + std::string("_pos_embed");

  if(bounds[0] >= 1 || bounds[1] >= 1 || bounds[2] < feat_size - 1 || bounds[3] < feat_size - 1)
    {
      std::vector<int> discretized_mask{(int)std::ceil(bounds[0]), (int)std::ceil(bounds[1]), (int)std::floor(bounds[2]), (int)std::floor(bounds[3])};
      if(image_type == std::string("template"))
        {
          int index = encoder_input_index_map_.at(input_name);
          makePositionEmbedding(encoder_input_tensors_.at(index).GetTensorMutableData<float>(),
                                feat_size, discretized_mask);
        }
      if(image_type == std::string("search"))
        {
          int index = decoder_input_index_map_.at(input_name);
          makePositionEmbedding(decoder_input_tensors_.at(index).GetTensorMutableData<float>(),
                                feat_size, discretized_mask);
        }
    }
  else
    {
      int size = transformer_dim_ * feat_size * feat_size * sizeof(float);
      if(image_type == std::string("template"))
        {
          int index = encoder_input_index_map_.at(input_name);
          auto tensor_info = encoder_input_tensors_.at(index).GetTensorTypeAndShapeInfo();
          std::vector<int64_t> output_node_dims;
          output_node_dims = tensor_info.GetShape();

          std::memcpy(encoder_input_tensors_.at(index).GetTensorMutableData<void>(),
                      (void*)default_template_pos_embed_.data, size);
       }

      if(image_type == std::string("search"))
        {
          int index = decoder_input_index_map_.at(input_name);
          std::memcpy(decoder_input_tensors_.at(index).GetTensorMutableData<void>(),
                      (void*)default_search_pos_embed_.data, size);
        }

    }
}
#endif

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(jsk_perception::TrTrTrackerRos, nodelet::Nodelet);
