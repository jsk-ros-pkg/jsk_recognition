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

#pragma once
#include <cv_bridge/cv_bridge.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_topic_tools/diagnostic_nodelet.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <omp.h>
#include <ros/ros.h>

#ifdef TENSORRT
#include <NvInfer.h>
#include <common.h>
#include <buffers.h>
#endif

#ifdef ONNXRT
#include <onnxruntime_cxx_api.h>
#endif


namespace trtr
{
  class Base
  {
  public:
    Base() {}
    ~Base() {}

    const cv::Point getBboxLT() const { return cv::Point(cx_ - w_/2, cy_ - h_/2);}
    const cv::Point getBboxRB() const { return cv::Point(cx_ + w_/2, cy_ + h_/2);}

    bool init(std::string encoder_model_file, std::string decoder_model_file, float score_threshold, float cosine_window_factor, int cosine_window_step, float size_lpf_factor);
    void reset(const std::vector<float>& bbox, const cv::Mat& img);
    void track(const cv::Mat& img);

  protected:

    void getSiamFCLikeScale(const int& template_img_size,  const float& w, const float& h, float& s_z, float& scale_z);
    void centerCropping(const cv::Mat& img, const std::vector<float>& bbox, const int& out_sz, const cv::Scalar& padding, cv::Mat& cropped_img);
    void makePositionEmbedding(float* p, int feat_size, std::vector<int> mask_bounds = std::vector<int>(0));
    void createCosineWindow();

    virtual bool loadModels(std::string encoder_model_file, std::string decoder_model_file) { return false; }
    virtual void encoderInference(const cv::Mat& img, const std::vector<float>& bounds) = 0;
    virtual void decoderInference(const cv::Mat& img, const std::vector<float>& bounds) = 0;
    virtual void processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type) = 0;

    /* visual tracking */
    cv::Vec3f img_mean_, img_std_;
    float cx_, cy_, w_, h_;
    int template_img_size_, search_img_size_;
    int template_feat_size_, search_feat_size_;
    int transformer_dim_;
    std::vector<float> pos_emded_dim_t_;
    cv::Mat default_template_pos_embed_;
    cv::Mat default_search_pos_embed_;

    cv::Mat heatmap_;
    cv::Mat bbox_reg_;
    cv::Mat bbox_wh_;

    /* postproces */
    cv::Mat cosine_window_;
    /*  hyper-parameter */
    float cosine_window_factor_;
    int cosine_window_step_;
    float size_lpf_factor_;
    float score_threshold_;
  };

#ifdef TENSORRT
  class TensorRT: public Base
  {
    template <typename T>
    struct TrtDestroyer
    {
      void operator()(T* t) { t->destroy(); }
    };

    template <typename T> using TrtUniquePtr = std::unique_ptr<T, TrtDestroyer<T> >;

    class Logger : public nvinfer1::ILogger {
    public:
      void log(nvinfer1::ILogger::Severity severity, const char* msg) override
      {
        // suppress information level log
        if (severity == Severity::kINFO) return;
        std::cout << msg << std::endl;
      }
    };

    struct InferDeleter
    {
      template <typename T>
      void operator()(T* obj) const
      {
        if (obj) obj->destroy();
      }
    };

  public:
    TensorRT() {}
    ~TensorRT(){}

  private:
    Logger logger_;
    std::shared_ptr<nvinfer1::ICudaEngine> encoder_engine_;
    std::shared_ptr<nvinfer1::ICudaEngine> decoder_engine_;
    std::shared_ptr<samplesCommon::BufferManager> encoder_buffers_;
    std::shared_ptr<samplesCommon::BufferManager> decoder_buffers_;
    std::shared_ptr<nvinfer1::IExecutionContext> encoder_context_;
    std::shared_ptr<nvinfer1::IExecutionContext> decoder_context_;

    nvinfer1::ICudaEngine* loadEngine(const std::string& model_file);
    bool loadModels(std::string encoder_model_file, std::string decoder_model_file) override;
    void encoderInference(const cv::Mat& img, const std::vector<float>& bounds) override;
    void decoderInference(const cv::Mat& img, const std::vector<float>& bounds) override;
    void processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type) override;
  };
#endif

#ifdef ONNXRT
  // ONNX RunTime
  class ONNX: public Base
  {
  public:
    ONNX() {}
    ~ONNX(){}

  private:

    std::shared_ptr<Ort::Session> encoder_session_;
    std::shared_ptr<Ort::Session> decoder_session_;
    Ort::AllocatorWithDefaultOptions allocator_;

    std::vector<Ort::Value> encoder_input_tensors_;
    std::vector<Ort::Value> decoder_input_tensors_;
    std::vector<char*> encoder_input_names_;
    std::vector<char*> decoder_input_names_;
    std::vector<char*> encoder_output_names_;
    std::vector<char*> decoder_output_names_;
    std::map<std::string, int> encoder_input_index_map_;
    std::map<std::string, int> decoder_input_index_map_;
    std::map<std::string, int> encoder_output_index_map_;
    std::map<std::string, int> decoder_output_index_map_;

    Ort::Env env_;

    bool loadModels(std::string encoder_model_file, std::string decoder_model_file) override;
    void encoderInference(const cv::Mat& img, const std::vector<float>& bounds) override;
    void decoderInference(const cv::Mat& img, const std::vector<float>& bounds) override;
    void processInput(const cv::Mat& img, const std::vector<float>& bounds, const std::string image_type) override;
  };
#endif
};

namespace jsk_perception
{
  class TrTrTrackerRos: public jsk_topic_tools::DiagnosticNodelet
  {
  public:
    TrTrTrackerRos():
      DiagnosticNodelet("TrTrTracking"),
      rect_initialized_(false) {}
  private:
    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      jsk_recognition_msgs::RectArray> ApproximateSyncPolicy;
    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      jsk_recognition_msgs::RectArray> ExactSyncPolicy;

    void onInit();
    void subscribe();
    void unsubscribe();

    void getTrackingResult(const sensor_msgs::Image::ConstPtr& image_msg);
    void setInitialRect(const sensor_msgs::Image::ConstPtr& img_msg,
                          const jsk_recognition_msgs::RectArray::ConstPtr& rect_msg);


    ros::Publisher rect_pub_;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    boost::shared_ptr<image_transport::ImageTransport> it_;

    boost::shared_ptr<message_filters::Synchronizer<ExactSyncPolicy> > sync_;
    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > async_;
    message_filters::Subscriber<sensor_msgs::Image> image_to_init_sub_;
    message_filters::Subscriber<jsk_recognition_msgs::RectArray> rect_to_init_sub_;

    std::shared_ptr<trtr::Base> tracker_;

    boost::mutex mutex_;
    bool rect_initialized_;
    bool approximate_sync_;
    int queue_size_;
  };
};

