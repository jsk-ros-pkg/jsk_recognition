#!/bin/bash

# encoder: onnx format -> tensorrt format (FP32)
/usr/src/tensorrt/bin/trtexec --onnx=./trained_data/trtr_resnet50_encoder.onnx --tacticSources=-cublasLt,+cublas --workspace=1024 --explicitBatch --saveEngine=./trained_data/trtr_resnet50_encoder.trt --shapes=input1:1x3x127x127,input2:1x256x16x16

# decoder: onnx format -> tensorrt format (FP32)
/usr/src/tensorrt/bin/trtexec --onnx=./trained_data/trtr_resnet50_decoder.onnx --tacticSources=-cublasLt,+cublas --workspace=1024 --explicitBatch  --saveEngine=./trained_data/trtr_resnet50_decoder.trt --shapes=input1:1x3x255x255,input2:1x256x32x32,input3:1x256x16x16,input4:256x1x256

# encoder: onnx format -> tensorrt format (FP16)
/usr/src/tensorrt/bin/trtexec --onnx=./trained_data/trtr_resnet50_encoder.onnx --tacticSources=-cublasLt,+cublas --workspace=1024 --explicitBatch --saveEngine=./trained_data/trtr_resnet50_encoder_fp16.trt --shapes=input1:1x3x127x127,input2:1x256x16x16 --fp16

# decoder: onnx format -> tensorrt format (FP16)
/usr/src/tensorrt/bin/trtexec --onnx=./trained_data/trtr_resnet50_decoder.onnx --tacticSources=-cublasLt,+cublas --workspace=1024 --explicitBatch  --saveEngine=./trained_data/trtr_resnet50_decoder_fp16.trt --shapes=input1:1x3x255x255,input2:1x256x32x32,input3:1x256x16x16,input4:256x1x256 --fp16
