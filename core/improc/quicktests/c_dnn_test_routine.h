/*
 * c_dnn_test_routine.h
 *
 *  Created on: Apr 30, 2024
 *      Author: amyznikov
 *
 *  From <https://github.com/ultralytics/ultralytics/tree/main/examples/YOLOv8-CPP-Inference>
 */

#pragma once
#ifndef __c_dnn_test_routine_h__
#define __c_dnn_test_routine_h__

#include <core/improc/c_image_processor.h>

#if HAVE_OpenCV_dnn
#include <opencv2/dnn.hpp>
#endif // HAVE_OpenCV_dnn

class c_dnn_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dnn_test_routine,
      "dnn_test", "c_dnn_test_routine");

  const std::string & onnx_model_path() const
  {
    return onnx_model_path_;
  }

  void set_onnx_model_path(const std::string & v)
  {
    onnx_model_path_ = v;
    initialized_ = false;
  }

  const std::string & classes_text_file() const
  {
    return classes_text_file_;
  }

  void set_classes_text_file(const std::string & v)
  {
    classes_text_file_ = v;
    initialized_ = false;
  }

  const cv::Size & model_input_shape() const
  {
    return model_input_shape_;
  }

  void set_model_input_shape(const cv::Size & v)
  {
    model_input_shape_ = v;
    initialized_ = false;
  }

  bool run_with_cuda() const
  {
    return run_with_cuda_;
  }

  void set_run_with_cuda(bool v)
  {
    run_with_cuda_ = v;
    initialized_ = false;
  }


  void get_parameters(std::vector<c_ctrl_bind> * ctls) override;
  bool serialize(c_config_setting settings, bool save) override;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) override;

  struct Detection
  {
    int class_id = 0;
    std::string className;
    float confidence = 0.0f;
    cv::Scalar color;
    cv::Rect box;
  };

  class Inference
  {
  public:

    Inference();

    bool initialize(const std::string & onnxModelPath, const cv::Size & modelInputShape = { 640, 640 },
        const std::string & classesTxtFile = "", const bool & runWithCuda = false);

    std::vector<Detection> runInference(const cv::Mat & input);

  private:
    bool loadClassesFromFile();
    bool loadOnnxNetwork();
    cv::Mat formatToSquare(const cv::Mat & source);

    std::string modelPath;
    std::string classesPath;
    bool cudaEnabled = false;

    std::vector<std::string> classes { "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
        "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse",
        "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
        "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
        "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant",
        "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven",
        "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush" };

    cv::Size modelShape;
    cv::Size outputShape;
    float modelConfidenceThreshold = 0.25f;
    float modelScoreThreshold = 0.25f;
    float modelNMSThreshold = 0.25;

    bool letterBoxForSquare = false;

#if HAVE_OpenCV_dnn
    cv::dnn::Net net;
#endif //
  };


protected:
  Inference inference_;
  std::string onnx_model_path_;
  std::string classes_text_file_;
  cv::Size model_input_shape_;// = cv::Size(640, 640);
  bool run_with_cuda_ = false;
  bool initialized_ = false;
};

#endif /* __c_dnn_test_routine_h__ */
