/*
 * c_dnn_test_routine.cc
 *
 *  Created on: Apr 30, 2024
 *      Author: amyznikov
 */

#include "c_dnn_test_routine.h"
#include <random>
#include <fstream>
// #include <opencv2/cudaimgproc.hpp>

#include <core/debug.h>

///////////////////////////////////////////////////////////////////////////////////////////////////

namespace {

typedef c_dnn_test_routine::Detection Detection;
typedef c_dnn_test_routine::Inference Inference;

} // namespace


c_dnn_test_routine::Inference::Inference()
{

}

bool Inference::initialize(const std::string & onnxModelPath, const cv::Size & modelInputShape,
    const std::string & classesTxtFile, const bool & runWithCuda)
{
  modelPath = onnxModelPath;
  modelShape = modelInputShape;
  classesPath = classesTxtFile;
  cudaEnabled = runWithCuda;

  return loadOnnxNetwork();
}

std::vector<Detection> Inference::runInference(const cv::Mat & modelInput)
{
#if HAVE_OpenCV_dnn


  const cv::Size size =
      modelShape.empty() ? outputShape :
          modelShape;

  cv::Mat blob;
  cv::dnn::blobFromImage(modelInput, blob, 1.0 / 255.0, size, cv::Scalar(), true, false);
  net.setInput(blob);

  const std::vector<cv::String> outBlobName =
      net.getUnconnectedOutLayersNames();

  std::vector<cv::Mat> outputs;
  net.forward(outputs, outBlobName);

  int rows = outputs[0].size[1];
  int dimensions = outputs[0].size[2];

  bool yolov8 = false;
  // yolov5 has an output of shape (batchSize, 25200, 85) (Num classes + box[x,y,w,h] + confidence[c])
  // yolov8 has an output of shape (batchSize, 84,  8400) (Num classes + box[x,y,w,h])

  if( dimensions > rows )  { // Check if the shape[2] is more than shape[1] (yolov8)

    yolov8 = true;
    rows = outputs[0].size[2];
    dimensions = outputs[0].size[1];

    outputs[0] = outputs[0].reshape(1, dimensions);
    cv::transpose(outputs[0], outputs[0]);
  }

  float * data = (float*) outputs[0].data;
  const float x_factor = modelInput.cols / (float)size.width;
  const float y_factor = modelInput.rows / (float)size.height;

  std::vector<int> class_ids;
  std::vector<float> confidences;
  std::vector<cv::Rect> boxes;

  for( int i = 0; i < rows; ++i ) {
    if( yolov8 ) {

      float * classes_scores =
          data + 4;

      cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
      cv::Point class_id;
      double maxClassScore;

      minMaxLoc(scores, 0, &maxClassScore, 0, &class_id);

      if( maxClassScore > modelScoreThreshold ) {
        confidences.push_back(maxClassScore);
        class_ids.push_back(class_id.x);

        const float x = data[0];
        const float y = data[1];
        const float w = data[2];
        const float h = data[3];

        const int left = int((x - 0.5 * w) * x_factor);
        const int top = int((y - 0.5 * h) * y_factor);

        const int width = int(w * x_factor);
        const int height = int(h * y_factor);

        boxes.push_back(cv::Rect(left, top, width, height));
      }
    }
    else { // yolov5

      float confidence = data[4];

      if( confidence >= modelConfidenceThreshold ) {

        float * classes_scores =
            data + 5;

        cv::Mat scores(1, classes.size(), CV_32FC1, classes_scores);
        cv::Point class_id;
        double max_class_score;

        minMaxLoc(scores, 0, &max_class_score, 0, &class_id);

        if( max_class_score > modelScoreThreshold ) {
          confidences.push_back(confidence);
          class_ids.push_back(class_id.x);

          const float x = data[0];
          const float y = data[1];
          const float w = data[2];
          const float h = data[3];

          const int left = int((x - 0.5 * w) * x_factor);
          const int top = int((y - 0.5 * h) * y_factor);

          const int width = int(w * x_factor);
          const int height = int(h * y_factor);

          boxes.push_back(cv::Rect(left, top, width, height));
        }
      }
    }

    data += dimensions;
  }

  std::vector<int> nms_result;
  cv::dnn::NMSBoxes(boxes, confidences, modelScoreThreshold, modelNMSThreshold, nms_result);

  std::vector<Detection> detections;

  for( size_t i = 0, n = nms_result.size(); i < n; ++i ) {

    const int idx =
        nms_result[i];

    Detection result;
    result.class_id = class_ids[idx];
    result.confidence = confidences[idx];

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(100, 255);
    result.color = cv::Scalar(dis(gen), dis(gen), dis(gen));

    result.className = classes[result.class_id];
    result.box = boxes[idx];

    detections.push_back(result);
  }

  return detections;

#else
  return std::vector<Detection>();
#endif
}

bool Inference::loadClassesFromFile()
{
#if HAVE_OpenCV_dnn
  std::ifstream inputFile(classesPath);

  if( inputFile.is_open() ) {

    std::string classLine;

    while (std::getline(inputFile, classLine)) {
      classes.push_back(classLine);
    }

    inputFile.close();

    return true;
  }
#endif

  return false;
}

bool Inference::loadOnnxNetwork()
{
#if HAVE_OpenCV_dnn
 try {

    net = cv::dnn::readNetFromONNX(modelPath);

    std::vector<cv::dnn::MatShape> inLayerShapes;
    std::vector<cv::dnn::MatShape> outLayerShapes;
    net.getLayerShapes(cv::dnn::MatShape(), 0, inLayerShapes, outLayerShapes);

    outputShape.width = outputShape.height = -1;

    if( outLayerShapes.size() > 0 ) {
      const cv::dnn::MatShape & s = outLayerShapes.back();
      if( s.size() == 4 ) {
        outputShape.width = s[3];
        outputShape.height = s[2];
      }
    }

    CF_DEBUG("modelShape: %dx%d outputShape: %dx%d",
        modelShape.width, modelShape.height,
        outputShape.width, outputShape.height);

    if ( false ) {

      CF_DEBUG("in layer shapes: %zu", inLayerShapes.size());
      for( const auto & n1 : inLayerShapes ) {
        std::string text = "- shape:";
        for( const auto n2 : n1 ) {
          text.append("  " + std::to_string(n2));
        }
        CF_DEBUG("- shape: %s", text.c_str());
      }

      CF_DEBUG("out layer shapes: %zu", outLayerShapes.size());
      for( const auto & n1 : outLayerShapes ) {
        std::string text = "- shape:";
        for( const auto n2 : n1 ) {
          text.append("  " + std::to_string(n2));
        }
        CF_DEBUG("- shape: %s", text.c_str());
      }

    }


    if( cudaEnabled ) {
      CF_DEBUG("Run on CUDA");
      net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
      net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    }
    else {
      CF_DEBUG("Run on CPU");
      net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    return true;
  }
  catch( const std::exception & e ) {
    CF_ERROR("Exception in loadOnnxNetwork() : %s",
        e.what());
  }
  catch( ... ) {
    CF_ERROR("Unknown Exception in loadOnnxNetwork()");
  }
#endif

  return false;
}

cv::Mat Inference::formatToSquare(const cv::Mat & source)
{
  int col = source.cols;
  int row = source.rows;
  int _max = MAX(col, row);

  cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);

  source.copyTo(result(cv::Rect(0, 0, col, row)));

  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void c_dnn_test_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, onnx_model_path, "onnx_model_path", "Specify onnx_model_path");
  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, classes_text_file, "classes_text_file", "Specify classes_text_file");
  BIND_CTRL(ctls, model_input_shape, "model_input_shape", "Specify model_input_shape");
  BIND_CTRL(ctls, run_with_cuda, "run_with_cuda", "Spefify run_with_cuda");
}

bool c_dnn_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {

    SERIALIZE_PROPERTY(settings, save, *this, onnx_model_path);
    SERIALIZE_PROPERTY(settings, save, *this, classes_text_file);
    SERIALIZE_PROPERTY(settings, save, *this, model_input_shape);
    SERIALIZE_PROPERTY(settings, save, *this, run_with_cuda);

    return true;
  }
  return false;
}

bool c_dnn_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( !initialized_ ) {

    initialized_ =
        inference_.initialize(onnx_model_path_,
            model_input_shape_,
            classes_text_file_,
            run_with_cuda_);

    if( !initialized_ ) {
      CF_ERROR("inference_.initialize() fails");
      return false;
    }
  }

  std::vector<Detection> output =
      inference_.runInference(image.getMat());

  CF_DEBUG("output.size=%zu", output.size());

  for ( const Detection & d: output ) {

    CF_DEBUG("box {%d %d %dx%d} conf=%g '%s'", d.box.x, d.box.y, d.box.width, d.box.height, d.confidence, d.className.c_str());

    cv::rectangle(image, d.box, d.color, 3, cv::LINE_4, 0);
  }

  return true;

}

///////////////////////////////////////////////////////////////////////////////////////////////////

