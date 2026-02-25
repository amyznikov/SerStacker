/*
 * c_segformer_routine.cc
 *
 *  Created on: Mar 4, 2025
 *      Author: amyznikov
 */

#include "c_segformer_routine.h"
#include <core/debug.h>

static const int INPUT_WIDTH = 520;
static const int INPUT_HEIGHT = 520;
//static const int INPUT_CHANNELS = 3;
//static const char *MODEL_PATH = "deeplabv3_plus_mobilenet.onnx";

// Colors for visualization
static const cv::Vec3b colorMap[21] = {
    { 0, 0, 0 }, { 128, 0, 0 }, { 0, 128, 0 }, { 128, 128, 0 }, { 0, 0, 128 },
    { 128, 0, 128 }, { 0, 128, 128 }, { 128, 128, 128 }, { 64, 0, 0 }, { 192, 0, 0 },
    { 64, 128, 0 }, { 192, 128, 0 }, { 64, 0, 128 }, { 192, 0, 128 }, { 64, 128, 128 },
    { 192, 128, 128 }, { 0, 64, 0 }, { 128, 64, 0 }, { 0, 192, 0 }, { 128, 192, 0 },
    { 0, 64, 128 }
};

// Function to preprocess the frame
static cv::Mat preprocess_image(const cv::Mat & image)
{
  cv::Mat resized, blob;

  if( image.channels() == 3 ) {
    cv::resize(image, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), 0, 0, cv::INTER_AREA);
  }
  else {
    cv::cvtColor(image, resized, cv::COLOR_GRAY2BGR);
    cv::resize(resized, resized, cv::Size(INPUT_WIDTH, INPUT_HEIGHT), 0, 0, cv::INTER_AREA);
  }

  cv::dnn::blobFromImage(resized, blob, 1.0 / 255.0, cv::Size(INPUT_WIDTH, INPUT_HEIGHT),
      cv::Scalar(0, 0, 0), true, false);

  return blob;
}

// Function to process the output
cv::Mat process_output(const cv::Mat & output)
{
  cv::Mat3b segMap(INPUT_HEIGHT, INPUT_WIDTH);

  for( int y = 0; y < INPUT_HEIGHT; ++y ) {
    for( int x = 0; x < INPUT_WIDTH; ++x ) {
      const int maxIdx = output.at<float>(0, y, x); // Since output is [1, 520, 520]
      segMap(y, x) = colorMap[maxIdx % 21]; // Assign class color
    }
  }
  return segMap;
}

//void c_segformer_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
//{
//  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, onnx_model_path, "onnx_model_path", "Specify onnx_model_path");
//}

bool c_segformer_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, onnx_model_path);
    return true;
  }
  return false;
}


bool c_segformer_routine::initialize()
{
//  CF_DEBUG("H");
//  _env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "SegFormer");
//  CF_DEBUG("H");
//  //_session_options.SetIntraOpNumThreads(1);
//  CF_DEBUG("H");

  return true;
}

void c_segformer_routine::release_session()
{
#if HAVE_OpenCV_dnn
  if ( !_net.empty() ) {
    _net = cv::dnn::Net();
  }
#endif  // HAVE_OpenCV_dnn
}


void c_segformer_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_browse_for_file(ctls, "onnx_model", ctx, &this_class::onnx_model_path, &this_class::set_onnx_model_path, "Specify onnx_model_path");
}

bool c_segformer_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( onnx_model_path().empty() ) {
    release_session();
    return true;
  }

  try {

    if( _net.empty() ) {
      _net = cv::dnn::readNetFromONNX(onnx_model_path());
      _net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
      _net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }

    cv::Mat inputBlob = preprocess_image(image.getMat());

    // Set input and run inference
    _net.setInput(inputBlob, "image");
    cv::Mat outputBlob = _net.forward("mask");

    // Process segmentation output
    cv::Mat segmentationMap = process_output(outputBlob);

    // Resize segmentation map to match the original frame size
    cv::resize(segmentationMap, segmentationMap, image.getMat().size(), 0, 0, cv::INTER_NEAREST);

    // Blend with original frame
    cv::addWeighted(image.getMat(), 0.5, segmentationMap, 0.5, 0, image);
  }
  catch (const std::exception & e) {
    CF_ERROR("std::exception in c_segformer_routine::process(): %s", e.what());
    return false;
  }

  catch (...) {
    CF_ERROR("unknown exception in c_segformer_routine::process()");
    return false;
  }

  return true;
}


