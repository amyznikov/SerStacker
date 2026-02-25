/*
 * c_segformer2_routine.cc
 *
 *  Created on: May 11, 2025
 *      Author: amyznikov
 */

#include "c_segformer2_routine.h"
#include <core/debug.h>


// Cityscapes color palette (19 classes)
static const cv::Vec3b cityscapes_palette[] = {
    {128, 64, 128}, {244, 35, 232}, {70, 70, 70}, {102, 102, 156},
    {190, 153, 153}, {153, 153, 153}, {250, 170, 30}, {220, 220, 0},
    {107, 142, 35}, {152, 251, 152}, {70, 130, 180}, {220, 20, 60},
    {255, 0, 0}, {0, 0, 142}, {0, 0, 70}, {0, 60, 100},
    {0, 80, 100}, {0, 0, 230}, {119, 11, 32}
};
//
//
//void c_segformer2_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
//{
//  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, onnx_preproc_model_path, "preproc model path", "Specify onnx_preproc_model_path");
//  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, onnx_model_path, "model path", "Specify onnx_model_path");
//}

bool c_segformer2_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, onnx_preproc_model_path);
    SERIALIZE_PROPERTY(settings, save, *this, onnx_model_path);
    return true;
  }
  return false;
}


bool c_segformer2_routine::initialize()
{
//  CF_DEBUG("H");
//  _env = Ort::Env(ORT_LOGGING_LEVEL_WARNING, "SegFormer");
//  CF_DEBUG("H");
//  //_session_options.SetIntraOpNumThreads(1);
//  CF_DEBUG("H");

  return true;
}

void c_segformer2_routine::release_session()
{
#if HAVE_OpenCV_dnn
  if ( !_preproc_net.empty() ) {
    _preproc_net = cv::dnn::Net();
  }
  if ( !_model_net.empty() ) {
    _model_net = cv::dnn::Net();
  }
#endif  // HAVE_OpenCV_dnn
}

void c_segformer2_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_browse_for_file(ctls, "onnx_preproc_model", ctx, &this_class::onnx_preproc_model_path, &this_class::set_onnx_preproc_model_path, "Specify onnx preproc model path");
  ctlbind_browse_for_file(ctls, "onnx_model", ctx, &this_class::onnx_model_path, &this_class::set_onnx_model_path, "Specify onnx_model path");
}

bool c_segformer2_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( onnx_model_path().empty() || onnx_preproc_model_path().empty() ) {
    release_session();
    return true;
  }

  mask.release();

  try {

    CF_DEBUG("1 _preproc_net.empty()=%d", _preproc_net.empty());
    if( _preproc_net.empty() ) {
      _preproc_net = cv::dnn::readNetFromONNX(onnx_preproc_model_path());
//      _preproc_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
//      _preproc_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    CF_DEBUG("2 _preproc_net.empty()=%d", _preproc_net.empty());


    CF_DEBUG("1 _model_net.empty()=%d", _model_net.empty());
    if( _model_net.empty() ) {
      _model_net = cv::dnn::readNetFromONNX(onnx_model_path());
//      _model_net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
//      _model_net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    }
    CF_DEBUG("2 _model_net.empty()=%d", _model_net.empty());

    const cv::Size modelSize =
        cv::Size (1024, 1024);

    cv::Mat resized;
    cv::resize(image.getMat(), resized, modelSize);

    CF_DEBUG("resized: %dx%d channels=%d depth=%d", resized.cols, resized.rows, resized.channels(), resized.depth());

    // Create input blob (BGR→RGB conversion)
    cv::Mat blob =
        cv::dnn::blobFromImage(resized, 1.0, modelSize, cv::Scalar(),
            true, false);

    CF_DEBUG("blob: size: {%d %d %d %d}  (%dx%d) channels=%d depth=%d",
        blob.size[0], blob.size[1], blob.size[2], blob.size[3],
        blob.cols, blob.rows, blob.channels(), blob.depth());

    // Preprocess
    _preproc_net.setInput(blob);

    CF_DEBUG("_preproc_net.forward()");

    cv::Mat normalized = _preproc_net.forward();
    CF_DEBUG("normalized: size: {%d %d %d %d}  (%dx%d) channels=%d depth=%d",
        normalized.size[0], normalized.size[1], normalized.size[2], normalized.size[3],
        normalized.cols, normalized.rows, normalized.channels(), normalized.depth());




    // Segmentation
    _model_net.setInput(normalized);

    CF_DEBUG("_model_net.forward()");

    cv::Mat logits = _model_net.forward();
    CF_DEBUG("logits: size: {%d %d %d %d}  (%dx%d) channels=%d depth=%d",
        logits.size[0], logits.size[1], logits.size[2], logits.size[3],
        logits.cols, logits.rows, logits.channels(), logits.depth());


    // Process output tensor (NCHW format: [1, 19, 1024, 1024])
    const int num_classes = 19;
    const int height = modelSize.width;
    const int width = modelSize.height;


    // Get pointer to output data
    float* data = (float*)logits.data;

    // Create class map
    cv::Mat class_map(height, width, CV_8UC1);

    // Calculate argmax for each pixel
    for( int y = 0; y < height; y++ ) {
      for( int x = 0; x < width; x++ ) {
        float max_score = -std::numeric_limits<float>::max();
        int max_idx = 0;

        // Find class with maximum score
        for( int c = 0; c < num_classes; c++ ) {
          // Access data in NCHW layout: [batch][channel][y][x]
          float score = data[c * height * width + y * width + x];
          if( score > max_score ) {
            max_score = score;
            max_idx = c;
          }
        }
        class_map.at<uchar>(y, x) = max_idx;
      }
    }

    CF_DEBUG("class_map: %dx%d channels=%d depth=%d", class_map.cols, class_map.rows, class_map.channels(), class_map.depth());

    // Create color mask
    cv::Mat color_mask(height, width, CV_8UC3);
    for( int y = 0; y < height; y++ ) {
      for( int x = 0; x < width; x++ ) {
        uchar class_id = class_map.at<uchar>(y, x);
        color_mask.at<cv::Vec3b>(y, x) = cityscapes_palette[class_id];
      }
    }

    CF_DEBUG("color_mask: %dx%d channels=%d depth=%d", color_mask.cols, color_mask.rows, color_mask.channels(), color_mask.depth());


    // Blend with original image
    // Blend with original frame
    cv::addWeighted(resized, 0.5, color_mask, 0.5, 0, image);

    CF_DEBUG("image: %dx%d channels=%d depth=%d", image.cols(), image.rows(), image.channels(), image.depth());

  }
  catch (const std::exception & e) {
    CF_ERROR("std::exception in c_segformer2_routine::process(): %s", e.what());
    return false;
  }

  catch (...) {
    CF_ERROR("unknown exception in c_segformer2_routine::process()");
    return false;
  }

  return true;
}



