/*
 * c_segformer2_routine.h
 *
 *  Created on: May 11, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_segformer2_routine_h__
#define __c_segformer2_routine_h__

#include <core/improc/c_image_processor.h>

#if HAVE_OpenCV_dnn
# include <opencv2/dnn.hpp>
#endif // HAVE_OpenCV_dnn

class c_segformer2_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_segformer2_routine,
      "segformer2", "test segformer2");

  void set_onnx_preproc_model_path(const std::string & v)
  {
    _onnx_preproc_model_path = v;
    release_session();
  }

  const std::string & onnx_preproc_model_path() const
  {
    return _onnx_preproc_model_path;
  }

  void set_onnx_model_path(const std::string & v)
  {
    _onnx_model_path = v;
    release_session();
  }

  const std::string & onnx_model_path() const
  {
    return _onnx_model_path;
  }

  bool initialize() final;
  void get_parameters(std::vector<c_ctrl_bind> * ctls) final;
  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;

protected:
  void release_session();

protected:
  std::string _onnx_preproc_model_path = "preproc.onnx";
  std::string _onnx_model_path = "segformer.onnx";

#if HAVE_OpenCV_dnn
  cv::dnn::Net _preproc_net;
  cv::dnn::Net _model_net;
#endif  // HAVE_OpenCV_dnn
};

#endif /* __c_segformer2_routine_h__ */
