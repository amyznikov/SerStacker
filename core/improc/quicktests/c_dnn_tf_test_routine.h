/*
 * c_dnn_tf_test_routine.h
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_dnn_tf_test_routine_h__
#define __c_dnn_tf_test_routine_h__

#include <core/improc/c_image_processor.h>

#if HAVE_OpenCV_dnn
#include <opencv2/dnn.hpp>
#endif // HAVE_OpenCV_dnn

class c_dnn_tf_test_routine :
    public c_image_processor_routine
{
public:
  DECLATE_IMAGE_PROCESSOR_CLASS_FACTORY(c_dnn_tf_test_routine,
      "c_dnn_tf_test", "c_dnn_tf_test_routine");

  const std::string & model_path() const
  {
    return _model_path;
  }

  void set_model_path(const std::string & v)
  {
    _model_path = v;
    _initialized = false;
  }

  bool serialize(c_config_setting settings, bool save) final;
  bool process(cv::InputOutputArray image, cv::InputOutputArray mask = cv::noArray()) final;
  static void getcontrols(c_control_list & ctls, const ctlbind_context & ctx);

protected:
  std::string _model_path;
//#if HAVE_OpenCV_dnn
  cv::dnn::Net net;
//#endif //

  bool _initialized = false;
};

#endif /* __c_dnn_tf_test_routine_h__ */
