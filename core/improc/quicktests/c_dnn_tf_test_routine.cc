/*
 * c_dnn_tf_test_routine.cc
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#include "c_dnn_tf_test_routine.h"

void c_dnn_tf_test_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_BROWSE_FOR_EXISTING_FILE_CTRL(ctls, model_path, "model_path", "Specify model_path");
}

bool c_dnn_tf_test_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this,  model_path);
    return true;
  }
  return false;
}

bool c_dnn_tf_test_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if ( !initialized_ ) {

    // net = cv::dnn::readNetFromTensorflow(model_path_, config);

  }

  return true;
}
