/*
 * c_dnn_tf_test_routine.cc
 *
 *  Created on: May 4, 2024
 *      Author: amyznikov
 */

#include "c_dnn_tf_test_routine.h"

void c_dnn_tf_test_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
  ctlbind_browse_for_file(ctls, "model_path", ctx, &this_class::model_path, &this_class::set_model_path, "Specify model_path");
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
  if ( !_initialized ) {

    // net = cv::dnn::readNetFromTensorflow(model_path_, config);

  }

  return true;
}
