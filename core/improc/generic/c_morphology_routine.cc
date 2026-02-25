/*
 * c_morphology_routine.cc
 *
 *  Created on: Jun 15, 2023
 *      Author: amyznikov
 */

#include "c_morphology_routine.h"

void c_morphology_routine::getcontrols(c_control_list & ctls, const ctlbind_context & ctx)
{
   ctlbind(ctls, "operation", ctx(&this_class::_operation), "");
   ctlbind(ctls, "se_shape", ctx, &this_class::se_shape, &this_class::set_se_shape, "");
   ctlbind(ctls, "se_size", ctx, &this_class::se_size, &this_class::set_se_size, "");
   ctlbind(ctls, "anchor", ctx, &this_class::anchor, &this_class::set_anchor, "");
   ctlbind(ctls, "iterations", ctx(&this_class::_iterations), "");
   ctlbind(ctls, "borderType", ctx(&this_class::_borderType), "");
   ctlbind(ctls, "borderValue", ctx(&this_class::_borderValue), "");
   ctlbind(ctls, "output to", ctx(&this_class::_output_channel),  "Output image name");
}

bool c_morphology_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, se_shape);
    SERIALIZE_PROPERTY(settings, save, *this, se_size);
    SERIALIZE_PROPERTY(settings, save, *this, anchor);
    SERIALIZE_OPTION(settings, save, *this, _operation);
    SERIALIZE_OPTION(settings, save, *this, _iterations);
    SERIALIZE_OPTION(settings, save, *this, _borderType);
    SERIALIZE_OPTION(settings, save, *this, _borderValue);
    SERIALIZE_OPTION(settings, save, *this, _input_channel);
    SERIALIZE_OPTION(settings, save, *this, _output_channel);
    return true;
  }
  return false;
}

bool c_morphology_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( SE.empty() ) {
    SE =
        cv::getStructuringElement(_se_shape,
            _se_size,
            _anchor);
  }

  cv::Mat src, dst;

  switch (_input_channel) {
    case DATA_CHANNEL::IMAGE:
      src = image.getMat();
      break;
    case DATA_CHANNEL::MASK:
      src = mask.getMat();
      break;
  }

  apply_morphology(src, dst,
      _operation,
      SE,
      _anchor,
      _iterations,
      _borderType,
      _borderValue);


  switch (_output_channel) {
    case DATA_CHANNEL::IMAGE:
      image.move(dst);
      break;
    case DATA_CHANNEL::MASK:
      mask.create(dst.size(), CV_8UC1);
      mask.setTo(cv::Scalar::all(0));
      mask.setTo(255, dst != 0);
      break;
  }

  return true;
}

