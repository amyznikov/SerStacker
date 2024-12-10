/*
 * c_morphology_routine.cc
 *
 *  Created on: Jun 15, 2023
 *      Author: amyznikov
 */

#include "c_morphology_routine.h"

void c_morphology_routine::get_parameters(std::vector<c_ctrl_bind> * ctls)
{
  BIND_PCTRL(ctls, operation, "Type of a morphological operation");
  BIND_PCTRL(ctls, se_shape, "Shape of structuring element");
  BIND_PCTRL(ctls, se_size, "Size of structuring element");
  BIND_PCTRL(ctls, anchor, "Anchor position with the kernel. Negative values mean that the anchor is at the kernel center");
  BIND_PCTRL(ctls, iterations, "Number of times erosion and dilation are applied");
  BIND_PCTRL(ctls, borderType, "Pixel extrapolation method, see #BorderTypes. BORDER_WRAP is not supported");
  BIND_PCTRL(ctls, borderValue, "Border value in case of a constant border");
  BIND_PCTRL(ctls, input_channel, "Input data source");
  BIND_PCTRL(ctls, output_channel, "Output destination");
}

bool c_morphology_routine::serialize(c_config_setting settings, bool save)
{
  if( base::serialize(settings, save) ) {
    SERIALIZE_PROPERTY(settings, save, *this, operation);
    SERIALIZE_PROPERTY(settings, save, *this, se_shape);
    SERIALIZE_PROPERTY(settings, save, *this, se_size);
    SERIALIZE_PROPERTY(settings, save, *this, anchor);
    SERIALIZE_PROPERTY(settings, save, *this, iterations);
    SERIALIZE_PROPERTY(settings, save, *this, borderType);
    SERIALIZE_PROPERTY(settings, save, *this, borderValue);
    SERIALIZE_PROPERTY(settings, save, *this, input_channel);
    SERIALIZE_PROPERTY(settings, save, *this, output_channel);
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

