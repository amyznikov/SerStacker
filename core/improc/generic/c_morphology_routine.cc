/*
 * c_morphology_routine.cc
 *
 *  Created on: Jun 15, 2023
 *      Author: amyznikov
 */

#include "c_morphology_routine.h"
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<c_morphology_routine::OPERATION>()
{
  static const c_enum_member members[] = {
      {c_morphology_routine::MORPH_ERODE, "ERODE", "cv::erode()"},
      {c_morphology_routine::MORPH_DILATE, "DILATE", "cv::dilate"},
      {c_morphology_routine::MORPH_OPEN, "OPEN", "opening operation"},
      {c_morphology_routine::MORPH_CLOSE, "CLOSE", "closing operation"},
      {c_morphology_routine::MORPH_GRADIENT, "GRADIENT", "morphological gradient"},
      {c_morphology_routine::MORPH_TOPHAT, "TOPHAT", "top hat"},
      {c_morphology_routine::MORPH_BLACKHAT, "BLACKHAT", "black hat"},
      {c_morphology_routine::MORPH_HITMISS, "HITMISS", "hit or miss"},
      {c_morphology_routine::MORPH_SMOOTH_OPEN, "SMOOTH_OPEN", "morphological_smooth_open"},
      {c_morphology_routine::MORPH_SMOOTH_CLOSE, "SMOOTH_CLOSE", "morphological_smooth_close"},
      {c_morphology_routine::MORPH_INTERNAL_GRADIENT, "INTERNAL_GRADIENT", "morphological_internal_gradient"},
      {c_morphology_routine::MORPH_EXTERNAL_GRADIENT, "EXTERNAL_GRADIENT", "morphological_external_gradient"},
      {c_morphology_routine::MORPH_LAPLACIAN, "LAPLACIAN", "morphological_laplacian"},
      {c_morphology_routine::MORPH_LAPLACIAN_ABS, "LAPLACIAN_ABS", "morphological laplacian absolute value"},
      {c_morphology_routine::MORPH_RAMPLEE, "RAMPLEE", "rampLee - \n"
          "Elementary grayscale morphological techniques can be used to distinguish smooth “ramp” edges from ripple “texture” edges."},
      {c_morphology_routine::MORPH_TEXLEE, "TEXLEE", "texLee - \n"
          "Elementary grayscale morphological techniques can be used to distinguish smooth “ramp” edges from ripple “texture” edges."},
      {c_morphology_routine::MORPH_GEO_FILL_HOLES4, "GEO_FILL_HOLES4", "geo_fill_holes"},
      {c_morphology_routine::MORPH_GEO_FILL_HOLES8, "GEO_FILL_HOLES8", "geo_fill_holes"},

      {c_morphology_routine::MORPH_GEO_OPEN4, "GEO_OPEN4", "geo_open(connectivity=4)"},
      {c_morphology_routine::MORPH_GEO_OPEN8, "GEO_OPEN8", "geo_open(connectivity=8)"},
      {c_morphology_routine::MORPH_GEO_CLOSE4, "GEO_CLOSE4", "geo_close(connectivity=4)"},
      {c_morphology_routine::MORPH_GEO_CLOSE8, "GEO_CLOSE8", "geo_close(connectivity=8)"},

      {c_morphology_routine::MORPH_ERODE},
  };

  return members;
}


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

  switch (_operation) {

    case MORPH_SMOOTH_OPEN:
      morphological_smooth_open(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_SMOOTH_CLOSE:
      morphological_smooth_close(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_INTERNAL_GRADIENT:
      morphological_internal_gradient(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_EXTERNAL_GRADIENT:
      morphological_external_gradient(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_LAPLACIAN:
      morphological_laplacian(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_LAPLACIAN_ABS:
      morphological_laplacian_abs(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_RAMPLEE:
      rampLee(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_TEXLEE:
      texLee(src, dst,
          SE,
          _borderType,
          _borderValue);
      break;

    case MORPH_GEO_FILL_HOLES4:
      geo_fill_holes(src, dst, 4);
      break;

    case MORPH_GEO_FILL_HOLES8:
      geo_fill_holes(src, dst, 8);
      break;

    case MORPH_GEO_OPEN4:
      geo_open(src, dst,
          SE, 4,
          cv::Point(-1,-1),
          _borderType,
          _borderValue);
      break;

    case MORPH_GEO_OPEN8:
      geo_open(src, dst,
          SE, 8,
          cv::Point(-1,-1),
          _borderType,
          _borderValue);
      break;

    case MORPH_GEO_CLOSE4:
      geo_close(src, dst,
          SE, 4,
          cv::Point(-1,-1),
          _borderType,
          _borderValue);
      break;

    case MORPH_GEO_CLOSE8:
      geo_close(src, dst,
          SE, 8,
          cv::Point(-1,-1),
          _borderType,
          _borderValue);
      break;

    default:
      cv::morphologyEx(src, dst,
          _operation,
          SE,
          _anchor,
          _iterations,
          _borderType,
          _borderValue);
      break;
  }


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

