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
    return true;
  }
  return false;
}

bool c_morphology_routine::process(cv::InputOutputArray image, cv::InputOutputArray mask)
{
  if( SE.empty() ) {
    SE =
        cv::getStructuringElement(se_shape_,
            se_size_,
            anchor_);
  }

  switch (operation_) {

    case MORPH_SMOOTH_OPEN:
      morphological_smooth_open(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_SMOOTH_CLOSE:
      morphological_smooth_close(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_INTERNAL_GRADIENT:
      morphological_internal_gradient(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_EXTERNAL_GRADIENT:
      morphological_external_gradient(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_LAPLACIAN:
      morphological_laplacian(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_LAPLACIAN_ABS:
      morphological_laplacian_abs(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_RAMPLEE:
      rampLee(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_TEXLEE:
      texLee(image.getMat(), image,
          SE,
          borderType_,
          borderValue_);
      break;

    case MORPH_GEO_FILL_HOLES4:
      geo_fill_holes(image.getMat(), image, 4);
      break;

    case MORPH_GEO_FILL_HOLES8:
      geo_fill_holes(image.getMat(), image, 8);
      break;

    default:
      cv::morphologyEx(image.getMat(), image,
          operation_,
          SE,
          anchor_,
          iterations_,
          borderType_,
          borderValue_);
      break;
  }

  return true;
}

