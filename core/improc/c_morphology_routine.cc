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
