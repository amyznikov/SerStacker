/*
 * pixtype.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#include "pixtype.h"
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>

template<>
const c_enum_member * members_of<PIXEL_DEPTH>()
{
  static constexpr c_enum_member members[] = {
      { PIXEL_DEPTH_NO_CHANGE, "NO_CHANGE",  },
      { PIXEL_DEPTH_8U, "CV_8U",  },
      { PIXEL_DEPTH_8S , "CV_8S", },
      { PIXEL_DEPTH_16U , "CV_16U", },
      { PIXEL_DEPTH_16S, "CV_16S", },
      { PIXEL_DEPTH_32S , "CV_32S", },
      { PIXEL_DEPTH_32F , "CV_32F", },
      { PIXEL_DEPTH_64F , "CV_64F", },
      { PIXEL_DEPTH_NO_CHANGE , nullptr, },
  };
  return members;
}


bool get_data_range_for_pixel_depth(int ddepth, double * minval, double * maxval)
{
  switch (CV_MAT_DEPTH(ddepth)) {
  case CV_8U :
    *minval = 0;
    *maxval = UINT8_MAX;
    break;
  case CV_8S :
    *minval = INT8_MIN;
    *maxval = INT8_MAX;
    break;
  case CV_16U :
    *minval = 0;
    *maxval = UINT16_MAX;
    break;
  case CV_16S :
    *minval = INT16_MIN;
    *maxval = INT16_MAX;
    break;
  case CV_32S :
    *minval = INT32_MIN;
    *maxval = INT32_MAX;
    break;
  case CV_32F :
    *minval = 0;
    *maxval = 1;
    break;
  case CV_64F :
    *minval = 0;
    *maxval = 1;
    break;
  default:
    *minval = 0;
    *maxval = 1;
    return false;
  }

  return true;
}


double get_maxval_for_pixel_depth(int ddepth)
{
  switch (CV_MAT_DEPTH(ddepth)) {
  case CV_8U :
    return UINT8_MAX;
  case CV_8S :
    return INT8_MAX;
  case CV_16U :
    return UINT16_MAX;
  case CV_16S :
    return INT16_MAX;
  case CV_32S :
    return INT32_MAX;
  }
  return 1;
}

/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool get_scale_offset(int src_depth, int dst_depth, double * scale, double * offset)
{
  double srcmin, srcmax;
  double dstmin, dstmax;

  get_data_range_for_pixel_depth(src_depth, &srcmin, &srcmax);
  get_data_range_for_pixel_depth(dst_depth, &dstmin, &dstmax);

  *scale = (dstmax - dstmin) / (srcmax - srcmin);
  *offset = dstmin - *scale * srcmin;

  return true;
}
