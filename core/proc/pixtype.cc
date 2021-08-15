/*
 * pixtype.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#include "pixtype.h"

const PIXEL_DEPTH_desc PIXEL_DEPTHS[] = {
    { "NO_CHANGE", PIXEL_DEPTH_NO_CHANGE },
    { "CV_8U", PIXEL_DEPTH_8U },
    { "CV_8S", PIXEL_DEPTH_8S },
    { "CV_16U", PIXEL_DEPTH_16U },
    { "CV_16S", PIXEL_DEPTH_16S },
    { "CV_32S", PIXEL_DEPTH_32S },
    { "CV_32F", PIXEL_DEPTH_32F },
    { "CV_64F", PIXEL_DEPTH_64F },
    { nullptr, PIXEL_DEPTH_NO_CHANGE },
} ;

std::string toStdString(enum PIXEL_DEPTH v)
{
  for ( uint i = 0; PIXEL_DEPTHS[i].name; ++i ) {
    if ( PIXEL_DEPTHS[i].value == v ) {
      return PIXEL_DEPTHS[i].name;
    }
  }
  return "";
}

enum PIXEL_DEPTH fromStdString(const std::string & s,
    enum PIXEL_DEPTH defval)
{
  const char * cstr = s.c_str();

  for ( uint i = 0; PIXEL_DEPTHS[i].name; ++i ) {
    if ( strcasecmp(PIXEL_DEPTHS[i].name, cstr) == 0 ) {
      return PIXEL_DEPTHS[i].value;
    }
  }
  return defval;
}

bool get_data_range_for_pixel_depth(int ddepth, double * minval, double * maxval)
{
  switch ( ddepth ) {
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
