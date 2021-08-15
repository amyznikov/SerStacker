/*
 * pixtype.h
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __pixtype_h__
#define __pixtype_h__

#include <opencv2/opencv.hpp>

enum PIXEL_DEPTH
{
  PIXEL_DEPTH_8U = CV_8U,
  PIXEL_DEPTH_8S = CV_8S,
  PIXEL_DEPTH_16U = CV_16U,
  PIXEL_DEPTH_16S = CV_16S,
  PIXEL_DEPTH_32S = CV_32S,
  PIXEL_DEPTH_32F = CV_32F,
  PIXEL_DEPTH_64F = CV_64F,
  PIXEL_DEPTH_NO_CHANGE = -1,
};

extern const struct PIXEL_DEPTH_desc {
  const char * name;
  enum PIXEL_DEPTH value;
} PIXEL_DEPTHS[];

std::string toStdString(enum PIXEL_DEPTH v);
enum PIXEL_DEPTH fromStdString(const std::string & s,
    enum PIXEL_DEPTH defval);


bool get_data_range_for_pixel_depth(int ddepth,
    double * minv,
    double * maxv);

#endif /* __pixtype_h__ */
