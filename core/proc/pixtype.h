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

bool get_data_range_for_pixel_depth(int ddepth,
    double * minv,
    double * maxv);

double get_maxval_for_pixel_depth(int ddepth);

int get_max_bpp_for_pixel_depth(int ddepth);


/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool get_scale_offset(int src_depth, int dst_depth,
    double * scale, double * offset);

bool get_scale_offset(int src_depth, int src_bpp,
    int dst_depth, double * scale, double * offset);

inline void convert_depth(cv::InputArray src, int ddepth, cv::OutputArray dst)
{
  double scale = 1, offset = 0;

  get_scale_offset(src.depth(), ddepth, &scale, &offset);
  src.getMat().convertTo(dst, CV_MAKETYPE(ddepth, src.channels()), scale, offset);
}

#endif /* __pixtype_h__ */
