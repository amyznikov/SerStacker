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

bool getDataRangeForPixelDepth(int ddepth,double * minv, double * maxv);
double getMaxValForPixelDepth(int ddepth);
int getMaxBppForPixelDepth(int ddepth);


/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool getScaleOffset(int src_depth, int dst_depth, double * scale, double * offset);
bool getScaleOffset(int src_depth, int src_bpp, int dst_depth, double * scale, double * offset);

void convertScaleDepth(cv::InputArray src, cv::OutputArray dst, int ddepth, bool autoscale, double s = 1);

const char * pixtype2str(int ddepth);


#define CV_DISPATCH(depth, func, ...) \
  switch ((depth)) { \
    case CV_8U:  return func<uint8_t>(__VA_ARGS__); \
    case CV_8S:  return func<int8_t>(__VA_ARGS__); \
    case CV_16U: return func<uint16_t>(__VA_ARGS__);\
    case CV_16S: return func<int16_t>(__VA_ARGS__); \
    case CV_32S: return func<int32_t>(__VA_ARGS__); \
    case CV_32F: return func<float>(__VA_ARGS__); \
    case CV_64F: return func<double>(__VA_ARGS__); \
    default: CF_ERROR("Not supported depth=%d", depth); break; \
  }

#define CV_DISPATCH2(depth1, depth2, func, ...) \
  switch (depth1) { \
    case CV_8U:  \
      switch (depth2) { \
        case CV_8U : return func<uint8_t, uint8_t>(__VA_ARGS__); \
        case CV_8S : return func<uint8_t, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<uint8_t, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<uint8_t, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<uint8_t, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<uint8_t, float>(__VA_ARGS__); \
        case CV_64F: return func<uint8_t, double>(__VA_ARGS__); \
      } \
      break; \
    case CV_8S: \
      switch (depth2) { \
        case CV_8U: return func<int8_t, uint8_t>(__VA_ARGS__); \
        case CV_8S: return func<int8_t, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<int8_t, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<int8_t, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<int8_t, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<int8_t, float>(__VA_ARGS__); \
        case CV_64F: return func<int8_t, double>(__VA_ARGS__); \
      } \
      break; \
    case CV_16U: \
      switch (depth2) { \
        case CV_8U: return func<uint16_t, uint8_t>(__VA_ARGS__); \
        case CV_8S: return func<uint16_t, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<uint16_t, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<uint16_t, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<uint16_t, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<uint16_t, float>(__VA_ARGS__); \
        case CV_64F: return func<uint16_t, double>(__VA_ARGS__); \
      } \
      break;\
    case CV_16S:\
      switch (depth2) { \
        case CV_8U: return func<int16_t, uint8_t>(__VA_ARGS__); \
        case CV_8S: return func<int16_t, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<int16_t, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<int16_t, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<int16_t, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<int16_t, float>(__VA_ARGS__); \
        case CV_64F: return func<int16_t, double>(__VA_ARGS__); \
      } \
      break; \
    case CV_32S: \
      switch (depth2) { \
        case CV_8U: return func<int32_t, uint8_t>(__VA_ARGS__); \
        case CV_8S: return func<int32_t, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<int32_t, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<int32_t, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<int32_t, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<int32_t, float>(__VA_ARGS__); \
        case CV_64F: return func<int32_t, double>(__VA_ARGS__); \
      } \
      break; \
    case CV_32F: \
      switch (depth2) { \
        case CV_8U: return func<float, uint8_t>(__VA_ARGS__); \
        case CV_8S: return func<float, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<float, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<float, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<float, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<float, float>(__VA_ARGS__); \
        case CV_64F: return func<float, double>(__VA_ARGS__); \
      } \
      break; \
    case CV_64F: \
      switch (depth2) { \
        case CV_8U: return func<double, uint8_t>(__VA_ARGS__); \
        case CV_8S: return func<double, int8_t>(__VA_ARGS__); \
        case CV_16U: return func<double, uint16_t>(__VA_ARGS__); \
        case CV_16S: return func<double, int16_t>(__VA_ARGS__); \
        case CV_32S: return func<double, int32_t>(__VA_ARGS__); \
        case CV_32F: return func<double, float>(__VA_ARGS__); \
        case CV_64F: return func<double, double>(__VA_ARGS__); \
      }\
      break;\
  }

#endif /* __pixtype_h__ */
