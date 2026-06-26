/*
 * pixtype.cc
 *
 *  Created on: Aug 15, 2021
 *      Author: amyznikov
 */

#include "pixtype.h"
#include <stdint.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>

// MINGW hack
#ifndef UINT8_WIDTH
# define UINT8_WIDTH 8
#endif
#ifndef INT8_WIDTH
# define INT8_WIDTH 8
#endif
#ifndef UINT16_WIDTH
# define UINT16_WIDTH 16
#endif
#ifndef INT16_WIDTH
# define INT16_WIDTH 16
#endif
#ifndef UINT32_WIDTH
# define UINT32_WIDTH 32
#endif
#ifndef INT32_WIDTH
# define INT32_WIDTH 32
#endif


template<>
const c_enum_member * members_of<PIXEL_DEPTH>()
{
  static const c_enum_member members[] = {
      { PIXEL_DEPTH_NO_CHANGE, "NO_CHANGE",  },
      { PIXEL_DEPTH_8U, "CV_8U",  },
      { PIXEL_DEPTH_8S , "CV_8S", },
      { PIXEL_DEPTH_16U , "CV_16U", },
      { PIXEL_DEPTH_16S, "CV_16S", },
      { PIXEL_DEPTH_32S , "CV_32S", },
      { PIXEL_DEPTH_32F , "CV_32F", },
      { PIXEL_DEPTH_64F , "CV_64F", },
      { PIXEL_DEPTH_NO_CHANGE, },
  };
  return members;
}


bool getDataRangeForPixelDepth(int ddepth, double * minval, double * maxval)
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


double getMaxValForPixelDepth(int ddepth)
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

int getMaxBppForPixelDepth(int ddepth)
{
  switch (CV_MAT_DEPTH(ddepth)) {
  case CV_8U :
  case CV_8S :
    return UINT8_WIDTH;
  case CV_16U :
  case CV_16S :
    return UINT16_WIDTH;
  case CV_32S :
    return INT32_WIDTH;
  }
  return 0;
}

/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool getScaleOffset(int src_depth, int dst_depth, double * scale, double * offset)
{
  double srcmin, srcmax;
  double dstmin, dstmax;

  getDataRangeForPixelDepth(src_depth, &srcmin, &srcmax);
  getDataRangeForPixelDepth(dst_depth, &dstmin, &dstmax);

  *scale = (dstmax - dstmin) / (srcmax - srcmin);
  *offset = dstmin - *scale * srcmin;

  return true;
}

/**
 *  dst = (src - srcmin) * (dstmax-dstmin) / (srcmax - srcmin) + dstmin;
 *  dst = src * scale  + offset;
 */
bool getScaleOffset(int src_depth, int src_bpp, int dst_depth, double * scale, double * offset)
{
  double srcmin, srcmax;
  double dstmin, dstmax;

  getDataRangeForPixelDepth(src_depth, &srcmin, &srcmax);
  getDataRangeForPixelDepth(dst_depth, &dstmin, &dstmax);

  *scale = (dstmax - dstmin) / (srcmax - srcmin);
  *offset = dstmin - *scale * srcmin;

  if( src_bpp > 0 ) {

    switch (CV_MAT_DEPTH(src_depth)) {
      case CV_8U:
        case CV_8S:
        if( src_bpp < UINT8_WIDTH ) {
          *scale *= (1 << (UINT8_WIDTH - src_bpp));
        }
        break;

      case CV_16U:
        case CV_16S:
        if( src_bpp < UINT16_WIDTH ) {
          *scale *= (1 << (UINT16_WIDTH - src_bpp));
        }
        break;

      case CV_32S:
        if( src_bpp < UINT32_WIDTH ) {
          *scale *= (1 << (UINT32_WIDTH - src_bpp));
        }
        break;
    }
  }

  return true;
}

void convertScaleDepth(cv::InputArray _src, cv::OutputArray dst, int ddepth, bool autoscale, double s)
{
  const cv::Mat src = _src.getMat();
  const int depth = src.depth();

  if ( ddepth < 0 ) {
    ddepth = dst.fixedType() ? dst.depth() : src.depth();
  }

  if ( depth == ddepth ) {
    if ( !autoscale ) {
      src.copyTo(dst);
    }
    else {
      src.convertTo(dst, -1, s);
    }
  }
  else {
    double alpha = 1, beta = 0;
    if ( autoscale ) {
      getScaleOffset(depth, ddepth, &alpha, &beta);
      alpha *= s;
    }
    src.convertTo(dst, ddepth, alpha, beta);
  }
}


template<class _Tp1, class _Tp2>
static void _convertScaleClamp(cv::InputArray _src, cv::OutputArray _dst,
    double imin, double imax, double omin, double omax, int ddepth)
{
  const cv::Mat_<_Tp1> src = _src.getMat();
  const cv::Size size = _src.size();
  const int cn = _src.channels();

  if( _dst.fixedType() ) {
    ddepth = _dst.depth();
  }
  else if( ddepth < 0 ) {
    ddepth = _src.depth();
  }

  cv::Mat dst(size, CV_MAKETYPE(ddepth, cn));

  const double scale = (omax - omin) / (imax - imin);
  const double offset = omin - scale * imin;

  cv::parallel_for_(cv::Range(0, size.height),
      [=, &src, &dst](const auto & range) {
        for ( int y = range.start; y < range.end; ++y ) {
          const _Tp1 * srcp = src[y];
          _Tp2 * __restrict dstp = dst.ptr<_Tp2>(y);
          for ( int x = 0, n = size.width * cn; x < n; ++x) {
            const double v = (*srcp++) * scale + offset;
            *dstp++ = cv::saturate_cast<_Tp2>(std::clamp(v, omin, omax));
          }
        }
      });

  _dst.move(dst);
}

void convertScaleClamp(cv::InputArray src, cv::OutputArray dst,
    double imin, double imax, double omin, double omax, int ddepth)
{
  if (  dst.fixedType() ) {
    ddepth = dst.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, _convertScaleClamp, src, dst,
      imin, imax, omin, omax, ddepth);
}


const char * pixtype2str(int ddepth)
{
  switch(ddepth)
  {
    case CV_8U: return "8U";
    case CV_8S: return "8S";
    case CV_16U: return "16U";
    case CV_16S: return "16S";
    case CV_32S: return "32S";
    case CV_32F: return "32F";
    case CV_64F: return "64F";
    case CV_16F: return "16F";
  }

  return "";
}
