/*
 * bayersym.cc
 *
 *  Created on: Oct 5, 2026
 *      Author: amyznikov
 */

#include "bayersym.h"
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/ssprintf.h>
#include <core/debug.h>

template<typename _Tp>
static bool _bgr2bayer(cv::InputArray _src, cv::OutputArray _dst, COLORID colorid)
{
  const int rows = _src.rows();
  const int cols = _src.cols();
  const int cn = _src.channels();

  if ( cn != 3 ) {
    CF_ERROR("Bad number of input image channels: %d. Must be 3");
    return false;
  }

  if ( (rows & 0x1) || (cols & 0x1) ) {
    CF_ERROR("Not supported uneven image size %dx%d. Must be even", rows, cols);
    return false;
  }


  static constexpr int B = 0, G = 1, R = 2;
  static constexpr int patternRGGB[2][2] = { R, G, G, B };
  static constexpr int patternBGGR[2][2] = { B, G, G, R };
  static constexpr int patternGRBG[2][2] = { G, R, B, G };
  static constexpr int patternGBRG[2][2] = { G, B, R, G };

  const int (*patternMask)[2][2] = nullptr;
  switch(colorid) {
    case COLORID_BAYER_RGGB:
      patternMask = &patternRGGB;
      break;
    case COLORID_BAYER_GRBG:
      patternMask = &patternGRBG;
      break;
    case COLORID_BAYER_GBRG:
      patternMask = &patternGBRG;
      break;
    case COLORID_BAYER_BGGR:
      patternMask = &patternBGGR;
      break;
    default:
      CF_ERROR("Not supported colorid=%d (%s)", (int)(colorid), toCString(colorid));
      return false;
  }

  const cv::Mat src = _src.getMat();
  cv::Mat dst = createOutOfPlace(_src, _dst, rows, cols, CV_MAKETYPE(_src.depth(), 1));

  using _Tpv = cv::Vec<_Tp, 3>;

  parallel_for(0, rows, [&](const auto & range) {
    for( int y = rbegin(range); y < rend(range); ++y ) {
      const _Tpv * srcp = src.ptr<_Tpv>(y);
      _Tp * __restrict dstp = dst.ptr<_Tp>(y);
      const int * mskp = (*patternMask)[y & 1];
      for( int x = 0; x < cols; ++x ) {
        dstp[x] = srcp[x][mskp[x & 1]];
      }
    }
  });

  assignOutOfPlace(_dst, dst);
  return true;
}

bool bgr2bayer(cv::InputArray srcImage, cv::OutputArray outImage, COLORID colorid)
{
  CV_DISPATCH(srcImage.depth(), _bgr2bayer, srcImage, outImage, colorid);
  CF_ERROR("Not supported image depth %d", srcImage.depth());
  return false;
}
