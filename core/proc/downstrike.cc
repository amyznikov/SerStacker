/*
 * downstrike.cc
 *
 *  Created on: May 26, 2018
 *      Author: amyznikov
 */
#include "downstrike.h"
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif

template<>
const c_enum_member* members_of<DOWNSTRIKE_MODE>()
{
  static const c_enum_member members[] = {
      { DOWNSTRIKE_EVEN, "DOWNSTRIKE_EVEN", "Reject each EVEN row and column: 0,2,4,6..." },
      { DOWNSTRIKE_UNEVEN, "DOWNSTRIKE_UNEVEN", "Reject each UNEVEN row and column: 1,3,5,7 ..." },
      { DOWNSTRIKE_UNEVEN},
  };

  return members;
}

///////////////////////////////////////////////////////////////////////////////

/*
 * 2x downsampling step by rejecting each EVEN row and column, keep only uneven
 * */
template<class _Tp>
static bool _downstrike_even(cv::InputArray _src, cv::OutputArray _dst, cv::Size dsize)
{
  const int src_cols = _src.cols();
  const int src_rows = _src.rows();
  const int channels = _src.channels();

  const cv::Mat_<_Tp> src = _src.getMat();

  if (dsize.empty()) {
    dsize = cv::Size((src_cols + 1) / 2, (src_rows + 1) / 2);
  }
  else if (std::abs(dsize.width * 2 - src_cols) > 2 || std::abs(dsize.height * 2 - src_rows) > 2) {
    CF_ERROR("Invalid dst_size = %dx%d requested for src.size = %dx%d",
        dsize.width, dsize.height, src_cols, src_rows);
    return false;
  }

  cv::Mat tmp(dsize, _src.type());
  cv::Mat_<_Tp> dst = tmp;

  const int ymax = dsize.height;
  const int xmax = dsize.width;

  parallel_for(0, ymax, [&, channels, xmax, src_rows, src_cols](const auto & range) {
    for ( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
      const int src_y = std::min(2 * y + 1, src_rows - 1);
      const _Tp * srcp = src[src_y] + channels;
      const _Tp * srce = src[src_y] + (src_cols - 1) * channels;
      _Tp * __restrict dstp = dst[y];

      for ( int x = 0; x < xmax; ++x, srcp = std::min(srcp + channels, srce) ) {
        for ( int c = 0; c < channels; ++c ) {
          *dstp++ = *srcp++;
        }
      }
    }
  });

  _dst.move(tmp);
  return true;
}


/*
 * 2x downsampling step by rejecting each EVEN row and column, keep only uneven
 * */
bool downstrike_even(cv::InputArray src, cv::OutputArray dst, cv::Size size)
{
  switch (src.depth()) {
  case CV_8U:
    return _downstrike_even<uint8_t>(src, dst, size);
  case CV_8S:
    return _downstrike_even<int8_t>(src, dst, size);
  case CV_16U:
    return _downstrike_even<uint16_t>(src, dst, size);
  case CV_16S:
    return _downstrike_even<int16_t>(src, dst, size);
  case CV_32S:
    return _downstrike_even<int32_t>(src, dst, size);
  case CV_32F:
    return _downstrike_even<float>(src, dst, size);
  case CV_64F:
    return _downstrike_even<double>(src, dst, size);
  default:
    CF_ERROR("Not supported image depth %d", src.depth());
  break;
  }
  return false;
}



/*
 * 2x downsampling step by rejecting each UNEVEN row and column, keep only even (0, 2, 4...)
 * */
template<class _Tp>
static bool _downstrike_uneven(cv::InputArray _src, cv::OutputArray _dst, cv::Size dsize)
{
  if (_src.empty()) {
    return false;
  }

  const int src_cols = _src.cols();
  const int src_rows = _src.rows();
  const int channels = _src.channels();

  const cv::Mat_<_Tp> src = _src.getMat();

  if (dsize.empty()) {
    dsize = cv::Size((src_cols + 1) / 2, (src_rows + 1) / 2);
  }
  else if (std::abs(dsize.width * 2 - src_cols) > 2 || std::abs(dsize.height * 2 - src_rows) > 2) {
    CF_ERROR("Invalid dst_size = %dx%d requested for src.size = %dx%d",
        dsize.width, dsize.height, src_cols, src_rows);
    return false;
  }

  cv::Mat tmp(dsize, _src.type());
  cv::Mat_<_Tp> dst = tmp;

  const int ymax = dsize.height;
  const int xmax = dsize.width;

  parallel_for(0, ymax, [&, channels, xmax, src_rows, src_cols](const auto & range) {
    for ( int y = rbegin(range), ymax = rend(range); y < ymax; ++y ) {
      const int src_y = std::min(2 * y, src_rows - 1);
      const _Tp * srcp = src[src_y];
      const _Tp * srce = src[src_y] + (src_cols - 1) * channels;
      _Tp * __restrict dstp = dst[y];

      for ( int x = 0; x < xmax; ++x, srcp = std::min(srcp + channels, srce) ) {
        for ( int c = 0; c < channels; ++c ) {
          *dstp++ = *srcp++;
        }
      }
    }
  });


  _dst.move(tmp);

  return true;
}


/*
 * 2x downsampling step by rejecting each UNEVEN row and column, keep only even (0, 2, 4...)
 * */
bool downstrike_uneven(cv::InputArray src, cv::OutputArray dst, cv::Size size)
{
  switch (src.depth()) {
  case CV_8U:  return _downstrike_uneven<uint8_t>(src, dst, size);
  case CV_8S:  return _downstrike_uneven<int8_t>(src, dst, size);
  case CV_16U: return _downstrike_uneven<uint16_t>(src, dst, size);
  case CV_16S: return _downstrike_uneven<int16_t>(src, dst, size);
  case CV_32S: return _downstrike_uneven<int32_t>(src, dst, size);
  case CV_32F: return _downstrike_uneven<float>(src, dst, size);
  case CV_64F: return _downstrike_uneven<double>(src, dst, size);
  default:
    CF_ERROR("Not supported image depth %d", src.depth());
    break;
  }
  return false;
}

/*
 * 2x upsampling step by injecting EVEN ZERO-VALUED rows and columns ...
 * If _zmask output is requested, it will contain single-channel image of
 * requested zdepth type (CV_8U by default) with non-zero values for pixels copied from src
 * and zeros for empty (injected) pixels
 * */
//template<class _Tp>
//static bool _upject_even(cv::InputArray _src, cv::OutputArray _dst,
//    cv::Size dsize, cv::OutputArray _zmask, int zdepth)
//{
//  const int src_cols = _src.cols();
//  const int src_rows = _src.rows();
//  const int src_channels = _src.channels();
//
//  if (_src.empty()) {
//    return false;
//  }
//
//  const cv::Mat_<_Tp> src = _src.getMat();
//
//  if (dsize.empty()) {
//    dsize = cv::Size(src.cols * 2, src.rows * 2);
//  }
//  else if (std::abs(dsize.width - src_cols * 2) > 2 ||  std::abs(dsize.height - src_rows * 2) > 2) {
//    CF_ERROR("Invalid dsize %dx%d for src %dx%d", dsize.width, dsize.height, src_cols, src_rows);
//    return false;
//  }
//
//  cv::Mat tmp = cv::Mat::zeros(dsize, _src.type());
//  cv::Mat_<_Tp> dst = tmp;
//
//  const bool zmask_requested = _zmask.needed();
//  cv::Mat1b zmask = zmask_requested ? cv::Mat1b::zeros(dsize) : cv::Mat1b();
//
//  const int ymax = 2 * (src_rows - 1) + 1 < dsize.height ? src_rows : src_rows - 1;
//  const int xmax = 2 * (src_cols - 1) + 1 < dsize.width ? src_cols : src_cols - 1;
//
//  parallel_loop(0, ymax, [&, xmax, src_channels, zmask_requested](int y) {
//
//    const _Tp* srcp = src[y];
//    _Tp* dstp = dst[2 * y + 1];
//
//    uint8_t * zmp = zmask_requested ? zmask[2 * y + 1] : nullptr;
//
//    for (int x = 0; x < xmax; ++x) {
//      for (int c = 0; c < src_channels; ++c) {
//        dstp[(2 * x + 1) * src_channels + c] = srcp[x * src_channels + c];
//      }
//      if ( zmask_requested ) {
//        zmp[2 * x + 1] = uint8_t(255);
//      }
//    }
//  });
//
//  _dst.move(tmp);
//
//  if (zmask_requested) {
//
//    if (zdepth < 0) {
//      zdepth = _zmask.fixedType()  ? _zmask.depth() : CV_8U;
//    }
//
//    if (zdepth == zmask.depth()) {
//      _zmask.move(zmask);
//    }
//    else {
//      double scale = 1, offset = 0;
//      getScaleOffset(zmask.depth(), zdepth, &scale, &offset);
//      zmask.convertTo(_zmask, zdepth, scale, offset);
//    }
//  }
//
//  return true;
//}

template<class _Tp>
static bool _upject_even(cv::InputArray _src, cv::OutputArray _dst, cv::Size dsize, cv::Mat1f * _zmask)
{
  if (_src.empty()) {
    return false;
  }

  const int src_cols = _src.cols();
  const int src_rows = _src.rows();
  const int src_channels = _src.channels();
  const cv::Mat_<_Tp> src = _src.getMat();

  if (dsize.empty()) {
    dsize = cv::Size(src_cols * 2, src.rows * 2);
  }
  else if (std::abs(dsize.width - src_cols * 2) > 2 || std::abs(dsize.height - src_rows * 2) > 2) {
    CF_ERROR("Invalid dsize %dx%d for src %dx%d", dsize.width, dsize.height, src_cols, src_rows);
    return false;
  }

  cv::Mat tmp(dsize, _src.type());
  cv::Mat_<_Tp> dst = tmp;

  const bool zmask_requested = _zmask != nullptr;
  cv::Mat1f zmask = zmask_requested ? cv::Mat1f(dsize) : cv::Mat1f();

  const int ymax = src_rows;
  const int xmax = src_cols;

  // Sequentially form a block of two rows of dsize: (2*y) and (2*y + 1).
  parallel_loop(0, ymax, [&, xmax, src_channels, zmask_requested, dsize](int y) {

    const int dst_y_even = 2 * y;
    const int dst_y_odd = 2 * y + 1;
    if (dst_y_even >= dsize.height) {
      return;
    }

    // Even row 2*y is completely zero, erase it from memory in one fell swoop.
    _Tp* __restrict dst_row_even = dst[dst_y_even];
    std::memset(dst_row_even, 0, dsize.width * src_channels * sizeof(_Tp));
    if (zmask_requested) {
      std::memset(zmask[dst_y_even], 0, dsize.width *  sizeof(*zmask[dst_y_even]));
    }

    if (dst_y_odd >= dsize.height) {
      return;
    }

    // Odd row 2*y + 1  - contains alternation: Zero, Value, Zero, Value...
    const _Tp* srcp = src[y];
    _Tp* __restrict dst_row_odd = dst[dst_y_odd];
    float * __restrict zmp = zmask_requested ? zmask[dst_y_odd] : nullptr;

    // Sequential writing in pairs (Zero + Pixel).
    const int safe_xmax = std::min(xmax, (dsize.width - 1) / 2);
    for (int x = 0; x < safe_xmax; ++x) {
      const int dst_x_even = 2 * x;
      const int dst_x_odd  = 2 * x + 1;

      // Write the pair of pixels: first even (zero), then odd (signal from src)
      for (int c = 0; c < src_channels; ++c) {
        dst_row_odd[dst_x_even * src_channels + c] = 0;
        dst_row_odd[dst_x_odd * src_channels + c] = srcp[x * src_channels + c];
      }

      if (zmask_requested) {
        zmp[dst_x_even] = 0;
        zmp[dst_x_odd]  = 1;
      }
    }

    if (safe_xmax < xmax) {
      const int x = safe_xmax;
      const int dst_x_even = 2 * x;
      const int dst_x_odd = 2 * x + 1;

      // Even zero if it fits into the frame width
      if (dst_x_even < dsize.width) {
        for (int c = 0; c < src_channels; ++c) {
          dst_row_odd[dst_x_even * src_channels + c] = 0;
        }
        if (zmask_requested) {
          zmp[dst_x_even] = 0;
        }

        // Odd pixel from src, only if it ALSO fits
        if (dst_x_odd < dsize.width) {
          for (int c = 0; c < src_channels; ++c) {
            dst_row_odd[dst_x_odd * src_channels + c] = srcp[x * src_channels + c];
          }
          if (zmask_requested) {
            zmp[dst_x_odd] = 1;
          }
        }
      }
    }

    // If dsize.width is wider than 2 * xmax, zero out the remaining tail of the string
    const int filled_width = 2 * xmax;
    if (filled_width < dsize.width) {
      std::memset(dst_row_odd + filled_width * src_channels, 0, (dsize.width - filled_width) * src_channels * sizeof(_Tp));
      if (zmask_requested) {
        std::memset(zmp + filled_width, 0, (dsize.width - filled_width) * sizeof(*zmp));
      }
    }
  });

  _dst.move(tmp);

  if (zmask_requested) {
    *_zmask = std::move(zmask);
  }

  return true;
}

/*
 * 2x upsampling step by injecting EVEN ZERO-VALUED rows and columns ...
 * If zmask output is requested, it will contain single-channel image of
 * requested zdepth type (CV_8U by default) with non-zero values for pixels copied from src
 * and zeros for empty (injected) pixels
 * */
bool upject_even(cv::InputArray src, cv::OutputArray dst, cv::Size dstSize, cv::Mat1f * zmask)
{
  if( !src.empty() ) {
    CV_DISPATCH(src.depth(), _upject_even, src, dst, dstSize, zmask);
    CF_ERROR("Invalid src.depth()=%d", src.depth());
  }
  return false;
}



/*
 * 2x upsampling step by injecting UNEVEN ZERO-VALUED rows and columns ...
 * Keep only EVEN (0, 2, 4...) positions from src.
 * */
template<class _Tp>
static bool _upject_uneven(cv::InputArray _src, cv::OutputArray _dst,
    cv::Size dsize, cv::OutputArray _zmask, int zdepth)
{
  const int src_cols = _src.cols();
  const int src_rows = _src.rows();
  const int src_channels = _src.channels();

  if (_src.empty()) {
    return false;
  }

  const cv::Mat_<_Tp> src = _src.getMat();

  if (dsize.empty()) {
    dsize = cv::Size(src_cols * 2, src_rows * 2);
  }
  else if (std::abs(dsize.width - src_cols * 2) > 2 ||  std::abs(dsize.height - src_rows * 2) > 2) {
    CF_ERROR("Invalid dsize %dx%d for src %dx%d", dsize.width, dsize.height, src_cols, src_rows);
    return false;
  }

  cv::Mat tmp = cv::Mat::zeros(dsize, _src.type());
  cv::Mat_<_Tp> dst = tmp;

  const bool zmask_requested = _zmask.needed();
  cv::Mat1b zmask = zmask_requested ? cv::Mat1b::zeros(dsize) : cv::Mat1b();

  const int ymax = 2 * (src_rows - 1) + 1 < dsize.height ? src_rows : src_rows - 1;
  const int xmax = 2 * (src_cols - 1) + 1 < dsize.width ? src_cols : src_cols - 1;

  parallel_loop(0, ymax, [&, xmax, src_channels, zmask_requested](int y) {

    const _Tp* srcp = src[y];
    _Tp* dstp = dst[2 * y];

    uint8_t * zmp = zmask_requested ? zmask[2 * y] : nullptr;

    for (int x = 0; x < xmax; ++x) {
      for (int c = 0; c < src_channels; ++c) {
        dstp[2 * x * src_channels + c] = srcp[x * src_channels + c];
      }
      if (zmask_requested) {
        zmp[2 * x] = uint8_t(255);
      }
    }
  });

  _dst.move(tmp);

  if (zmask_requested) {
    if (zdepth < 0) {
      zdepth = _zmask.fixedType()  ? _zmask.depth() : CV_8U;
    }
    if (zdepth == zmask.depth()) {
      _zmask.move(zmask);
    }
    else {
      double scale = 1, offset = 0;
      getScaleOffset(zmask.depth(), zdepth, &scale, &offset);
      zmask.convertTo(_zmask, zdepth, scale, offset);
    }
  }

  return true;
}

bool upject_uneven(cv::InputArray src, cv::OutputArray dst, cv::Size dstSize,
    cv::OutputArray zmask, int zdepth)
{
  if (!src.empty()) {
    switch (src.depth()) {
    case CV_8U:  return _upject_uneven<uint8_t>(src, dst, dstSize, zmask, zdepth);
    case CV_8S:  return _upject_uneven<int8_t>(src, dst, dstSize, zmask, zdepth);
    case CV_16U: return _upject_uneven<uint16_t>(src, dst, dstSize, zmask, zdepth);
    case CV_16S: return _upject_uneven<int16_t>(src, dst, dstSize, zmask, zdepth);
    case CV_32S: return _upject_uneven<int32_t>(src, dst, dstSize, zmask, zdepth);
    case CV_32F: return _upject_uneven<float>(src, dst, dstSize, zmask, zdepth);
    case CV_64F: return _upject_uneven<double>(src, dst, dstSize, zmask, zdepth);
    default:
      CF_ERROR("Not supported image depth %d", src.depth());
      break;
    }
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
