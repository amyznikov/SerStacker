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
  const int src_channels = _src.channels();

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

  const int ymax = 2 * (dsize.height - 1) + 1 < src_rows ? dsize.height : dsize.height - 1;
  const int xmax = 2 * (dsize.width - 1) + 1 < src_cols ? dsize.width : dsize.width - 1;

  run_loop(0, ymax, [&, src_channels, xmax](int y) {
    const _Tp * srcp = src[2 * y + 1];
    _Tp * dstp = dst[y];
    for ( int x = 0; x < xmax; ++x ) {
      for ( int c = 0; c < src_channels; ++c ) {
        dstp[x * src_channels + c] = srcp[(2 * x + 1) * src_channels + c];
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
  const int src_cols = _src.cols();
  const int src_rows = _src.rows();
  const int src_channels = _src.channels();

  if (_src.empty()) {
    return false;
  }

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

  run_loop(0, ymax, [&, src_channels, xmax](int y) {
    const _Tp * srcp = src[2 * y];
    _Tp * dstp = dst[y];
    for ( int x = 0; x < xmax; ++x ) {
      for ( int c = 0; c < src_channels; ++c ) {
        dstp[x * src_channels + c] = srcp[(2 * x) * src_channels + c];
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
template<class _Tp>
static bool _upject_even(cv::InputArray _src, cv::OutputArray _dst,
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
    dsize = cv::Size(src.cols * 2, src.rows * 2);
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

  run_loop(0, ymax, [&, xmax, src_channels, zmask_requested](int y) {

    const _Tp* srcp = src[y];
    _Tp* dstp = dst[2 * y + 1];

    uint8_t * zmp = zmask_requested ? zmask[2 * y + 1] : nullptr;

    for (int x = 0; x < xmax; ++x) {
      for (int c = 0; c < src_channels; ++c) {
        dstp[(2 * x + 1) * src_channels + c] = srcp[x * src_channels + c];
      }
      if ( zmask_requested ) {
        zmp[2 * x + 1] = uint8_t(255);
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

/*
 * 2x upsampling step by injecting EVEN ZERO-VALUED rows and columns ...
 * If zmask output is requested, it will contain single-channel image of
 * requested zdepth type (CV_8U by default) with non-zero values for pixels copied from src
 * and zeros for empty (injected) pixels
 * */
bool upject_even(cv::InputArray src, cv::OutputArray dst, cv::Size dstSize,
    cv::OutputArray zmask, int zdepth)
{
  if ( !src.empty() ) {
    switch (src.depth()) {
    case CV_8U:  return _upject_even<uint8_t>(src, dst, dstSize, zmask, zdepth);
    case CV_8S:  return _upject_even<int8_t>(src, dst, dstSize, zmask, zdepth);
    case CV_16U: return _upject_even<uint16_t>(src, dst, dstSize, zmask, zdepth);
    case CV_16S: return _upject_even<int16_t>(src, dst, dstSize, zmask, zdepth);
    case CV_32S: return _upject_even<int32_t>(src, dst, dstSize, zmask, zdepth);
    case CV_32F: return _upject_even<float>(src, dst, dstSize, zmask, zdepth);
    case CV_64F: return _upject_even<double>(src, dst, dstSize, zmask, zdepth);
    default:
      CF_ERROR("Not supported image depth %d", src.depth());
      break;
    }
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

  run_loop(0, ymax, [&, xmax, src_channels, zmask_requested](int y) {

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
