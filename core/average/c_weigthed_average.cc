/*
 * c_weigthed_average.cc
 *
 *  Created on: Nov 28, 2026
 *      Author: amyznikov
 */
#include "c_frame_accumulation.h"
#include <core/proc/weighted_average.h>
#include <core/proc/pixtype.h>
#include <core/proc/run-loop.h>
#include <core/proc/reduce_channels.h>
#include <core/ssprintf.h>
#include <core/debug.h>

#ifndef ALWAYS_INLINE
#if defined(_MSC_VER)
#  define ALWAYS_INLINE __forceinline
#elif defined(__GNUC__) || defined(__clang__)
#  define ALWAYS_INLINE inline __attribute__((always_inline))
#else
#  define ALWAYS_INLINE inline
#endif
#endif // ALWAYS_INLINE


c_weigthed_average::c_weigthed_average()
{
}


void c_weigthed_average::clear()
{
  _accumulator.release();
  _weights.release();
  _accumulated_frames = 0;
}

bool c_weigthed_average::reinitialize(cv::InputArray src, cv::InputArray accw)
{
  clear();

  src.getMat().copyTo(_accumulator);
  accw.getMat().copyTo(_weights);
  _accumulated_frames = 1;

  return true;
}

bool c_weigthed_average::add(cv::InputArray src, cv::InputArray weights)
{
  INSTRUMENT_REGION("");

  if ( _accumulated_frames < 1 ) {
    const int wcn = (!weights.empty() && weights.channels() == src.channels()) ? src.channels() : 1;
    _accumulator.create(src.size(), CV_MAKETYPE(CV_32F, src.channels()));
    _weights.create(src.size(), CV_MAKETYPE(CV_32F, wcn));
    _accumulator.setTo(0);
    _weights.setTo(0);
    _accumulated_frames = 0;
  }

  if ( src.size() != _accumulator.size() ) {
    CF_ERROR("ERROR in weigthed_frame_average: current frame (%dx%d) and accumulator (%dx%d) sizes not match",
        src.cols(), src.rows(), _accumulator.cols, _accumulator.rows );
    return false;
  }

  if ( src.channels() != _accumulator.channels() ) {
    CF_ERROR("ERROR in weigthed_frame_average: current frame (%d) and accumulator (%d) channel count not match",
        src.channels(), _accumulator.channels());
    return false;
  }

  if( !weights.empty() && src.size() != weights.size() ) {
    CF_ERROR("ERROR in weigthed_frame_average: image size=%dx%d and weights size = %dx%d not match",
        src.cols(), src.rows(),
        weights.cols(), weights.rows());
    return false;
  }

  if ( !weighted_average_update(src, weights, _accumulator, _weights) ) {
    CF_ERROR("weighted_average_update() fails");
    return false;
  }

  ++_accumulated_frames;

  return true;
}

bool c_weigthed_average::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  INSTRUMENT_REGION("");

  cv::Mat acc, accw;
  int accumulated_frames = 0;

  synchronized([&]() {
    acc = _accumulator;
    accw = _weights;
    accumulated_frames = _accumulated_frames;
  });

  if( accumulated_frames < 1 ) {
    CF_ERROR("no frames");
    return false;
  }

  if( avg.fixedType() ) {
    ddepth = avg.depth();
  }
  else if( ddepth < 0 ) {
    ddepth = CV_32F;
  }

  if( avg.needed() ) {
    if( ddepth < 0 ) {
      ddepth = avg.fixedType() ? avg.depth() : acc.depth();
    }
    if( ddepth == acc.depth() && std::abs(dscale - 1) <= FLT_EPSILON ) {
      acc.copyTo(avg);
    }
    else {
      acc.convertTo(avg, ddepth, dscale);
    }
  }

  if( mask.needed() ) {
    if( accw.channels() == 1 || (mask.fixedType() && mask.channels() == accw.channels()) ) {
      cv::compare(accw, 0, mask, cv::CMP_GT);
    }
    else {
      cv::Mat1f w_max;
      reduce_color_channels(accw, w_max, cv::REDUCE_MAX);
      cv::compare(w_max, 0, mask, cv::CMP_GT);
    }
  }

  return true;
}



