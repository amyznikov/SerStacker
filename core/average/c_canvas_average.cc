/*
 * c_canvas_average.cc
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


cv::Size c_canvas_average::computeCanvasSize(const cv::Size & inputFrameSize)
{
  const int W = inputFrameSize.width;
  const int H = inputFrameSize.height;
  return cv::Size(W + 8 * FRAME_MARGIN, H + 8 * FRAME_MARGIN);
}

void c_canvas_average::clear()
{
  std::scoped_lock lock(_mtx);

  _accumulator.release();
  _weights.release();
  _accumulated_frames = 0;
  _last_bbox = cv::Rect();
}

void c_canvas_average::reset()
{
  std::scoped_lock lock(_mtx);

  _accumulated_frames = 0;
  _last_bbox = cv::Rect();
  if ( !_accumulator.empty() ) {
    _accumulator.setTo(0);
  }
  if ( !_weights.empty() ) {
    _weights.setTo(0);
  }
}

void c_canvas_average::maintainCanvasBoundaries(cv::Rect & bbox)
{
  if (_accumulator.empty() || bbox.width <= 0 || bbox.height <= 0) {
    return;
  }

  const int margin = FRAME_MARGIN;

  int shift_x = 0;
  int shift_y = 0;

  if( bbox.x < margin ) {
    shift_x = 2 * margin;
  }
  else if( (bbox.x + bbox.width) >= _accumulator.cols - margin ) {
    shift_x = -2 * margin;
  }

  if( bbox.y < margin ) {
    shift_y = 2 * margin;
  }
  else if( (bbox.y + bbox.height) >= _accumulator.rows - margin ) {
    shift_y = -2 * margin;
  }

  if ( shift_x || shift_y ) {

    const int copy_w = _accumulator.cols - std::abs(shift_x);
    const int copy_h = _accumulator.rows - std::abs(shift_y);
    if (copy_w > 0 && copy_h > 0)  {

      const int src_x = (shift_x > 0) ? 0 : -shift_x;
      const int src_y = (shift_y > 0) ? 0 : -shift_y;
      const int dst_x = (shift_x > 0) ? shift_x : 0;
      const int dst_y = (shift_y > 0) ? shift_y : 0;
      const cv::Rect src_roi(src_x, src_y, copy_w, copy_h);
      const cv::Rect dst_roi(dst_x, dst_y, copy_w, copy_h);

      cv::Mat new_accum = cv::Mat::zeros(_accumulator.size(), _accumulator.type());
      cv::Mat new_counter = cv::Mat::zeros(_weights.size(), _weights.type());

      _accumulator(src_roi).copyTo(new_accum(dst_roi));
      _weights(src_roi).copyTo(new_counter(dst_roi));
      _accumulator = new_accum;
      _weights = new_counter;
      bbox.x += shift_x;
      bbox.y += shift_y;
    }
  }
}

bool c_canvas_average::add(cv::InputArray remapped_image, cv::InputArray remapped_weights_or_mask,
    const cv::Point & tl /* top-left position in accumulator to insert image */)
{
  INSTRUMENT_REGION("");

  cv::Mat img = remapped_image.getMat();
  cv::Mat weights = remapped_weights_or_mask.getMat();
  if (img.empty()) {
    CF_ERROR("input image is empty");
    return false;
  }

  if ( true ) {
    std::scoped_lock lock(_mtx);

    if( !_accumulated_frames ) {
      const cv::Size frameSize = img.size();
      const cv::Size computedCanvasSize = computeCanvasSize(frameSize);
      const cv::Size canvasSize(std::max(_canvasSize.width, computedCanvasSize.width),
          std::max(_canvasSize.height, computedCanvasSize.height));

      const int target_x = canvasSize.width / 2 - frameSize.width / 2;
      const int target_y = canvasSize.height / 2 - frameSize.height / 2;

      if( _accumulator.size() != canvasSize || _accumulator.type() != img.type() ) {
        _accumulator = cv::Mat::zeros(canvasSize, img.type());
      }

      const int wcn = (!weights.empty() && weights.channels() == img.channels()) ? img.channels() : 1;
      const int wtype = CV_MAKETYPE(CV_32F, wcn);
      if ( _weights.size() != canvasSize || _weights.type() != wtype ) {
        _weights = cv::Mat::zeros(canvasSize, wtype);
      }

      _last_bbox = cv::Rect(target_x, target_y, frameSize.width, frameSize.height);
    }
    else {
      cv::Rect dstRoi(tl.x, tl.y, img.cols, img.rows);
      maintainCanvasBoundaries(dstRoi);

      const cv::Rect dstIntersect = dstRoi & cv::Rect(0, 0, _accumulator.cols, _accumulator.rows);
      if( dstIntersect.empty() ) {
        CF_ERROR("Bad Remapped image ROI: completely outside the canvas boundaries");
        return true; // ignore this error
      }

      const cv::Rect srcRoi(dstIntersect.x - dstRoi.x, dstIntersect.y - dstRoi.y,
          dstIntersect.width, dstIntersect.height);

      img = img(srcRoi);
      if( !weights.empty() ) {
        weights = weights(srcRoi);
      }

      _last_bbox = dstIntersect;
    }
  };

  weighted_average_update(img, weights, _accumulator(_last_bbox), _weights(_last_bbox));
  ++_accumulated_frames;
  return true;
}

/*
 * Return fragment of canvas limited by requested rbbox or full canvas if rbbox is empty
 * */
bool c_canvas_average::compute(cv::OutputArray avg, cv::OutputArray mask,
    double dscale, int ddepth, const cv::Rect & rbbox /*= cv::Rect()*/) const
{
  INSTRUMENT_REGION("");

  cv::Mat local_accumulator;
  cv::Mat local_weights;
  int local_accumulated_frames = 0;

  synchronized([&]() {
    local_accumulator = _accumulator;
    local_weights = _weights;
    local_accumulated_frames = _accumulated_frames;
  });

  if ( local_accumulated_frames < 1 || local_accumulator.empty() ) {
    return false;
  }

  const cv::Rect cbox = cv::Rect(0, 0, local_accumulator.cols, local_accumulator.rows);
  const cv::Rect bbox = rbbox.empty() ? cbox : (rbbox & cbox);
  if ( bbox.empty() ) {
    return false;
  }

  if ( avg.needed() ) {
    if ( ddepth < 0 ) {
      ddepth = avg.fixedType() ? avg.depth() : local_accumulator.depth();
    }
    if( ddepth == local_accumulator.depth() && std::abs(dscale - 1) <= FLT_EPSILON ) {
      local_accumulator(bbox).copyTo(avg);
    }
    else {
      local_accumulator(bbox).convertTo(avg, ddepth, dscale);
    }
  }

  if ( mask.needed() ) {
    if ( local_weights.channels() == 1 || (mask.fixedType() && mask.channels() == local_weights.channels()) ) {
      cv::compare(local_weights(bbox), 0, mask, cv::CMP_GT);
    }
    else {
      cv::Mat w_max;
      reduce_color_channels(local_weights(bbox), w_max, cv::REDUCE_MAX);
      cv::compare(w_max, 0, mask, cv::CMP_GT);
    }
  }

  return true;
}




