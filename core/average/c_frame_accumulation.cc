/*
 * c_frame_accumulation.cc
 *
 *  Created on: Feb 14, 2021
 *      Author: amyznikov
 */

#include "c_frame_accumulation.h"
#include <core/proc/pixtype.h>
#include <core/proc/fft.h>
#include <core/proc/run-loop.h>
#include <core/proc/reduce_channels.h>
#include <core/proc/laplacian_pyramid.h>
#include <core/proc/inpaint/linear_interpolation_inpaint.h>
#include <core/proc/divide.h>
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class _Tp, class _Tpw, bool _PerChannelWeights>
static bool _weighted_average_core(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  const cv::Size size = _src_image.size();
  const int cn = _src_image.channels();

  constexpr bool has_weights = !std::is_void_v<_Tpw>;
  const int wcn = has_weights ? _src_weights.channels() : 0;

  if constexpr ( has_weights ) {
    if ( wcn != 1 && (!_PerChannelWeights || wcn != cn) ) {
      CF_ERROR("Invalid weights channels=%d for image channels=%d and accw channels=%d",
          wcn, cn, _weights_accumulator.channels());
      return false;
    }
  }

  const cv::Mat src = _src_image.getMat();
  const uint8_t * const src_base = src.ptr();
  const size_t src_stride = src.step;

  const cv::Mat srcw = has_weights ? _src_weights.getMat() : cv::Mat();
  const uint8_t * const srcw_base = has_weights ? srcw.ptr() : nullptr;
  const size_t srcw_stride = has_weights ? srcw.step : 0;

  cv::Mat & acc = _src_accumulator.getMatRef();
  uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;

  cv::Mat & W = _weights_accumulator.getMatRef();
  uint8_t * const accw_base = W.ptr();
  const size_t accw_stride = W.step;

  parallel_for(0, size.height, [=](const auto & range) {
    const int y0 = rbegin(range);
    const int y1 = rend(range);
    for ( int y = y0; y < y1; ++y ) {
      const _Tp * srcp = (const _Tp *)(src_base + y * src_stride);
      float * __restrict accp = (float *)(acc_base + y * acc_stride);
      float * __restrict accwp = (float *)(accw_base + y * accw_stride);
      const _Tpw * wp = has_weights ? (const _Tpw *)(srcw_base + y * srcw_stride) : nullptr;

      for ( int x = 0; x < size.width; ++x, srcp += cn, accp += cn ) {

        if constexpr ( !_PerChannelWeights ) {
          // 1-channel weight accumulator (CV_32FC1)
          float w_new = 1.0f;
          if constexpr ( has_weights ) {
            if constexpr ( std::is_integral_v<_Tpw> ) {
              w_new = (*wp++) ? 1.0f : 0.0f;
            }
            else {
              w_new = *wp++;
            }
          }

          if ( w_new > 0.0f ) {
            const float W_new = *accwp + w_new;
            const float factor = w_new / W_new;
            *accwp = W_new;

            for ( int c = 0; c < cn; ++c ) {
              const float I_new = srcp[c];
              const float A_old = accp[c];
              accp[c] = A_old + (I_new - A_old) * factor;
            }
          }
          ++accwp;
        }
        else {
          // Per-channel weight accumulator (CV_32FC(cn))
          if constexpr ( !has_weights ) {
            for ( int c = 0; c < cn; ++c ) {
              const float W_new = accwp[c] + 1.0f;
              const float factor = 1.0f / W_new;
              accwp[c] = W_new;

              const float I_new = srcp[c];
              const float A_old = accp[c];
              accp[c] = A_old + (I_new - A_old) * factor;
            }
          }
          else if ( wcn == 1 ) {
            // Single-channel weights applied to per-channel accumulator
            float w_new;
            if constexpr ( std::is_integral_v<_Tpw> ) {
              w_new = (*wp++) ? 1.0f : 0.0f;
            }
            else {
              w_new = *wp++;
            }

            if ( w_new > 0.0f ) {
              for ( int c = 0; c < cn; ++c ) {
                const float W_new = accwp[c] + w_new;
                const float factor = w_new / W_new;
                accwp[c] = W_new;

                const float I_new = float(srcp[c]);
                const float A_old = accp[c];
                accp[c] = A_old + (I_new - A_old) * factor;
              }
            }
          }
          else {
            // Multi-channel weights (wcn == cn)
            for ( int c = 0; c < cn; ++c ) {
              float w_new;
              if constexpr ( std::is_integral_v<_Tpw> ) {
                w_new = wp[c] ? 1.0f : 0.0f;
              }
              else {
                w_new = wp[c];
              }

              if ( w_new > 0.0f ) {
                const float W_new = accwp[c] + w_new;
                const float factor = w_new / W_new;
                accwp[c] = W_new;

                const float I_new = srcp[c];
                const float A_old = accp[c];
                accp[c] = A_old + (I_new - A_old) * factor;
              }
            }
            wp += cn;
          }
          accwp += cn;
        }
      }
    }
  });

  return true;
}

template<class _Tp, class _Tpw>
static bool _weighted_average_dispatch_wcn(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  const int accw_cn = _weights_accumulator.channels();
  if ( accw_cn == 1 ) {
    return _weighted_average_core<_Tp, _Tpw, false>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
  }
  if ( accw_cn == _src_image.channels() ) {
    return _weighted_average_core<_Tp, _Tpw, true>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
  }

  CF_ERROR("Invalid weights_accumulator channels=%d (expected 1 or %d)", accw_cn, _src_image.channels());
  return false;
}

template<class _Tp>
static bool _weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  if ( _src_weights.empty() ) {
    return _weighted_average_dispatch_wcn<_Tp, void>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
  }

  switch ( _src_weights.depth() ) {
    case CV_8U:
      return _weighted_average_dispatch_wcn<_Tp, uint8_t>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    case CV_16U:
      return _weighted_average_dispatch_wcn<_Tp, uint16_t>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    case CV_32F:
      return _weighted_average_dispatch_wcn<_Tp, float>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    case CV_64F:
      return _weighted_average_dispatch_wcn<_Tp, double>(_src_image, _src_weights, _src_accumulator, _weights_accumulator);
    default:
      break;
  }

  CF_ERROR("Unsupported _src_weights depth=%d in weighted_average_update", _src_weights.depth());
  return false;
}

static bool weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  INSTRUMENT_REGION("");

  const cv::Size size = _src_image.size();
  const int cn = _src_image.channels();

  if ( _src_accumulator.size() != size || _weights_accumulator.size() != size ) {
    CF_ERROR("Image and accumulator sizes not match");
    return false;
  }

  if ( _src_accumulator.depth() != CV_32F || _weights_accumulator.depth() != CV_32F ) {
    CF_ERROR("Bad accumulator depth: acc=%d, accw=%d (must be CV_32F)",
        _src_accumulator.depth(), _weights_accumulator.depth());
    return false;
  }

  if ( _src_accumulator.channels() != cn ) {
    CF_ERROR("Bad number of image channels=%d. Must be %d", cn, _src_accumulator.channels());
    return false;
  }

  if ( !_src_weights.empty() && _src_weights.size() != size ) {
    CF_ERROR("Image size (%dx%d) and weights size (%dx%d) not match",
        size.width, size.height, _src_weights.cols(), _src_weights.rows());
    return false;
  }

  CV_DISPATCH(_src_image.depth(), _weighted_average_update, _src_image, _src_weights,
      _src_accumulator, _weights_accumulator);

  CF_ERROR("APP BUG: BAD _src_image.depth()=%d encountered", _src_image.depth());
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
const c_enum_member* members_of<c_laplacian_pyramid_focus_stacking::fusing_policy>()
{
  static const c_enum_member members[] = {

      { c_laplacian_pyramid_focus_stacking::select_max_energy, "select_max_energy",
          "select max laplacian energy" },

      { c_laplacian_pyramid_focus_stacking::weighted_average, "weighted_average",
          "average with weighting by laplacian energy" },

      { c_laplacian_pyramid_focus_stacking::select_max_energy },
  };
  return members;
}


c_laplacian_pyramid_focus_stacking::c_laplacian_pyramid_focus_stacking(const options & opts) :
    _opts(opts)
{
}

cv::Mat c_laplacian_pyramid_focus_stacking::duplicate_channels(const cv::Mat & src, int cn)
{
  cv::Mat m;
  cv::merge(std::vector<cv::Mat>(cn, src), m);
  return m;
}

void c_laplacian_pyramid_focus_stacking::clear()
{
  acc.clear();
  G.release();
  _accumulated_frames = 0;
  _image_size =  cv::Size(-1,-1);
}

bool c_laplacian_pyramid_focus_stacking::reinitialize(cv::InputArray src, cv::InputArray accw)
{
  //clear();
  return false;
}


bool c_laplacian_pyramid_focus_stacking::add(cv::InputArray src, cv::InputArray mask)
{
  static const auto graystdev =
      [](const cv::Mat & image) -> double {
        cv::Scalar m, s;
        cv::meanStdDev(image, m, s);
        double sv = s[0];
        for ( int i = 1, cn = image.channels(); i < cn; ++i ) {
          sv += s[i];
        }
        return sv;
      };

  static const auto apply_mask =
      [](std::vector<cv::Mat> & lpyr, cv::InputArray m) {

        if( !m.empty() && m.type() == CV_8UC1 ) {

          std::vector<cv::Mat> mskpyr;

          cv::buildPyramid(m, mskpyr, lpyr.size() - 1);

          for( int i = 0, n = mskpyr.size(); i < n - 1; ++i ) {
            cv::compare(mskpyr[i], 255, mskpyr[i], cv::CMP_LT);
            if( !cv::countNonZero(mskpyr[i]) ) {
              break;
            }
            lpyr[i].setTo(0, mskpyr[i]);
          }
        }
      };

  static const auto compute_energy =
      [](cv::Mat & lap, cv::Mat & w, const cv::Mat & G, bool avgc) {

        if ( !avgc || lap.channels() == 1 ) {
          cv::multiply(lap, lap, w);
        }
        else {
          reduce_color_channels(lap, w, cv::REDUCE_SUM);
          cv::multiply(w, w, w);
        }

        if ( !G.empty() ) {
          cv::sepFilter2D(w, w, -1, G, G, cv::Point(-1, -1), 1e-12);
        }

        if ( lap.channels() == w.channels() ) {
          cv::multiply(lap, w, lap);
        }
        else {
          cv::multiply(lap, duplicate_channels(w, lap.channels()), lap);
        }
      };


  const cv::Mat image = src.getMat();

  if ( _image_size.empty() ) {
    _image_size = image.size();
  }
  else if( image.size() != _image_size ) {

    CF_ERROR("Input image size %dx%d not match: expected %dx%d",
        image.cols, image.rows,
        _image_size.width, _image_size.height);

    return false;
  }

  if( _opts.inpaint_mask_holes ) {
    linear_interpolation_inpaint(image, mask, image);
  }

  if( acc.empty() ) {

    if( G.empty() && (_opts.ksigma > 0 || _opts.kradius > 0) ) {

      G = cv::getGaussianKernel(std::max(0, 2 * _opts.kradius + 1),
          std::max(0., _opts.ksigma),
          CV_32F);
    }

    build_laplacian_pyramid(image, acc, 8);
    apply_mask(acc, mask);

    if( _opts.fusing_policy == weighted_average ) {

      wwp.resize(acc.size() - 1);
      for( int i = 0, n = acc.size(); i < n - 1; ++i ) {
        compute_energy(acc[i], wwp[i], G, _opts.avgchannel);
      }
    }

    ++_accumulated_frames;
    return true;
  }

  if( image.channels() != acc.front().channels() ) {
    CF_ERROR("Number of channels in input image %d not match to accumulator channels %d",
        image.channels(), acc.front().channels());
    return false;
  }


  std::vector<cv::Mat> pyr;
  cv::Mat w[2], ww, m;

  build_laplacian_pyramid(image, pyr, 8);
  apply_mask(pyr, mask);

  const int pyrsize = pyr.size();

  if( pyrsize != acc.size() ) {
    CF_ERROR("UNEXPECTED APP BUG: current pyramid size %zu not match to acc pyramid size %zu",
        pyr.size(), acc.size());
    return false;
  }

  const double sv[2] = {
      graystdev(acc.back()),
      graystdev(pyr.back())
  };

  cv::addWeighted(acc.back(), sv[0] / (sv[0] + sv[1]),
      pyr.back(), sv[1] / (sv[0] + sv[1]),
      0,
      acc.back());

  const int cn =
      image.channels();

  for( int i = 0; i < pyrsize - 1; ++i ) {

    switch (_opts.fusing_policy) {
      case select_max_energy: {

        cv::absdiff(acc[i], 0, w[0]);
        cv::absdiff(pyr[i], 0, w[1]);

        if( !G.empty() ) {
          cv::sepFilter2D(w[0], w[0], -1, G, G, cv::Point(-1, -1));
          cv::sepFilter2D(w[1], w[1], -1, G, G, cv::Point(-1, -1));
        }

        cv::compare(w[1], w[0], m, cv::CMP_GT);
        pyr[i].copyTo(acc[i], m);

        break;
      }

      case weighted_average:
        default: {
        compute_energy(pyr[i], ww, G, _opts.avgchannel);
        cv::add(pyr[i], acc[i], acc[i]);
        cv::add(ww, wwp[i], wwp[i]);
        break;
      }
    }
  }

  ++_accumulated_frames;

  return true;
}

bool c_laplacian_pyramid_focus_stacking::compute(cv::OutputArray avg, cv::OutputArray mask,
    double dscale, int ddepth) const
{
  switch (_opts.fusing_policy) {
    case select_max_energy:
      reconstruct_laplacian_pyramid(avg, acc);
      break;

    case weighted_average:
      default:
      if( !acc.empty() ) {
        std::vector<cv::Mat> lpyr(acc.size());
        cv::Mat w;
        for( int i = 0, n = acc.size(); i < n - 1; ++i ) {
          if( acc[i].channels() == wwp[i].channels() ) {
            cv::divide(acc[i], wwp[i], lpyr[i]);
          }
          else {
            cv::divide(acc[i], duplicate_channels(wwp[i], acc[i].channels()), lpyr[i]);
          }
        }
        lpyr.back() = acc.back();
        reconstruct_laplacian_pyramid(avg, lpyr);
      }
      break;
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

namespace {
  enum class RmapMode {
    Empty,
    Direct, // rmap.size() == acc_size
    Scaled  // rmap.size() == cfa_size (on-fly interpolation)
  };
}

// weights for the Catmull-Rom bicubic spline (a = -0.5) for t in [0, 1]
ALWAYS_INLINE
static void catmull_rom_weights(float t, float w[4])
{
  const float t2 = t * t;
  const float t3 = t * t * t;
  w[0] = -0.5f * t + t2 - 0.5f * t3;
  w[1] = 1.0f - 2.5f * t2 + 1.5f * t3;
  w[2] = 0.5f * t + 2.0f * t2 - 1.5f * t3;
  w[3] = -0.5f * t2 + 0.5f * t3;
}

template<class _Tp, class _Tpw, RmapMode _RmapMode>
static bool _cfa_accumulate_core(cv::InputArray _cfa_image,
    cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const int cfa_lookup[2][2],
    cv::InputArray weights)
{
  const cv::Mat_<_Tp> cfa_image = _cfa_image.getMat();
  const uint8_t * const cfa_base = cfa_image.ptr();
  const size_t cfa_stride = cfa_image.step;

  const uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;

  const uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  constexpr bool has_weights = !std::is_void_v<_Tpw>;
  const cv::Mat wmap = has_weights ? weights.getMat() : cv::Mat();
  const uint8_t * const wmap_base = has_weights ? wmap.ptr() : nullptr;
  const size_t wmap_stride = has_weights ? wmap.step : 0;

  const uint8_t * const rmap_base = (_RmapMode != RmapMode::Empty) ? rmap.ptr() : nullptr;
  const size_t rmap_stride = (_RmapMode != RmapMode::Empty) ? rmap.step : 0;

  const int cfa_rows = cfa_image.rows;
  const int cfa_cols = cfa_image.cols;
  const int acc_rows = acc.rows;
  const int acc_cols = acc.cols;

  const float step_x = float(cfa_cols) / float(acc_cols);
  const float step_y = float(cfa_rows) / float(acc_rows);

  // Ensure 1 pixel of the sensor array covers no more than 2x2 pixels of the CFA
  if ( step_x > 1.0f || step_y > 1.0f ) {
    CF_ERROR("Downscaling is not supported in 2x2 CFA drizzle (step_x=%f, step_y=%f)", step_x, step_y);
    return false;
  }


  parallel_for(0, acc_rows, [=](const auto & range) {

    const int cfa_lut[2][2] = {
        { cfa_lookup[0][0], cfa_lookup[0][1] },
        { cfa_lookup[1][0], cfa_lookup[1][1] }
    };

    // Hardcoded pixfrac = 1 / scale = min(step_x, step_y)
    const float half_drop = 0.5f * std::max(1e-2f, std::min(step_x, step_y));
    const float half_acc_x = 0.5f * step_x;
    const float half_acc_y = 0.5f * step_y;

    const float offset_x = 0.5f * step_x - 0.5f;
    const float offset_y = 0.5f * step_y - 0.5f;

    for ( int y = rbegin(range); y < rend(range); ++y ) {
      cv::Vec3f * __restrict accp = (cv::Vec3f *)(acc_base + y * acc_stride);
      cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
      const float rmap_y = y * step_y + offset_y;

      const cv::Vec2f * rmp0 = nullptr;
      const cv::Vec2f * rmp1 = nullptr;
      const cv::Vec2f * rmp2 = nullptr;
      const cv::Vec2f * rmp3 = nullptr;
      float rwy[4] = {0.0f, 1.0f, 0.0f, 0.0f};

      if constexpr ( _RmapMode == RmapMode::Direct ) {
        // remap.size() == acc.size()
        rmp1 = (const cv::Vec2f *)(rmap_base + y * rmap_stride);
      }
      else if constexpr ( _RmapMode == RmapMode::Scaled ) {
        // remap.size() == cfa_image.size()
        const float clamped_rmap_y = std::clamp(rmap_y, 0.0f, float(cfa_rows - 1));
        const int ry1 = cvFloor(clamped_rmap_y);
        const int ry0 = std::max(ry1 - 1, 0);
        const int ry2 = std::min(ry1 + 1, cfa_rows - 1);
        const int ry3 = std::min(ry1 + 2, cfa_rows - 1);

        catmull_rom_weights(clamped_rmap_y - ry1, rwy);

        rmp0 = (const cv::Vec2f *)(rmap_base + ry0 * rmap_stride);
        rmp1 = (const cv::Vec2f *)(rmap_base + ry1 * rmap_stride);
        rmp2 = (const cv::Vec2f *)(rmap_base + ry2 * rmap_stride);
        rmp3 = (const cv::Vec2f *)(rmap_base + ry3 * rmap_stride);
      }

      for ( int x = 0; x < acc_cols; ++x ) {
        const float rmap_x = x * step_x + offset_x;

        float px, py;
        if constexpr ( _RmapMode == RmapMode::Empty ) {
          // no remap
          px = rmap_x;
          py = rmap_y;
        }
        else if constexpr ( _RmapMode == RmapMode::Direct ) {
          // remap.size() == acc.size()
          const cv::Vec2f p = rmp1[x];
          px = p[0];
          py = p[1];
        }
        else { // RmapMode::Scaled (Bicubic Catmull-Rom)
          // remap.size() == cfa_image.size()
          const float rx = std::clamp(rmap_x, 0.0f, float(cfa_cols - 1));
          const int rx1 = cvFloor(rx);
          const int rx0 = std::max(rx1 - 1, 0);
          const int rx2 = std::min(rx1 + 1, cfa_cols - 1);
          const int rx3 = std::min(rx1 + 2, cfa_cols - 1);

          float rwx[4];
          catmull_rom_weights(rx - rx1, rwx);

          const cv::Vec2f row0 = rmp0[rx0] * rwx[0] + rmp0[rx1] * rwx[1] + rmp0[rx2] * rwx[2] + rmp0[rx3] * rwx[3];
          const cv::Vec2f row1 = rmp1[rx0] * rwx[0] + rmp1[rx1] * rwx[1] + rmp1[rx2] * rwx[2] + rmp1[rx3] * rwx[3];
          const cv::Vec2f row2 = rmp2[rx0] * rwx[0] + rmp2[rx1] * rwx[1] + rmp2[rx2] * rwx[2] + rmp2[rx3] * rwx[3];
          const cv::Vec2f row3 = rmp3[rx0] * rwx[0] + rmp3[rx1] * rwx[1] + rmp3[rx2] * rwx[2] + rmp3[rx3] * rwx[3];
          const cv::Vec2f p = row0 * rwy[0] + row1 * rwy[1] + row2 * rwy[2] + row3 * rwy[3];
          px = p[0];
          py = p[1];
        }

        // Account for an external half-pixel border [-0.5, cols-0.5]
        if ( px < -0.5f || px > cfa_cols - 0.5f || py < -0.5f || py > cfa_rows - 0.5f ) {
          continue;
        }

        float w;
        if constexpr ( !has_weights ) {
          w = 1.0f;
        }
        else {
          const int wy_idx = std::clamp(cvRound(py), 0, cfa_rows - 1);
          const int wx_idx = std::clamp(cvRound(px), 0, cfa_cols - 1);
          const _Tpw ww = *((const _Tpw *)(wmap_base + wy_idx * wmap_stride) + wx_idx);
          if ( !(ww > 0) ) {
            continue;
          }
          if constexpr ( std::is_integral_v<_Tpw> ) {
            w = 1.0f;
          }
          else {
            w = ww;
          }
        }

        // 2x2 neighborhood: sx0 <= px < sx1, sy0 <= py < sy1
        const int sx0 = cvFloor(px), sy0 = cvFloor(py);
        const int sx1 = sx0 + 1, sy1 = sy0 + 1;

        // acc footprint boundaries in CFA coordinates
        const float ax0 = px - half_acc_x;
        const float ax1 = px + half_acc_x;
        const float ay0 = py - half_acc_y;
        const float ay1 = py + half_acc_y;

        // 1D intersections for 2x2 (given that sx0 <= px < sx1)
        const float ov_x0 = std::max(0.0f, std::min(ax1, sx0 + half_drop) - ax0);
        const float ov_x1 = std::max(0.0f, ax1 - std::max(ax0, sx1 - half_drop));

        const float ov_y0 = std::max(0.0f, std::min(ay1, sy0 + half_drop) - ay0);
        const float ov_y1 = std::max(0.0f, ay1 - std::max(ay0, sy1 - half_drop));

        // Boundary validity masks
        const float wx0 = (sx0 >= 0) ? ov_x0 : 0.0f;
        const float wx1 = (sx1 < cfa_cols) ? ov_x1 : 0.0f;
        const float wy0 = (sy0 >= 0) ? (ov_y0 * w) : 0.0f;
        const float wy1 = (sy1 < cfa_rows) ? (ov_y1 * w) : 0.0f;

        // Safe read indices
        const int ix0 = std::max(sx0, 0);
        const int iy0 = std::max(sy0, 0);
        const int ix1 = std::min(sx1, cfa_cols - 1);
        const int iy1 = std::min(sy1, cfa_rows - 1);

        const _Tp * const srcp0 = (const _Tp *)(cfa_base + iy0 * cfa_stride);
        const _Tp * const srcp1 = (const _Tp *)(cfa_base + iy1 * cfa_stride);

        const float v00 = srcp0[ix0], v01 = srcp0[ix1];
        const float v10 = srcp1[ix0], v11 = srcp1[ix1];

        const float w00 = wy0 * wx0, w01 = wy0 * wx1;
        const float w10 = wy1 * wx0, w11 = wy1 * wx1;

        const int c00 = cfa_lut[sy0 & 1][sx0 & 1];
        const int c01 = cfa_lut[sy0 & 1][sx1 & 1];
        const int c10 = cfa_lut[sy1 & 1][sx0 & 1];
        const int c11 = cfa_lut[sy1 & 1][sx1 & 1];

        cv::Vec3f a = accp[x];
        cv::Vec3f cnt = cntrp[x];

        a[c00] += v00 * w00; cnt[c00] += w00;
        a[c01] += v01 * w01; cnt[c01] += w01;
        a[c10] += v10 * w10; cnt[c10] += w10;
        a[c11] += v11 * w11; cnt[c11] += w11;

        accp[x] = a;
        cntrp[x] = cnt;
      }
    }
  });

  return true;
}

template<class _Tp, class _Tpw>
static bool _cfa_accumulate_dispatch_rmap(cv::InputArray cfa_image,
    cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const int bayer_lookup[2][2],
    cv::InputArray weights)
{
  if ( rmap.empty() ) {
    return _cfa_accumulate_core<_Tp, _Tpw, RmapMode::Empty>(cfa_image, acc, cntr, rmap, bayer_lookup, weights);
  }

  if ( rmap.size() == acc.size() ) {
    return _cfa_accumulate_core<_Tp, _Tpw, RmapMode::Direct>(cfa_image, acc, cntr, rmap, bayer_lookup, weights);
  }

  if ( rmap.size() == cfa_image.size() ) {
    return _cfa_accumulate_core<_Tp, _Tpw, RmapMode::Scaled>(cfa_image, acc, cntr, rmap, bayer_lookup, weights);
  }

  CF_ERROR("Invalid remap size (%dx%d) for src size (%dx%d) and acc size (%dx%d)",
      rmap.cols, rmap.rows,
      cfa_image.cols(), cfa_image.rows(),
      acc.cols, acc.rows);
  return false;
}

template<class _Tp>
static bool _cfa_accumulate_dispatch_weights(cv::InputArray cfa_image,
    cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const int bayer_lookup[2][2],
    cv::InputArray weights)
{
  if ( weights.empty() ) {
    return _cfa_accumulate_dispatch_rmap<_Tp, void>(cfa_image, acc, cntr, rmap, bayer_lookup, weights);
  }

  if ( weights.size() != cfa_image.size() ) {
    CF_ERROR("Invalid weights size (%dx%d) for src size (%dx%d)", weights.cols(), weights.rows(),
        cfa_image.cols(), cfa_image.rows());
    return false;
  }

  switch ( weights.type() ) {
    case CV_8UC1:
      return _cfa_accumulate_dispatch_rmap<_Tp, uint8_t>(cfa_image, acc, cntr, rmap, bayer_lookup, weights);
    case CV_32FC1:
      return _cfa_accumulate_dispatch_rmap<_Tp, float>(cfa_image, acc, cntr, rmap, bayer_lookup, weights);
    default:
      break;
  }

  CF_ERROR("Not supported weights type=%d in cfa_accumulate (expected CV_8UC1 or CV_32FC1)", weights.type());
  return false;
}

static bool cfa_accumulate(cv::InputArray cfa_image,
    cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const int bayer_lookup[2][2],
    cv::InputArray weights)
{
  INSTRUMENT_REGION("");

  if ( cfa_image.empty() || cfa_image.channels() != 1 ) {
    CF_ERROR("Expected non-empty single-channel raw bayer image, got channels=%d",
        cfa_image.channels());
    return false;
  }

  if ( acc.empty() || cntr.empty() || acc.size() != cntr.size() ) {
    CF_ERROR("Invalid or mismatched accumulator (%dx%d) and counter (%dx%d) sizes",
        acc.cols, acc.rows, cntr.cols, cntr.rows);
    return false;
  }

  CV_DISPATCH(cfa_image.depth(), _cfa_accumulate_dispatch_weights,
      cfa_image, acc, cntr, rmap, bayer_lookup, weights);

  CF_ERROR("APP BUG: BAD cfa_image.depth()=%d encountered in cfa_accumulate()", cfa_image.depth());
  return false;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<>
const c_enum_member * members_of<c_cfa_drizzle::SCALE>()
{
  static const c_enum_member members[] = {
      { c_cfa_drizzle::SCALE_10x, "DRIZZLE_x1.0", "" },
      { c_cfa_drizzle::SCALE_15x, "DRIZZLE_x1.5", "" },
      { c_cfa_drizzle::SCALE_20x, "DRIZZLE_x2.0", "" },
      { c_cfa_drizzle::SCALE_30x, "DRIZZLE_x3.0", "" },
      { c_cfa_drizzle::SCALE_40x, "DRIZZLE_x4.0", "" },
      { c_cfa_drizzle::SCALE_10x}
  };
  return members;
}

void c_cfa_drizzle::generate_bayer_lookup_mask()
{
  switch (_colorid) {
    case COLORID_BAYER_RGGB:
      /*
       * R G
       * G B
       * */
      _cfa_lookup[0][0] = BAYER_R; _cfa_lookup[0][1] = BAYER_G;
      _cfa_lookup[1][0] = BAYER_G; _cfa_lookup[1][1] = BAYER_B;
      break;

    case COLORID_BAYER_GRBG:
      /*
       * G R
       * B G
       * */
      _cfa_lookup[0][0] = BAYER_G; _cfa_lookup[0][1] = BAYER_R;
      _cfa_lookup[1][0] = BAYER_B; _cfa_lookup[1][1] = BAYER_G;
      break;
    case COLORID_BAYER_GBRG:
      /*
       * G B
       * R G
       * */
      _cfa_lookup[0][0] = BAYER_G; _cfa_lookup[0][1] = BAYER_B;
      _cfa_lookup[1][0] = BAYER_R; _cfa_lookup[1][1] = BAYER_G;
      break;
    case COLORID_BAYER_BGGR:
      /*
       * B G
       * G R
       * */
      _cfa_lookup[0][0] = BAYER_B; _cfa_lookup[0][1] = BAYER_G;
      _cfa_lookup[1][0] = BAYER_G; _cfa_lookup[1][1] = BAYER_R;
      break;
    default:
      break;
  }
}

bool c_cfa_drizzle::add(cv::InputArray src, cv::InputArray weights)
{
  if( src.channels() != 1 ) {
    CF_ERROR("Expected single-channel raw bayer image, got channels=%d", src.channels());
    return false;
  }

  const cv::Size src_size = src.size();

  cv::Size dst_size;

  switch (_scale) {
    case SCALE_10x:
      dst_size = src_size;
      break;
    case SCALE_15x:
      dst_size.width = src_size.width * 3 / 2;
      dst_size.height = src_size.height * 3 / 2;
      break;
    case SCALE_20x:
      dst_size.width = src_size.width * 2;
      dst_size.height = src_size.height * 2;
      break;
    case SCALE_30x:
      dst_size.width = src_size.width * 3;
      dst_size.height = src_size.height * 3;
      break;
    case SCALE_40x:
      dst_size.width = src_size.width * 4;
      dst_size.height = src_size.height * 4;
      break;

    default:
      CF_ERROR("Bad or not supported scale = %d requested", _scale);
      return false;
  }


  if ( true ) {
    std::scoped_lock lock(_mtx);

    if( _accumulated_frames < 1 ) {
      _accumulator.create(dst_size);
      _counter.create(dst_size);
      _accumulator.setTo(cv::Scalar::all(0));
      _counter.setTo(cv::Scalar::all(0));
      _accumulated_frames = 0;
      generate_bayer_lookup_mask();
    }
    else if( _accumulator.size() != dst_size ) {
      CF_ERROR("Expected destination size (%dx%d) does not match accumulator size (%dx%d)",
          dst_size.width, dst_size.height, _accumulator.cols, _accumulator.rows);
      return false;
    }
  }

  if( !cfa_accumulate(src, _accumulator, _counter, _rmap, _cfa_lookup, weights) ) {
    CF_ERROR("cfa_accumulate() fails");
    return false;
  }

  ++_accumulated_frames;
  return true;
}

template<class _Tp, bool _NeedAvg, bool _NeedMask>
static void _compute_cfa_drizzle_output_impl(const cv::Mat3f & acc, const cv::Mat3f & cntr,
    cv::OutputArray avg, cv::OutputArray mask, float scale)
{
  constexpr float eps = 1e-5f;

  const cv::Size size = acc.size();

  cv::Mat dst_avg;
  if constexpr ( _NeedAvg ) {
    avg.create(size, CV_MAKETYPE(cv::DataType<_Tp>::depth, 3));
    dst_avg = avg.getMat();
  }

  cv::Mat1b dst_mask;
  if constexpr ( _NeedMask ) {
    mask.create(size, CV_8UC1);
    dst_mask = mask.getMat();
  }

  const uint8_t * const acc_base = acc.ptr();
  const size_t acc_stride = acc.step;
  const uint8_t * const cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  uint8_t * const avg_base = _NeedAvg ? dst_avg.ptr() : nullptr;
  const size_t avg_stride = _NeedAvg ? dst_avg.step : 0;
  uint8_t * const msk_base = _NeedMask ? dst_mask.ptr() : nullptr;
  const size_t msk_stride = _NeedMask ? dst_mask.step : 0;

  parallel_for(0, size.height, [=](const auto & range) {
    for ( int y = rbegin(range); y < rend(range); ++y ) {
      const cv::Vec3f * const accp = (const cv::Vec3f *)(acc_base + y * acc_stride);
      const cv::Vec3f * const cntrp = (const cv::Vec3f *)(cntr_base + y * cntr_stride);
      cv::Vec<_Tp, 3> * __restrict avgp = _NeedAvg ? (cv::Vec<_Tp, 3> *)(avg_base + y * avg_stride) : nullptr;
      uint8_t * __restrict mskp = _NeedMask ? (msk_base + y * msk_stride) : nullptr;

      for ( int x = 0; x < size.width; ++x ) {
        const cv::Vec3f c = cntrp[x];

        if constexpr ( _NeedMask ) {
          mskp[x] = (c[0] > 0.0f || c[1] > 0.0f || c[2] > 0.0f) ? 255 : 0;
        }

        if constexpr ( _NeedAvg ) {
          const cv::Vec3f a = accp[x];
          avgp[x][0] = cv::saturate_cast<_Tp>(c[0] > eps ? (a[0] * scale / c[0]) : 0.0f);
          avgp[x][1] = cv::saturate_cast<_Tp>(c[1] > eps ? (a[1] * scale / c[1]) : 0.0f);
          avgp[x][2] = cv::saturate_cast<_Tp>(c[2] > eps ? (a[2] * scale / c[2]) : 0.0f);
        }
      }
    }
  });
}

template<class _Tp>
static bool _compute_cfa_drizzle_output(const cv::Mat3f & acc, const cv::Mat3f & cntr,
    cv::OutputArray avg, cv::OutputArray mask, float scale)
{
  const bool need_avg = avg.needed();
  const bool need_mask = mask.needed();

  if ( need_avg && need_mask ) {
    _compute_cfa_drizzle_output_impl<_Tp, true, true>(acc, cntr, avg, mask, scale);
  }
  else if ( need_avg ) {
    _compute_cfa_drizzle_output_impl<_Tp, true, false>(acc, cntr, avg, mask, scale);
  }
  else if ( need_mask ) {
    _compute_cfa_drizzle_output_impl<_Tp, false, true>(acc, cntr, avg, mask, scale);
  }

  return true;
}

bool c_cfa_drizzle::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if ( !avg.needed() && !mask.needed() ) {
    return true;
  }

  cv::Mat3f acc, cntr;
  int accumulated_frames = 0;

  synchronized([&]() {
    acc = _accumulator;
    cntr = _counter;
    accumulated_frames = _accumulated_frames;
  });

  if ( accumulated_frames < 1 ) {
    CF_ERROR("no frames");
    return false;
  }

  if ( avg.fixedType() ) {
    ddepth = avg.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = CV_32F;
  }

  CV_DISPATCH(ddepth, _compute_cfa_drizzle_output, acc, cntr, avg, mask, float(dscale));
  CF_ERROR("APP BUG: BAD ddepth=%d encountered in c_bayer_drizzle::compute", ddepth);
  return false;
}


////////////////////////////////////////////////////////////////////////////////
