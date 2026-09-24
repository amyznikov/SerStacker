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

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template<class _Tp>
static bool _weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  const cv::Size size = _src_image.size();
  const int cn = _src_image.channels();

  if( _src_accumulator.size() != size ) {
    CF_DEBUG("Image and accumulator sizes not match");
    return false;
  }

  if( _weights_accumulator.size() != size ) {
    CF_DEBUG("Image and weights accumulator sizes not match");
    return false;
  }

  if( _weights_accumulator.type() != CV_32FC1 ) {
    CF_DEBUG("Bad weights_accumulator type. Must be CV_32FC1");
    return false;
  }

  if( _src_accumulator.depth() != CV_32F ) {
    CF_DEBUG("Bad image_accumulator depth = %d. Must be CV_32F", _src_accumulator.depth());
    return false;
  }

  if( _src_accumulator.channels() != cn ) {
    CF_DEBUG("Bad number of image channels=%d. Must be %d", cn, _src_accumulator.channels());
    return false;
  }


  const cv::Mat src = _src_image.getMat();
  const uint8_t * const src_base = (uint8_t*)src.ptr();
  const size_t src_stride = src.step;

  const cv::Mat srcw = _src_weights.getMat();
  const int weights_type = _src_weights.empty() ? -1 : _src_weights.type();
  const uint8_t * const srcw_base = (const uint8_t * )(weights_type < 0 ? nullptr : srcw.data);
  const size_t srcw_stride = weights_type == -1 ? 0 : srcw.step;

  cv::Mat & acc = _src_accumulator.getMatRef();
  uint8_t * const acc_base = (uint8_t * )acc.ptr();
  const size_t acc_stride = acc.step;

  cv::Mat & W = _weights_accumulator.getMatRef();
  uint8_t * const accw_base = (uint8_t * )W.ptr();
  const size_t accw_stride = W.step;

  parallel_for(0, size.height, [=](const auto & range) {

    const int y0 = rbegin(range);

    const uint8_t* srcpy = src_base + y0 * src_stride;
    uint8_t * accpy = acc_base + y0 * acc_stride;
    uint8_t * accwpy = accw_base + y0 * accw_stride;

    for ( int y = y0; y < rend(range); ++y, srcpy += src_stride, accpy += acc_stride, accwpy += accw_stride ) {
      const _Tp* srcp = (const _Tp* )(srcpy);
      float* __restrict accp = (float* )(accpy);
      float* __restrict accwp = (float* )(accwpy);

      if (weights_type < 0) { // no weights
        for (int x = 0; x < size.width; ++x, srcp += cn, accp += cn, ++accwp) {
          const float W_new = *accwp + 1.0f;
          const float factor = 1.0f / W_new;
          *accwp = W_new;
          for (int c = 0; c < cn; ++c) {
            const float I_new = srcp[c];
            const float A_old = accp[c];
            accp[c] = A_old + (I_new - A_old) * factor;
          }
        }
      }
      else if (weights_type == CV_8UC1) { // binary mask is assumed
        const uint8_t* __restrict mp = (const uint8_t*)(srcw_base + y * srcw_stride);
        for (int x = 0; x < size.width; ++x, ++mp, srcp += cn, accp += cn, ++accwp) {
          if (*mp ) {
            const float W_new = *accwp + 1.0f;
            const float factor = 1.0f / W_new;
            *accwp = W_new;
            for (int c = 0; c < cn; ++c) {
              const float I_new = srcp[c];
              const float A_old = accp[c];
              accp[c] = A_old + (I_new - A_old) * factor;
            }
          }
        }
      }
      else if (weights_type == CV_32FC1) { // floating point weights is assumed
        const float* __restrict wp = (const float*)(srcw_base + y * srcw_stride);
        for (int x = 0; x < size.width; ++x, ++wp, srcp += cn, accp += cn, ++accwp) {
          const float w_new = *wp;
          if (w_new > 0) {
            const float W_new = *accwp + w_new;
            const float factor = w_new / W_new;
            *accwp = W_new;
            for (int c = 0; c < cn; ++c) {
              const float I_new = srcp[c];
              const float A_old = accp[c];
              accp[c] = A_old + (I_new - A_old) * factor;
            }
          }
        }
      }
    }});

  return true;
}

static bool weighted_average_update(cv::InputArray _src_image, cv::InputArray _src_weights,
    cv::InputOutputArray _src_accumulator, cv::InputOutputArray _weights_accumulator)
{
  INSTRUMENT_REGION("");
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

  if( _accumulated_frames < 1 ) {
    _accumulator.create(src.size(), CV_MAKETYPE(CV_32F, src.channels()));
    _weights.create(src.size());
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

  if ( _accumulated_frames < 1 ) {
    return false;
  }

  if ( avg.needed() ) {
    if ( ddepth < 0 ) {
      ddepth = avg.fixedType() ? avg.depth() : _accumulator.depth();
    }
    if( ddepth == _accumulator.depth() && std::abs(dscale - 1) <= FLT_EPSILON ) {
      _accumulator.copyTo(avg);
    }
    else {
      _accumulator.convertTo(avg, ddepth, dscale);
    }
  }

  if ( mask.needed() ) {
    cv::compare(_weights, 0, mask, cv::CMP_GT);
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
      cv::Mat1f new_counter = cv::Mat1f::zeros(_weights.size());

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
    const cv::Point & boxpos)
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
      if( _weights.size() != canvasSize ) {
        _weights = cv::Mat1f::zeros(canvasSize);
      }

      _last_bbox = cv::Rect(target_x, target_y, frameSize.width, frameSize.height);
    }
    else {
      cv::Rect ROI = cv::Rect(boxpos.x, boxpos.y, img.cols, img.rows);
      maintainCanvasBoundaries(ROI);
      if ( ROI.width != img.cols || ROI.height != img.rows ) {
        CF_ERROR("APP BUG: Bad ROI size from maintainCanvasBoundaries()");
        return false;
      }

      const cv::Rect CLIPPED_ROI = ROI & cv::Rect(0, 0, _accumulator.cols, _accumulator.rows);
      if( CLIPPED_ROI.empty() ) {
        CF_ERROR("ROI is empty after canvas boundary check");
        return false;
      }

      const cv::Rect CROP(CLIPPED_ROI.x - ROI.x, CLIPPED_ROI.y - ROI.y, CLIPPED_ROI.width, CLIPPED_ROI.height);
      img = img(CROP);
      if ( !weights.empty() ) {
        weights = weights(CROP);
      }
      _last_bbox = CLIPPED_ROI;
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
  cv::Mat1f local_weights;
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
    if ( mask.fixedType() && mask.depth() == CV_32F ) {
      local_weights(bbox).copyTo(mask);
    }
    else {
      cv::compare(local_weights(bbox), 0, mask, cv::CMP_GT);
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


  const cv::Mat image =
      src.getMat();

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

template<class _Tp>
static bool _bayer_drizzle(cv::InputArray bayer_image, cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const int bayer_lookup[2][2],
    cv::InputArray weigths,
    float pixfrac)
{
  const cv::Mat_<_Tp> raw_bayer = bayer_image.getMat();
  const uint8_t * bayer_base = raw_bayer.ptr();
  const size_t bayer_stride = raw_bayer.step;

  const uint8_t * acc_base = acc.ptr();
  const size_t acc_stride = acc.step;

  const uint8_t * cntr_base = cntr.ptr();
  const size_t cntr_stride = cntr.step;

  const int rows = acc.rows;
  const int cols = acc.cols;

  if( rmap.empty() ) {

    if( weigths.empty() ) {
      // CF_DEBUG("NO WEIGHTS");

      parallel_for(0, rows, [=](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          cv::Vec3f * __restrict accp = (cv::Vec3f * )(acc_base + y * acc_stride);
          cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
          const _Tp * bayerp = (const _Tp * )(bayer_base + y * bayer_stride);
          for( int x = 0; x < cols; ++x ) {
            const int cc = bayer_lookup[y & 1][x & 1];
            accp[x][cc] += bayerp[x];
            cntrp[x][cc] += 1;
          }
        }
      });
    }
    else if( weigths.type() == CV_8UC1 ) {

      //  CF_DEBUG("CV_8UC1 WEIGHTS: ACC: %dx%d BAYER: %dx%d", acc.cols, acc.rows, raw_bayer.cols, raw_bayer.rows);

      const cv::Mat1b wmap = weigths.getMat();
      const uint8_t * wmap_base = wmap.ptr();
      const size_t wmap_stride = wmap.step;

      parallel_for(0, rows, [=](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          cv::Vec3f * __restrict accp = (cv::Vec3f * )(acc_base + y * acc_stride);
          cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
          const _Tp * bayerp = (const _Tp * )(bayer_base + y * bayer_stride);
          const uint8_t * wp = (const uint8_t * )(wmap_base + y * wmap_stride);

          for( int x = 0; x < cols; ++x ) {
            if ( wp[x] ) {
              const int cc = bayer_lookup[y & 1][x & 1];
              accp[x][cc] += bayerp[x];
              cntrp[x][cc] += 1;
            }
          }
        }
      });
    }
    else if( weigths.type() == CV_32FC1 ) {

      // CF_DEBUG("CV_32FC1 WEIGHTS");

      const cv::Mat1f wmap = weigths.getMat();
      const uint8_t * wmap_base = wmap.ptr();
      const size_t wmap_stride = wmap.step;

      parallel_for(0, rows, [=](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          cv::Vec3f * __restrict accp = (cv::Vec3f * )(acc_base + y * acc_stride);
          cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
          const _Tp * bayerp = (const _Tp * )(bayer_base + y * bayer_stride);
          const float * wp = (const float * )(wmap_base + y * wmap_stride);
          for( int x = 0; x < cols; ++x ) {
            const int cc = bayer_lookup[y & 1][x & 1];
            accp[x][cc] += bayerp[x] * wp[x];
            cntrp[x][cc] += wp[x];
          }
        }
      });
    }

  }
  else {

    const uint8_t * rmap_base = rmap.ptr();
    const size_t rmap_stride = rmap.step;

    if( weigths.empty() ) {

      // CF_DEBUG("NO WEIGHTS");

      parallel_for(0, rows, [=](const auto & range) {
        constexpr int w = 1;
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          cv::Vec3f * __restrict accp = (cv::Vec3f * )(acc_base + y * acc_stride);
          cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
          const _Tp * bayerp = (const _Tp * )(bayer_base + y * bayer_stride);
          const cv::Vec2f *rmp = (const cv::Vec2f *)(rmap_base + y * rmap_stride);

          for( int x = 0; x < cols; ++x ) {
            const cv::Vec2f & p = rmp[x];
            // Boundaries of the target pixel (x, y) in the source frame
            // Find the sx range where drops guaranteed to intersect the target pixel along the X-axis.
            // Intersection condition: (sx + 0.5 - half_drop < target_x_max) AND (sx + 0.5 + half_drop > target_x_min)
            const float target_x_min = p[0] - 0.5f;
            const float target_x_max = p[0] + 0.5f;
            const float target_y_min = p[1] - 0.5f;
            const float target_y_max = p[1] + 0.5f;
            const float half_drop = pixfrac * 0.5f;
            const int src_x_start = std::max(0, cvCeil (target_x_min - 0.5f - half_drop));
            const int src_x_end   = std::min(cols - 1, cvFloor(target_x_max - 0.5f + half_drop));
            if (src_x_start > src_x_end) { // No intersections along the X-axis.
              continue;
            }
            const int src_y_start = std::max(0, cvCeil (target_y_min - 0.5f - half_drop));
            const int src_y_end   = std::min(rows - 1, cvFloor(target_y_max - 0.5f + half_drop));
            if (src_y_start > src_y_end) { // No intersections along the Y-axis.
              continue;
            }

            // Iterate only over pixels where s > 0
            for (int sy = src_y_start; sy <= src_y_end; ++sy) {
              const _Tp* src_row = (const _Tp* )(bayer_base + sy * bayer_stride);
              const float drop_y_min = (sy + 0.5f) - half_drop;
              const float drop_y_max = (sy + 0.5f) + half_drop;
              const float overlap_y = std::min(target_y_max, drop_y_max) - std::max(target_y_min, drop_y_min);
              const float weight_y = overlap_y * w;
              const int bayer_y = sy & 1;
              for (int sx = src_x_start; sx <= src_x_end; ++sx) {
                const float drop_x_min = (sx + 0.5f) - half_drop;
                const float drop_x_max = (sx + 0.5f) + half_drop;
                const float overlap_x = std::min(target_x_max, drop_x_max) - std::max(target_x_min, drop_x_min);
                const float s = overlap_x * weight_y;
                const int cc = bayer_lookup[bayer_y][sx & 1];
                accp[x][cc] += src_row[sx] * s;
                cntrp[x][cc] += s;
              }
            }
          }
        }
      });
    }
    else if( weigths.type() == CV_8UC1 ) {

      // CF_DEBUG("CV_8UC1 WEIGHTS: ACC: %dx%d BAYER: %dx%d", acc.cols, acc.rows, raw_bayer.cols, raw_bayer.rows);

      const cv::Mat1b wmap = weigths.getMat();
      const uint8_t * wmap_base = wmap.ptr();
      const size_t wmap_stride = wmap.step;

      parallel_for(0, rows, [=](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          cv::Vec3f * __restrict accp = (cv::Vec3f * )(acc_base + y * acc_stride);
          cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
          const uint8_t * wp = (const uint8_t * )(wmap_base + y * wmap_stride);
          const _Tp * bayerp = (const _Tp * )(bayer_base + y * bayer_stride);
          const cv::Vec2f *rmp = (const cv::Vec2f *)(rmap_base + y * rmap_stride);

          for( int x = 0; x < cols; ++x ) {
            if ( wp[x] ) {
              const cv::Vec2f & p = rmp[x];
              // Boundaries of the target pixel (x, y) in the source frame
              // Find the sx range where drops guaranteed to intersect the target pixel along the X-axis.
              // Intersection condition: (sx + 0.5 - half_drop < target_x_max) AND (sx + 0.5 + half_drop > target_x_min)
              const float target_x_min = p[0] - 0.5f;
              const float target_x_max = p[0] + 0.5f;
              const float target_y_min = p[1] - 0.5f;
              const float target_y_max = p[1] + 0.5f;
              const float half_drop = pixfrac * 0.5f;
              const int src_x_start = std::max(0, cvCeil (target_x_min - 0.5f - half_drop));
              const int src_x_end   = std::min(cols - 1, cvFloor(target_x_max - 0.5f + half_drop));
              if (src_x_start > src_x_end) { // No intersections along the X-axis.
                continue;
              }
              const int src_y_start = std::max(0, cvCeil (target_y_min - 0.5f - half_drop));
              const int src_y_end   = std::min(rows - 1, cvFloor(target_y_max - 0.5f + half_drop));
              if (src_y_start > src_y_end) { // No intersections along the Y-axis.
                continue;
              }

              // Iterate only over pixels where s > 0
              for (int sy = src_y_start; sy <= src_y_end; ++sy) {
                const _Tp* src_row = (const _Tp* )(bayer_base + sy * bayer_stride);
                const float drop_y_min = (sy + 0.5f) - half_drop;
                const float drop_y_max = (sy + 0.5f) + half_drop;
                const float overlap_y = std::min(target_y_max, drop_y_max) - std::max(target_y_min, drop_y_min);
                const float weight_y = overlap_y;
                const int bayer_y = sy & 1;
                for (int sx = src_x_start; sx <= src_x_end; ++sx) {
                  const float drop_x_min = (sx + 0.5f) - half_drop;
                  const float drop_x_max = (sx + 0.5f) + half_drop;
                  const float overlap_x = std::min(target_x_max, drop_x_max) - std::max(target_x_min, drop_x_min);
                  const float s = overlap_x * weight_y;
                  const int cc = bayer_lookup[bayer_y][sx & 1];
                  accp[x][cc] += src_row[sx] * s;
                  cntrp[x][cc] += s;
                }
              }
            }
          }
        }
      });
    }
    else if( weigths.type() == CV_32FC1 ) {

      // CF_DEBUG("CV_32FC1 WEIGHTS");

      const cv::Mat1f wmap = weigths.getMat();
      const uint8_t * wmap_base = wmap.ptr();
      const size_t wmap_stride = wmap.step;

      parallel_for(0, rows, [=](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          cv::Vec3f * __restrict accp = (cv::Vec3f * )(acc_base + y * acc_stride);
          cv::Vec3f * __restrict cntrp = (cv::Vec3f *)(cntr_base + y * cntr_stride);
          const float * wp = (const float * )(wmap_base + y * wmap_stride);
          const _Tp * bayerp = (const _Tp * )(bayer_base + y * bayer_stride);
          const cv::Vec2f *rmp = (const cv::Vec2f *)(rmap_base + y * rmap_stride);
          for( int x = 0; x < cols; ++x ) {
            if ( wp[x] ) {
              const cv::Vec2f & p = rmp[x];
              const float target_x_min = p[0] - 0.5f;
              const float target_x_max = p[0] + 0.5f;
              const float target_y_min = p[1] - 0.5f;
              const float target_y_max = p[1] + 0.5f;
              const float half_drop = pixfrac * 0.5f;
              const int src_x_start = std::max(0, cvCeil (target_x_min - 0.5f - half_drop));
              const int src_x_end   = std::min(cols - 1, cvFloor(target_x_max - 0.5f + half_drop));
              if (src_x_start > src_x_end) { // No intersections along the X-axis.
                continue;
              }
              const int src_y_start = std::max(0, cvCeil (target_y_min - 0.5f - half_drop));
              const int src_y_end   = std::min(rows - 1, cvFloor(target_y_max - 0.5f + half_drop));
              if (src_y_start > src_y_end) { // No intersections along the Y-axis.
                continue;
              }

              // Iterate only over pixels where s > 0
              for (int sy = src_y_start; sy <= src_y_end; ++sy) {
                const _Tp* src_row = (const _Tp* )(bayer_base + sy * bayer_stride);
                const float drop_y_min = (sy + 0.5f) - half_drop;
                const float drop_y_max = (sy + 0.5f) + half_drop;
                const float overlap_y = std::min(target_y_max, drop_y_max) - std::max(target_y_min, drop_y_min);
                const float weight_y = overlap_y * wp[x];
                const int bayer_y = sy & 1;
                for (int sx = src_x_start; sx <= src_x_end; ++sx) {
                  const float drop_x_min = (sx + 0.5f) - half_drop;
                  const float drop_x_max = (sx + 0.5f) + half_drop;
                  const float overlap_x = std::min(target_x_max, drop_x_max) - std::max(target_x_min, drop_x_min);
                  const float s = overlap_x * weight_y;
                  const int cc = bayer_lookup[bayer_y][sx & 1];
                  accp[x][cc] += src_row[sx] * s;
                  cntrp[x][cc] += s;
                }
              }
            }
          }
        }
      });
    }
  }

  return true;
}


static bool bayer_drizzle(cv::InputArray bayer_image, cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const int bayer_lookup[2][2],
    cv::InputArray weigths,
    double pixfrac)
{
  CV_DISPATCH(bayer_image.depth(), _bayer_drizzle, bayer_image, acc, cntr, rmap, bayer_lookup, weigths, pixfrac);
  CF_ERROR("APP BUG: BAD bayer_image.depth()=%d encountered", bayer_image.depth());
  return false;
}


void c_bayer_drizzle::set_bayer_pattern(COLORID colorid)
{
  _colorid = colorid;
  if ( !_accumulator.size().empty() ) {
    generate_bayer_lookup_mask();
  }
}

COLORID c_bayer_drizzle::bayer_pattern() const
{
  return _colorid;
}

void c_bayer_drizzle::set_pixfrac(double v)
{
  _pixfrac = v;
}

double c_bayer_drizzle::pixfrac() const
{
  return _pixfrac;
}

void c_bayer_drizzle::set_remap(const cv::Mat2f & rmap)
{
  _rmap = rmap;
}

const cv::Mat2f & c_bayer_drizzle::remap() const
{
  return _rmap ;
}

void c_bayer_drizzle::clear()
{
  _accumulator.release();
  _counter.release();
  _rmap.release();
  _accumulated_frames = 0;
}

bool c_bayer_drizzle::reinitialize(cv::InputArray src, cv::InputArray accw)
{
  return false;
}

bool c_bayer_drizzle::add(cv::InputArray src, cv::InputArray weights)
{
  if( _accumulated_frames < 1 ) {
    const cv::Size image_size = src.size();
    _accumulator.create(image_size);
    _counter.create(image_size);
    _accumulator.setTo(cv::Scalar::all(0));
    _counter.setTo(cv::Scalar::all(0));
    _accumulated_frames = 0;
    generate_bayer_lookup_mask();
  }

  if ( !bayer_drizzle(src, _accumulator, _counter, _rmap, bayer_lookup, weights, _pixfrac) ) {
    CF_ERROR("bayer_drizzle() fails");
    return false;
  }

  ++_accumulated_frames;

  return true;
}

bool c_bayer_drizzle::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if( _accumulated_frames < 1 ) {
    return false;
  }

  if ( avg.needed() ) {
    divideImages(_accumulator, _counter, avg, ddepth, dscale, 1e-5);
  }

  if( mask.needed() ) {
    cv::Mat msk;
    cv::compare(_counter, 0, msk, cv::CMP_GT);
    reduce_color_channels(msk, mask, cv::REDUCE_MAX);
  }

  return true;
}

void c_bayer_drizzle::generate_bayer_lookup_mask()
{
  switch (_colorid) {
    case COLORID_BAYER_RGGB:
      /*
       * R G
       * G B
       * */
      bayer_lookup[0][0] = BAYER_R; bayer_lookup[0][1] = BAYER_G;
      bayer_lookup[1][0] = BAYER_G; bayer_lookup[1][1] = BAYER_B;
      break;

    case COLORID_BAYER_GRBG:
      /*
       * G R
       * B G
       * */
      bayer_lookup[0][0] = BAYER_G; bayer_lookup[0][1] = BAYER_R;
      bayer_lookup[1][0] = BAYER_B; bayer_lookup[1][1] = BAYER_G;
      break;
    case COLORID_BAYER_GBRG:
      /*
       * G B
       * R G
       * */
      bayer_lookup[0][0] = BAYER_G; bayer_lookup[0][1] = BAYER_B;
      bayer_lookup[1][0] = BAYER_R; bayer_lookup[1][1] = BAYER_G;
      break;
    case COLORID_BAYER_BGGR:
      /*
       * B G
       * G R
       * */
      bayer_lookup[0][0] = BAYER_B; bayer_lookup[0][1] = BAYER_G;
      bayer_lookup[1][0] = BAYER_G; bayer_lookup[1][1] = BAYER_R;
      break;
    default:
      break;
  }
}


