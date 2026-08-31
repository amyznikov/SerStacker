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
      else if (weights_type == CV_32FC1) { // floating point weight is assumed
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

cv::Size c_weigthed_average::accumulator_size() const
{
  return _accumulator.size();
}

const cv::Mat & c_weigthed_average::accumulator() const
{
  return _accumulator;
}

const cv::Mat & c_weigthed_average::counter() const
{
  return _weights;
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

bool c_weigthed_average::get_acc_counters(cv::Mat & accw) const
{
  if ( _weights.channels() == 1) {
    _weights.copyTo(accw);
  }
  else {
    cv::cvtColor(_weights, accw, cv::COLOR_BGR2GRAY);
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

cv::Size c_canvas_average::computeCanvasSize(const cv::Size & inputFrameSize)
{
  const int W = inputFrameSize.width;
  const int H = inputFrameSize.height;
  return cv::Size(3 * W / 2, 3 * H / 2);
}


void c_canvas_average::maintainCanvasBoundaries(cv::Rect & bbox, const cv::Size & frameSize)
{
  if (_accumulator.empty() || bbox.width <= 0 || bbox.height <= 0) {
    return;
  }

  const int margin = 32;

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


bool c_canvas_average::add(cv::InputArray current_image, cv::InputArray current_weights_or_mask,
    const cv::Mat2f & rmap, const cv::Rect & new_canvas_bbox)
{
  INSTRUMENT_REGION("");

  if( !rmap.empty() && (rmap.cols > _accumulator.cols || rmap.rows > _accumulator.rows) ) {
    CF_ERROR("Invalid argument: rmap.size()=%dx%d > _accumulator.size()=%dx%d",
        rmap.cols, rmap.rows,
        _accumulator.cols, _accumulator.rows);
    return false;
  }

  cv::Mat img = current_image.getMat();
  cv::Mat weights = current_weights_or_mask.getMat();
  if (img.empty()) {
    CF_ERROR("input image is empty");
    return false;
  }

  cv::Rect ROI;

  if( _accumulator.empty() ) {
    // very first frame
    const cv::Size frameSize = current_image.size();

    const cv::Size computedCanvasSize = computeCanvasSize(frameSize);
    const cv::Size canvasSize(std::max(_canvasSize.width, computedCanvasSize.width),
        std::max(_canvasSize.height, computedCanvasSize.height));

    const int target_x = canvasSize.width / 2 - frameSize.width / 2;
    const int target_y = canvasSize.height / 2 - frameSize.height / 2;
    ROI = cv::Rect(target_x, target_y, frameSize.width, frameSize.height);

    _accumulator = cv::Mat::zeros(canvasSize, current_image.type());
    _weights = cv::Mat1f::zeros(canvasSize);
    weighted_average_update(img, weights, _accumulator(ROI), _weights(ROI));

    _last_bbox = ROI;
    ++_accumulated_frames;
    return true;
  }

  cv::Mat remapped_image, remapped_weights;

  if( new_canvas_bbox.empty() && rmap.empty() ) {
    // no remap requested
    remapped_image = img;
    remapped_weights = weights;
    ROI = cv::Rect(_last_bbox.x, _last_bbox.y, img.cols, img.rows) &
        cv::Rect(0, 0, _accumulator.cols, _accumulator.rows);
  }
  else {
    // remap requested

    ROI = new_canvas_bbox & cv::Rect(0, 0, _accumulator.cols, _accumulator.rows);
    if (ROI.empty()) {
      CF_ERROR("ROI is empty");
      return false;
    }

    maintainCanvasBoundaries(ROI, current_image.size());

    cv::remap(img, remapped_image, rmap, cv::noArray(), opts.interpolation, cv::BORDER_REPLICATE);
    if( !weights.empty() ) {
      const int mask_interp = (weights.type() == CV_8UC1) ? cv::INTER_NEAREST : cv::INTER_LINEAR;
      cv::remap(weights, remapped_weights, rmap, cv::noArray(), mask_interp, cv::BORDER_CONSTANT, cv::Scalar::all(0));
    }
  }

  weighted_average_update(remapped_image, remapped_weights,
      _accumulator(ROI), _weights(ROI));

  _last_bbox = ROI;
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

  if ( _accumulated_frames < 1 ) {
    return false;
  }

  const cv::Rect cbox = cv::Rect(0, 0, _accumulator.cols, _accumulator.rows);
  const cv::Rect bbox = rbbox.empty() ? cbox : (rbbox & cbox);
  if ( bbox.empty() ) {
    return false;
  }

  if ( avg.needed() ) {
    if ( ddepth < 0 ) {
      ddepth = avg.fixedType() ? avg.depth() : _accumulator.depth();
    }
    if( ddepth == _accumulator.depth() && std::abs(dscale - 1) <= FLT_EPSILON ) {
      _accumulator(bbox).copyTo(avg);
    }
    else {
      _accumulator(bbox).convertTo(avg, ddepth, dscale);
    }
  }

  if ( mask.needed() ) {

    if ( mask.fixedType() && mask.depth() == CV_32F ) {
      _weights(bbox).copyTo(mask);
    }
    else {
      cv::compare(_weights(bbox), 0, mask, cv::CMP_GT);
    }
  }

  return true;
}

void c_canvas_average::clear()
{
  _accumulator.release();
  _weights.release();
  _accumulated_frames = 0;
  _last_bbox = cv::Rect();
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

bool c_laplacian_pyramid_focus_stacking::get_acc_counters(cv::Mat & accw) const
{
  accw.release();
  return true;
}


cv::Size c_laplacian_pyramid_focus_stacking::accumulator_size() const
{
  return _image_size;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void c_frame_accumulation_with_fft::clear()
{
  _accumulated_frames = 0;
  _accumulators.clear();
  _weights.clear();
  _rc.x = _rc.y = _rc.width = _rc.height = 0;
  _fftSize.width =  _fftSize.height = 0;
  _border_top = 0;
  _border_bottom = 0;
  _border_left = 0;
  _border_right = 0;
}

bool c_frame_accumulation_with_fft::reinitialize(cv::InputArray src, cv::InputArray accw)
{
  return false;
}

bool c_frame_accumulation_with_fft::add(cv::InputArray src, cv::InputArray _w)
{
  (void)(_w);


  const int nc = src.channels();

  if ( !_accumulators.empty() && _accumulators.size() != nc ) {
    CF_ERROR("Number of channels not match, expected %zu channel input image", _accumulators.size());
    return  false;
  }

  cv::Mat channels[nc];
  cv::Mat weights[nc];

  if ( nc == 1 ) {
    src.getMat().copyTo(channels[0]);
  }
  else {
    cv::split(src.getMat(), channels);
  }

  const cv::Size src_size =
      channels[0].size();

  if ( _accumulators.empty() ) {

    _fftSize = fftGetOptimalSize(src_size, cv::Size(0,0), nullptr, false);

    CF_DEBUG("src_size=%dx%d fftSize_=%dx%d",
        src_size.width, src_size.height,
        _fftSize.width, _fftSize.height);


    if ( src_size == _fftSize ) {
      _rc.x = _rc.y = _rc.width = _rc.height = 0;
    }
    else {
      _border_top = (_fftSize.height - src_size.height) / 2;
      _border_bottom = (_fftSize.height - src_size.height - _border_top);
      _border_left = (_fftSize.width - src_size.width) / 2;
      _border_right = (_fftSize.width - src_size.width - _border_left);
      _rc = cv::Rect(_border_left, _border_top, src.cols(), src.rows());

    }

  }

  if ( nc == 1 ) {
    if ( src_size == _fftSize ) {
      cv::dft(channels[0], channels[0],
          cv::DFT_COMPLEX_OUTPUT);
    }
    else {

      cv::copyMakeBorder(channels[0], channels[0],
          _border_top, _border_bottom,
          _border_left, _border_right,
          cv::BORDER_REFLECT);

      cv::dft(channels[0], channels[0],
          cv::DFT_COMPLEX_OUTPUT);

      fftPower(channels[0], weights[0], true);
    }
  }
  else {

    parallel_loop(0, nc, [this, src_size, &channels, &weights](int i) {
      if ( src_size == _fftSize ) {
        cv::dft(channels[i], channels[i],
            cv::DFT_COMPLEX_OUTPUT);
      }
      else {
        cv::Mat tmp;
        cv::copyMakeBorder(channels[i], tmp,
            _border_top, _border_bottom,
            _border_left, _border_right,
            cv::BORDER_REFLECT);

        channels[i] = tmp;
        cv::dft(channels[i], channels[i],
            cv::DFT_COMPLEX_OUTPUT);

      }

      fftPower(channels[i], weights[i], false);
    });
  }

  if ( _accumulators.empty() ) {

    _accumulators.resize(nc);
    _weights.resize(nc);

    for ( int i = 0; i < nc; ++i ) {

      _accumulators[i].create(channels[i].size(), channels[i].type());
      _accumulators[i].setTo(0);

      _weights[i].create(weights[i].size(), weights[i].type());
      _weights[i].setTo(0);
    }

  }


  for ( int i = 0; i < nc; ++i ) {

    cv::accumulateProduct(channels[i], weights[i], _accumulators[i]);
    cv::accumulate(weights[i], _weights[i]);
  }


  ++_accumulated_frames;

  return true;
}

bool c_frame_accumulation_with_fft::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{

  const int nc = _accumulators.size();
  if ( nc < 1 || _accumulators[0].empty() ) {
    CF_ERROR("c_frame_accumulation_with_fft: accumulator is empty");
    return false;
  }


  cv::Mat channels[nc];

  if ( ddepth < 0 ) {
    ddepth = _accumulators[0].depth();
  }

  for ( int i = 0; i < nc; ++i ) {
    cv::divide(_accumulators[i], _weights[i], channels[i], dscale, ddepth);
    cv::idft(channels[i], channels[i], cv::DFT_REAL_OUTPUT | cv::DFT_SCALE);
  }

  if ( _rc.empty() ) {
    if ( nc == 1 ) {
      avg.move(channels[0]);
    }
    else {
      cv::merge(channels, nc, avg);
    }
  }
  else {
    if ( nc == 1 ) {
      channels[0](_rc).copyTo(avg);
    }
    else {
      cv::Mat tmp;
      cv::merge(channels, nc, tmp);
      tmp(_rc).copyTo(avg);
    }
  }

  if ( mask.needed() ) {
    cv::Mat1b m(avg.size(), 255);
    mask.move(m);
  }


  return true;
}

bool c_frame_accumulation_with_fft::get_acc_counters(cv::Mat & accw) const
{
  accw.release();
  return false;
}

cv::Size c_frame_accumulation_with_fft::accumulator_size() const
{
  return _accumulators.empty() ? cv::Size(0,0) : _accumulators[0].size();
}

const std::vector<cv::Mat> & c_frame_accumulation_with_fft::accumulators() const
{
  return _accumulators;
}

const std::vector<cv::Mat> & c_frame_accumulation_with_fft::weights() const
{
  return _weights;
}

int c_frame_accumulation_with_fft::countNaNs(const cv::Mat & image)
{
  int cnt = 0;

  const int nc = image.channels();

  for ( int y = 0; y < image.rows; ++y ) {

    const float * p = image.ptr<const float>(y);

    for ( int x = 0; x < image.cols * nc; ++x ) {
      if ( std::isnan(p[x]) ) {
        ++cnt;
      }
    }
  }

  return cnt;
}

double c_frame_accumulation_with_fft::power(double x)
{
  return x * x * x;
}

double c_frame_accumulation_with_fft::square(double x)
{
  return x * x;
}

bool c_frame_accumulation_with_fft::fftPower(const cv::Mat & src, cv::Mat & dst, bool mc)
{
  if ( src.channels() != 2 || src.depth() != CV_32F ) {
    CF_ERROR("invalid arg: FP32 2-channel input image expected");
    return false;
  }

  const cv::Mat2f csrc = src;
  cv::Mat2f cmag;

  cmag.create(src.size());

  double scale =
      square(1. / src.size().area());


  if ( !mc ) {
    for ( int y = 0; y < csrc.rows; ++y ) {
      for ( int x = 0; x < csrc.cols; ++x ) {
        const double a = csrc[y][x][0];
        const double b = csrc[y][x][1];
        const double p = power((a * a + b * b) * scale);
        cmag[y][x][0] = cmag[y][x][1] = std::max(scale, p);
      }
    }
  }
  else {
    parallel_loop(0, csrc.rows, [&csrc, &cmag, scale](int y) {
      for ( int x = 0; x < csrc.cols; ++x ) {
        const double a = csrc[y][x][0];
        const double b = csrc[y][x][1];
        const double p = power((a * a + b * b) * scale);
        cmag[y][x][0] = cmag[y][x][1] = std::max(scale, p);
      }
    });
  }

  dst = std::move(cmag);

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


template<class BT>
static bool _bayer_accumulate(cv::InputArray bayer_image, cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const cv::Mat1b & bayer_pattern,
    const cv::Mat & weigths)
{
  const cv::Mat_<BT> src = bayer_image.getMat();

  if( rmap.empty() ) {

    if( weigths.empty() ) {
      parallel_for(0, acc.rows, [&](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          for( int x = 0; x < acc.cols; ++x ) {
            const int cc = bayer_pattern[y][x]; // color channel for update
            acc[y][x][cc] += src[y][x];
            cntr[y][x][cc] += 1;
          }
        }
      });
    }
    else if( weigths.type() == CV_8UC1 ) {

      const cv::Mat1b & w = weigths;

      parallel_for(0, acc.rows, [&](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          for( int x = 0; x < acc.cols; ++x ) {
            if ( w[y][x] ) {
              const int cc = bayer_pattern[y][x];
              acc[y][x][cc] += src[y][x];
              cntr[y][x][cc] += 1;
            }
          }
        }
      });
    }
    else if( weigths.type() == CV_32FC1 ) {

      const cv::Mat1f & w = weigths;

      parallel_for(0, acc.rows, [&](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          for( int x = 0; x < acc.cols; ++x ) {
            const int cc = bayer_pattern[y][x];
            acc[y][x][cc] += src[y][x] * w[y][x];
            cntr[y][x][cc] += w[y][x];
          }
        }
      });
    }

  }
  else {

    static const auto interpolate =
        [](int x, int y, const cv::Vec2f & p, const cv::Mat_<BT> & src, cv::Mat3f & acc, cv::Mat3f & cntr,
            const cv::Mat1b & bayer_pattern, float w) {

              const int src_x = (int)(p[0]);
              const int src_y = (int)(p[1]);

              if( src_x >= 0 && src_x < src.cols - 1 && src_y >= 0 && src_y < src.rows - 1 ) {

                // select color channels and pixel weights for update

                const double ax = (src_x + 1 - p[0]);// occupied x side on [src_x] pixel
                const double ay = (src_y + 1 - p[1]);// occupied y side on [src_y] pixel
                const double bx = (p[0] - src_x);// occupied x side on [src_x+1] pixel
                const double by = (p[1] - src_y);// occupied y side on [src_y+1] pixel


                const double s00 = ax * ay * w;
                const int c00 = bayer_pattern[src_y + 0][src_x + 0];
                acc[y][x][c00] += src[src_y + 0][src_x + 0] * s00;
                cntr[y][x][c00] += s00;


                const double s01 = bx * ay * w;
                const int c01 = bayer_pattern[src_y + 0][src_x + 1];
                acc[y][x][c01] += src[src_y + 0][src_x + 1] * s01;
                cntr[y][x][c01] += s01;


                const double s10 = ax * by * w;
                const int c10 = bayer_pattern[src_y + 1][src_x + 0];
                acc[y][x][c10] += src[src_y + 1][src_x + 0] * s10;
                cntr[y][x][c10] += s10;


                const double s11 = bx * by * w;
                const int c11 = bayer_pattern[src_y + 1][src_x + 1];
                acc[y][x][c11] += src[src_y + 1][src_x + 1] * s11;
                cntr[y][x][c11] += s11;
              }
        };

    if( weigths.empty() ) {
      parallel_for(0, acc.rows, [&](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          const cv::Vec2f *rmp = rmap[y];
          for( int x = 0; x < acc.cols; ++x ) {
            interpolate(x, y, rmp[x], src, acc, cntr, bayer_pattern, 1);
          }
        }
      });
    }
    else if( weigths.type() == CV_8UC1 ) {

      const cv::Mat1b w = weigths;

      parallel_for(0, acc.rows, [&](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          const cv::Vec2f *rmp = rmap[y];
          for( int x = 0; x < acc.cols; ++x ) {
            if ( w[y][x] ) {
              interpolate(x, y, rmp[x], src, acc, cntr, bayer_pattern, 1);
            }
          }
        }
      });
    }
    else if( weigths.type() == CV_32FC1 ) {

      const cv::Mat1f w = weigths;

      parallel_for(0, acc.rows, [&](const auto & range) {
        for ( int y = rbegin(range); y < rend(range); ++y ) {
          const cv::Vec2f *rmp = rmap[y];
          for( int x = 0; x < acc.cols; ++x ) {
            interpolate(x, y, rmp[x], src, acc, cntr, bayer_pattern, w[y][x]);
          }
        }
      });
    }
  }

  return true;
}

static bool bayer_accumulate(cv::InputArray bayer_image, cv::Mat3f & acc, cv::Mat3f & cntr,
    const cv::Mat2f & rmap,
    const cv::Mat1b & bayer_pattern,
    const cv::Mat & weigths)
{
  CV_DISPATCH(bayer_image.depth(), _bayer_accumulate, bayer_image, acc, cntr, rmap, bayer_pattern, weigths);
  CF_ERROR("APP BUG: BAD bayer_image.depth()=%d encountered", bayer_image.depth());
  return false;
}

void c_bayer_average::set_bayer_pattern(COLORID colorid)
{
  _colorid = colorid;
  if ( !_accumulator.size().empty() ) {
    generate_bayer_pattern_mask();
  }
}

COLORID c_bayer_average::bayer_pattern() const
{
  return _colorid;
}

void c_bayer_average::set_remap(const cv::Mat2f & rmap)
{
  _rmap = rmap;
}

const cv::Mat2f & c_bayer_average::remap() const
{
  return _rmap ;
}

void c_bayer_average::clear()
{
  _accumulator.release();
  _counter.release();
  _rmap.release();
  _bayer_pattern.release();
  _accumulated_frames = 0;
}

bool c_bayer_average::reinitialize(cv::InputArray src, cv::InputArray accw)
{
  return false;
}

bool c_bayer_average::add(cv::InputArray src, cv::InputArray weights)
{
  const cv::Mat src_bayer = src.getMat();
  const cv::Mat w = weights.getMat();

  if( _accumulated_frames < 1 ) {

    const cv::Size image_size = src.size();

    _accumulator.create(image_size);
    _counter.create(image_size);

    _accumulator.setTo(0);
    _counter.setTo(0);

    _accumulated_frames = 0;

    generate_bayer_pattern_mask();
  }

  if ( !bayer_accumulate(src, _accumulator, _counter, _rmap, _bayer_pattern, w) ) {
    CF_ERROR("bayer_accumulate() fails");
    return false;
  }

  ++_accumulated_frames;

  return true;
}

bool c_bayer_average::compute(cv::OutputArray avg, cv::OutputArray mask, double dscale, int ddepth) const
{
  if( _accumulated_frames < 1 ) {
    return false;
  }

  cv::Mat3f img(_accumulator.size(), 0.f);

  for( int y = 0; y < img.rows; ++y ) {

    for( int x = 0; x < img.cols; ++x ) {

      for( int c = 0; c < 3; ++c ) {

        if( _counter[y][x][c] > 0 ) {
          img[y][x][c] = _accumulator[y][x][c] / _counter[y][x][c];
        }
        else {
          img[y][x][c] = 0;
        }
      }
    }
  }

  avg.move(img);

  if( mask.needed() ) {
    cv::Mat msk;
    cv::compare(_counter, 0, msk, cv::CMP_GT);
    reduce_color_channels(msk, mask, cv::REDUCE_MAX);
  }

  return true;
}

bool c_bayer_average::get_acc_counters(cv::Mat & accw) const
{
  if( is_bayer_pattern(_colorid) ) { // should be always true
    cv::multiply(_counter, cv::Scalar(1, 0.5, 1), accw);
  }
  else {
    _counter.copyTo(accw);
  }

  return true;
}

cv::Size c_bayer_average::accumulator_size() const
{
  return _accumulator.size();
}

const cv::Mat & c_bayer_average::accumulator() const
{
  return _accumulator;
}

const cv::Mat & c_bayer_average::counter() const
{
  return _counter;
}

void c_bayer_average::generate_bayer_pattern_mask()
{
  _bayer_pattern.create(_accumulator.size());

  switch (_colorid) {
    case COLORID_BAYER_RGGB:
      /*
       * R G
       * G B
       * */
      for ( int y = 0; y < _bayer_pattern.rows / 2; ++y ) {
        for ( int x = 0; x < _bayer_pattern.cols / 2; ++x ) {
          _bayer_pattern[2 * y + 0][2 * x + 0] = BAYER_R;
          _bayer_pattern[2 * y + 0][2 * x + 1] = BAYER_G;
          _bayer_pattern[2 * y + 1][2 * x + 0] = BAYER_G;
          _bayer_pattern[2 * y + 1][2 * x + 1] = BAYER_B;
        }
      }
      break;


    case COLORID_BAYER_GRBG:
      /*
       * G R
       * B G
       * */
      for ( int y = 0; y < _bayer_pattern.rows / 2; ++y ) {
        for ( int x = 0; x < _bayer_pattern.cols / 2; ++x ) {
          _bayer_pattern[2 * y + 0][2 * x + 0] = BAYER_G;
          _bayer_pattern[2 * y + 0][2 * x + 1] = BAYER_R;
          _bayer_pattern[2 * y + 1][2 * x + 0] = BAYER_B;
          _bayer_pattern[2 * y + 1][2 * x + 1] = BAYER_G;
        }
      }
      break;
    case COLORID_BAYER_GBRG:
      /*
       * G B
       * R G
       * */
      for ( int y = 0; y < _bayer_pattern.rows / 2; ++y ) {
        for ( int x = 0; x < _bayer_pattern.cols / 2; ++x ) {
          _bayer_pattern[2 * y + 0][2 * x + 0] = BAYER_G;
          _bayer_pattern[2 * y + 0][2 * x + 1] = BAYER_B;
          _bayer_pattern[2 * y + 1][2 * x + 0] = BAYER_R;
          _bayer_pattern[2 * y + 1][2 * x + 1] = BAYER_G;
        }
      }
      break;
    case COLORID_BAYER_BGGR:
      /*
       * B G
       * G R
       * */
      for ( int y = 0; y < _bayer_pattern.rows / 2; ++y ) {
        for ( int x = 0; x < _bayer_pattern.cols / 2; ++x ) {
          _bayer_pattern[2 * y + 0][2 * x + 0] = BAYER_B;
          _bayer_pattern[2 * y + 0][2 * x + 1] = BAYER_G;
          _bayer_pattern[2 * y + 1][2 * x + 0] = BAYER_G;
          _bayer_pattern[2 * y + 1][2 * x + 1] = BAYER_R;
        }
      }
      break;
    default:
      break;
  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


