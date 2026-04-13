/*
 * c_cubic_hermite_mtf.cc
 *
 *  Created on: Mar 6, 2026
 *      Author: amyznikov
 */

#include "c_smooth_rational_mtf.h"
#include <core/proc/run-loop.h>
#include <core/debug.h>

template<class _Tp>
static bool suggest_levels_range(int depth, _Tp * minv, _Tp * maxv)
{
  switch (depth) {
    case CV_8U:
      *minv = 0;
      *maxv = UINT8_MAX;
      break;
    case CV_8S:
      *minv = INT8_MIN;
      *maxv = INT8_MAX;
      break;
    case CV_16U:
      *minv = 0;
      *maxv = UINT16_MAX;
      break;
    case CV_16S:
      *minv = INT16_MIN;
      *maxv = INT16_MAX;
      break;
    case CV_32S:
      *minv = INT32_MIN;
      *maxv = INT32_MAX;
      break;
    case CV_32F:
      *minv = 0;
      *maxv = 1;
      break;
    case CV_64F:
      *minv = 0;
      *maxv = 1;
      break;
    default:
      *minv = 0;
      *maxv = 1;
      return false;
  }
  return true;
}

bool c_smooth_rational_mtf::suggest_levels_range(int depth, float * minv, float * maxv)
{
  return ::suggest_levels_range(depth, minv, maxv);
}

bool c_smooth_rational_mtf::suggest_levels_range(int depth, double * minv, double * maxv)
{
  return ::suggest_levels_range(depth, minv, maxv);
}

void c_smooth_rational_mtf::update_coeffs()
{
  // shadows, highlights - exponents (usually 0.5 - 3.0)
  // midtones - value at 0.5 (lightness)
  // Calculate k so that f(0.5) = midtones
  // midtones = 0.5^_shadows / (0.5^_shadows + (k * 0.5)^_highlights)

  _s = std::pow(10.f, _opts.shadows);
  _h = std::pow(10.f, _opts.highlights);

  // Solving the equation for k:
  const float half_s = std::pow(0.5f, _s);
  _k = (std::pow((half_s / _opts.midtones) - half_s, 1.0f / _h)) / 0.5f;
  _log_k = std::log(_k);

  // Update also input / output ranges for faster computation
  if ( !(_imax > _imin) ) {
    _xmin = 0, _xmax = 1, _xscale = 1;
  }
  else {
    const float & irange = _imax - _imin;
    _xmin = _opts.lclip >= 0 && _opts.lclip < 1 ? _imin + _opts.lclip * irange : _imin;
    _xmax = _opts.hclip > 0 && _opts.hclip <= 1 ? _imin + _opts.hclip * irange : _imax;
    _xscale = _xmax > _xmin ? 1 / std::abs(_xmax - _xmin) : 1;
  }

  // release also LUT tables
  _lut8.release();
  _lut16.release();
}

// Helper function for checking and creating a 8-bit LUT
void c_smooth_rational_mtf::ensure_lut8() const
{
  if( _lut8.empty() ) {
    _lut8.create(1, 256);
    uint8_t * dstp = _lut8.ptr();
    for( int i = 0; i < 256; ++i ) {
      dstp[i] = cv::saturate_cast<uint8_t>(apply(i));
    }
  }
}

// Helper function for checking and creating a 16-bit LUT
void c_smooth_rational_mtf::ensure_lut16() const
{
  if( _lut16.empty() ) {
    _lut16.create(1, 65536);
    uint16_t * ptr = _lut16.ptr<uint16_t>();
    for( int i = 0; i < 65536; ++i ) {
      ptr[i] = cv::saturate_cast<uint16_t>(apply(i));
    }
  }
}

template<typename Tin, typename Tout>
void c_smooth_rational_mtf::parallel_apply(const cv::Mat & src, cv::Mat & dst) const
{
  if( std::is_same_v<Tin, uint8_t> && std::is_same_v<Tout, uint8_t> ) {
    ensure_lut8();
    cv::LUT(src, _lut8, dst);
    return;
  }

  if( std::is_same_v<Tin, uint16_t> && std::is_same_v<Tout, uint16_t> ) {

    ensure_lut16();

    const uint16_t * lutp = _lut16.ptr<uint16_t>();
    const int width = src.cols * src.channels();

    parallel_for(0, src.rows,
        [&src, &dst, lutp, width](const auto & range) {
          const int beg = rbegin(range), end = rend(range);
          for ( int r = beg; r != end; ++r ) {
            const uint16_t * srcp = src.ptr<uint16_t>(r);
            uint16_t * dstp = dst.ptr<uint16_t>(r);
            for (int c = 0; c < width; ++c) {
              dstp[c] = lutp[srcp[c]];
            }
          }
        });
  }
  else {

    const float xmin = _xmin;
    const float xmax = _xmax;
    const float xs = _xscale;
    const float k = _k;
    //const float log_k = _log_k;
    const float s = _s;// _opts.shadows;
    const float h = _h; // _opts.highlights;
    const float omin = _omin;
    const float omax = _omax;
    const float orange = _omax - _omin;
    const int width = src.cols * src.channels();

    parallel_for(0, src.rows,
        [=, &src, &dst](const auto & range) {
          const int beg = rbegin(range), end = rend(range);
          for ( int r = beg; r != end; ++r ) {
            const Tin * srcp = src.ptr<Tin>(r);
            Tout * dstp = dst.ptr<Tout>(r);
            for (int c = 0; c < width; ++c) {
              const float x = static_cast<float>(srcp[c]);
              dstp[c] = cv::saturate_cast<Tout>( (x <= xmin) ? omin : x >= xmax ? omax :
                  std::fma(eval((x - xmin) * xs, k, s, h), orange, omin));
            }
          }
        });
  }
}

template<typename _Tp>
void c_smooth_rational_mtf::dispatch_output(const cv::Mat & src, cv::Mat & dst) const
{
  switch (dst.depth()) {
    case CV_8U: parallel_apply<_Tp, uint8_t>(src, dst); break;
    case CV_8S: parallel_apply<_Tp, int8_t>(src, dst); break;
    case CV_16U: parallel_apply<_Tp, uint16_t>(src, dst); break;
    case CV_16S: parallel_apply<_Tp, int16_t>(src, dst); break;
    case CV_32S: parallel_apply<_Tp, int32_t>(src, dst); break;
    case CV_32F: parallel_apply<_Tp, float>(src, dst); break;
    case CV_64F: parallel_apply<_Tp, double>(src, dst); break;
    default: break;
  }
}

void c_smooth_rational_mtf::get_mtf_curve(std::vector<float> & cy, size_t n) const
{
  if ( n < 2 ) {
    cy.resize(n, 0);
  }
  else {
    cy.resize(n);
    for( size_t i = 0; i < n; ++i ) {
      cy[i] = eval((float) (i) / (n - 1));
    }
  }
}


bool c_smooth_rational_mtf::apply(cv::InputArray input_image, cv::OutputArray output_image, int ddepth) const
{
  const cv::Mat src = input_image.getMat();
  if( src.empty() ) {
    return false;
  }

  if( ddepth < 0 ) {
    ddepth = output_image.fixedType() ? output_image.depth() : src.depth();
  }

  if( !output_image.empty() ) {
    cv::Mat dst = output_image.getMat();
    if( src.datastart == dst.datastart ) {
      if( ddepth != src.depth() ) {
        return false;
      }
    }
  }

  output_image.create(src.size(), CV_MAKETYPE(ddepth, src.channels()));
  cv::Mat dst = output_image.getMat();

  switch (src.depth()) {
    case CV_8U: dispatch_output<uint8_t>(src, dst); break;
    case CV_8S: dispatch_output<int8_t>(src, dst); break;
    case CV_16U: dispatch_output<uint16_t>(src, dst); break;
    case CV_16S: dispatch_output<int16_t>(src, dst); break;
    case CV_32S: dispatch_output<int32_t>(src, dst); break;
    case CV_32F: dispatch_output<float>(src, dst); break;
    case CV_64F: dispatch_output<double>(src, dst); break;
    default:
      return false;
  }
  return true;
}
