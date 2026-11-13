/*
 * c_rational_scurve_mtf.cc
 *
 *  Created on: Nov 13, 2026
 *      Author: amyznikov
 */

#include "c_rational_scurve_mtf.h"
#include <core/proc/run-loop.h>
#include <core/proc/pixtype.h>
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

bool c_rational_scurve_mtf::suggest_levels_range(int depth, float * minv, float * maxv)
{
  return ::suggest_levels_range(depth, minv, maxv);
}

bool c_rational_scurve_mtf::suggest_levels_range(int depth, double * minv, double * maxv)
{
  return ::suggest_levels_range(depth, minv, maxv);
}

void c_rational_scurve_mtf::update_coeffs()
{
  static const auto make_homogeneous_coeffs =
      [](float v, float & c0, float & c1, float & d0, float & d1, float & d2) {
        const float alpha = std::pow(10.0f, v) - 1.0f;
        if ( alpha > 0.0f ) {
          c0 = 0.0f;
          c1 = 1.0f;
          d0 = 0.0f;
          d1 = 1.0f;
          d2 = alpha;
        }
        else {
          const float beta = std::pow(10.0f, -v) - 1.0f;
          c0 = 1.0f + beta;
          c1 = 0.0f;
          d0 = 1.0f;
          d1 = beta;
          d2 = 0.0f;
        }
      };

  make_homogeneous_coeffs(_opts.shadows, _cs0, _cs1, _ds0, _ds1, _ds2);
  make_homogeneous_coeffs(_opts.highlights, _ch0, _ch1, _dh0, _dh1, _dh2);

  const float m = _opts.midtones;
  _k = (1.0f - m) / m;

  if( !(_imax > _imin) ) {
    _xmin = 0.0f;
    _xmax = 1.0f;
    _xscale = 1.0f;
  }
  else {
    const float irange = _imax - _imin;
    _xmin = (_opts.lclip >= 0.0f && _opts.lclip < 1.0f) ? (_imin + _opts.lclip * irange) : _imin;
    _xmax = (_opts.hclip > 0.0f && _opts.hclip <= 1.0f) ? (_imin + _opts.hclip * irange) : _imax;
    _xscale = (_xmax > _xmin) ? (1.0f / (_xmax - _xmin)) : 1.0f;
  }

  _lut8.release();
  _lut16.release();
}



// Helper function for checking and creating a 8-bit LUT
void c_rational_scurve_mtf::ensure_lut8() const
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
void c_rational_scurve_mtf::ensure_lut16() const
{
  if( _lut16.empty() ) {
    _lut16.create(1, 65536);
    uint16_t * ptr = _lut16.ptr<uint16_t>();
    for( int i = 0; i < 65536; ++i ) {
      ptr[i] = cv::saturate_cast<uint16_t>(apply(i));
    }
  }
}

void c_rational_scurve_mtf::get_mtf_curve(std::vector<float> & cy, size_t n) const
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

template<typename _Tp1, typename _Tp2>
bool c_rational_scurve_mtf::parallel_apply(cv::InputArray _src, cv::OutputArray _dst) const
{
  if( std::is_same_v<_Tp1, uint8_t> && std::is_same_v<_Tp2, uint8_t> ) {
    ensure_lut8();
    cv::LUT(_src, _lut8, _dst);
    return true;
  }

  const int cn = _src.channels();
  const int total_width = cn * _src.cols();


  if( std::is_same_v<_Tp1, uint16_t> && std::is_same_v<_Tp2, uint16_t> ) {

    ensure_lut16();

    const cv::Mat src = _src.getMat();
    const uint8_t * src_base = src.ptr();
    const size_t src_stride = src.step;

    _dst.create(_src.size(), CV_MAKETYPE(cv::DataType<_Tp2>::depth, cn));
    cv::Mat dst = _dst.getMat();
    uint8_t * dst_base = dst.ptr();
    const size_t dst_stride = dst.step;

    const uint16_t * lutp = _lut16.ptr<uint16_t>();

    parallel_for(0, src.rows, [=](const auto & range) {
      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const uint16_t * srcp = (const uint16_t * )(src_base + y * src_stride);
        uint16_t * __restrict dstp = (uint16_t * )(dst_base + y * dst_stride);
        for (int x = 0; x < total_width; ++x) {
          dstp[x] = lutp[srcp[x]];
        }
      }
    });
  }
  else {
    INSTRUMENT_REGION("");

    const cv::Mat src = _src.getMat();
    const uint8_t * src_base = src.ptr();
    const size_t src_stride = src.step;

    _dst.create(_src.size(), CV_MAKETYPE(cv::DataType<_Tp2>::depth, cn));
    cv::Mat dst = _dst.getMat();
    uint8_t * dst_base = dst.ptr();
    const size_t dst_stride = dst.step;

    parallel_for(0, src.rows, [=](const auto & range) {
      const float imin = _xmin;
      const float xs = _xscale;
      const float omin = _omin;
      const float orange = _omax - _omin;
      const float cs0 = _cs0, cs1 = _cs1, ds0 = _ds0, ds1 = _ds1, ds2 = _ds2;
      const float ch0 = _ch0, ch1 = _ch1, dh0 = _dh0, dh1 = _dh1, dh2 = _dh2;
      const float k = _k;

      for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
        const _Tp1 * srcp = (const _Tp1 * ) (src_base + y * src_stride);
        _Tp2 * __restrict dstp = (_Tp2 * )(dst_base + y * dst_stride);

        for ( int x = 0; x < total_width; ++x ) {
          const float t = std::min(1.0f, std::max(0.0f, (float(srcp[x]) - imin) * xs));
          dstp[x] = cv::saturate_cast<_Tp2>(std::fma(eval(t,
              cs0, cs1, ds0, ds1, ds2, ch0, ch1, dh0, dh1, dh2, k),
              orange, omin));
        }
      }
    });

  }

  return true;
}


bool c_rational_scurve_mtf::apply(cv::InputArray src, cv::OutputArray dst, int ddepth) const
{
  if ( dst.fixedType() ) {
    if ( dst.channels() != src.channels() ) {
      CF_ERROR("Bad fixed number of output channels %d. Must be same as src.channels=%d",
          dst.channels(), src.channels());
      return false;
    }
    ddepth = dst.depth();
  }
  else if ( ddepth < 0 ) {
    ddepth = src.depth();
  }

  CV_DISPATCH2(src.depth(), ddepth, parallel_apply, src, dst);

  CF_ERROR("Not supported combination of input (%d) and output (%d) data types",
      src.depth(), ddepth);

  return false;
}
