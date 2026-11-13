/*
 * c_rational_scurve_mtf.h
 *
 *  Created on: Nov 13, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_rational_scurve_mtf_h__
#define __c_rational_scurve_mtf_h__

#include <opencv2/opencv.hpp>

/**
  * Evaluates a global C-infinity rational S-curve Midtones Transfer Function (MTF)
  * without transcendental calls (pow/exp/log) or piecewise stitching.
  *
  * Mathematical formulation:
  *   1. Midtones MTF mapping:
  *        u(t) = t / d,   where  r = k * (1 - t),  d = t + r,  k = (1 - m) / m
  *
  *   2. Unbounded rational S-curve warp in display space:
  *        f(t) = P(u) / (P(u) + Q(1 - u))
  *
  *      where:
  *        P(u)     = u * (cs0 + cs1 * u) / (ds0 + ds1 * u + ds2 * u^2)
  *        Q(1 - u) = (1 - u) * (ch0 + ch1 * (1 - u)) / (dh0 + dh1 * (1 - u) + dh2 * (1 - u)^2)
  *
  *   3. Homogeneous algebraic reduction (collapses 4 divisions into 1):
  *        num_s = cs0 * (t * d) + cs1 * t^2
  *        den_s = ds0 * d^2 + ds1 * (t * d) + ds2 * r^2
  *        num_h = ch0 * (r * d) + ch1 * r^2
  *        den_h = dh0 * d^2 + dh1 * (r * d) + dh2 * t^2
  *
  *        eval(t) = (num_s * den_h) / (num_s * den_h + num_h * den_s)
  *
  * @param t   Normalized input intensity in [0, 1] after black/white clipping.
  * @param cs0, cs1, ds0, ds1, ds2  Precomputed shadow rational polynomial coefficients.
  * @param ch0, ch1, dh0, dh1, dh2  Precomputed highlight rational polynomial coefficients.
  * @param k   Midtones balance factor: (1 - midtones) / midtones.
  * @return    Mapped intensity in.
**/

class c_rational_scurve_mtf
{
public:
  typedef c_rational_scurve_mtf this_class;
  typedef std::shared_ptr<this_class> sptr;

  struct Options {
    float lclip = 0.f;
    float hclip = 1.f;
    float shadows = .0f;
    float highlights = .0f;
    float midtones = 0.5f;
  };

  c_rational_scurve_mtf()
  {
    update_coeffs();
  }

  void set_input_range(float minv, float maxv)
  {
    _imin = minv;
    _imax = maxv;
    update_coeffs();
  }

  void get_input_range(float * minv, float * maxv) const
  {
    *minv = _imin;
    *maxv = _imax;
  }

  void get_input_range(double * minv, double * maxv) const
  {
    *minv = _imin;
    *maxv = _imax;
  }

  void set_output_range(float minv, float maxv)
  {
    _omin = minv;
    _omax = maxv;
    update_coeffs();
  }

  void get_output_range(float * minv, float * maxv) const
  {
    *minv = _omin;
    *maxv = _omax;
  }

  void get_output_range(double * minv, double * maxv) const
  {
    *minv = _omin;
    *maxv = _omax;
  }

  void set_lclip(float v)
  {
    _opts.lclip = v;
    update_coeffs();
  }

  float lclip() const
  {
    return _opts.lclip;
  }
  void set_hclip(float v)
  {
    _opts.hclip = v;
    update_coeffs();
  }

  float hclip() const
  {
    return _opts.hclip;
  }

  void set_shadows(float v)
  {
    _opts.shadows = v;
    update_coeffs();
  }

  float shadows() const
  {
    return _opts.shadows;
  }

  void set_highlights(float v)
  {
    _opts.highlights = v;
    update_coeffs();
  }

  float highlights() const
  {
    return _opts.highlights;
  }

  void set_midtones(float v)
  {
    _opts.midtones = std::clamp(v, 0.001f, 0.999f);
    update_coeffs();
  }

  float midtones() const
  {
    return _opts.midtones;
  }

  void set_opts(const Options & opts)
  {
    _opts = opts;
    update_coeffs();
  }

  const Options & opts() const
  {
    return _opts;
  }

  __attribute__((always_inline))
  static inline float eval(float t,
      float cs0, float cs1, float ds0, float ds1, float ds2,
      float ch0, float ch1, float dh0, float dh1, float dh2,
      float k)
  {
    const float r = k * (1.0f - t);
    const float d = t + r;

    const float t2 = t * t;
    const float r2 = r * r;
    const float d2 = d * d;
    const float td = t * d;
    const float rd = r * d;

    const float num_s = std::fma(cs1, t2, cs0 * td);
    const float den_s = std::fma(ds2, r2, std::fma(ds1, td, ds0 * d2));

    const float num_h = std::fma(ch1, r2, ch0 * rd);
    const float den_h = std::fma(dh2, t2, std::fma(dh1, rd, dh0 * d2));

    const float p = num_s * den_h;
    const float q = num_h * den_s;

    return p / (p + q);
  }

  __attribute__((always_inline))
  inline float eval(float t) const
  {
    return eval(t, _cs0, _cs1, _ds0, _ds1, _ds2, _ch0, _ch1, _dh0, _dh1, _dh2, _k);
  }

  __attribute__((always_inline))
  inline float apply(float x) const
  {
    return x <= _xmin ? _omin : x >= _xmax ? _omax :
        std::fma(eval((x - _xmin) * _xscale), _omax - _omin, _omin);
  }

  bool apply(cv::InputArray input_image, cv::OutputArray output_image, int ddepth = -1) const;
  void get_mtf_curve(std::vector<float> & cy, size_t n) const;
  static bool suggest_levels_range(int depth, float * minv, float * maxv);
  static bool suggest_levels_range(int depth, double * minv, double * maxv);

protected:
  void update_coeffs();
  void ensure_lut8() const;
  void ensure_lut16() const;
  template<typename _Tp1, typename _Tp2>
  bool parallel_apply(cv::InputArray _src, cv::OutputArray _dst) const;

protected:
  Options _opts;
  mutable cv::Mat1b _lut8;
  mutable cv::Mat1w _lut16;
  float _imin = 0.f;
  float _imax = 1.f;
  float _omin = 0.f;
  float _omax = 1.f;
  float _xmin = 0.f;
  float _xmax = 1.f;
  float _xscale = 1.f;
  float _cs0 = 1.f, _cs1 = 0.f, _ds0 = 1.f, _ds1 = 0.f, _ds2 = 0.f;
  float _ch0 = 1.f, _ch1 = 0.f, _dh0 = 1.f, _dh1 = 0.f, _dh2 = 0.f;
  float _k = 1.0f;
};

#endif /* __c_rational_scurve_mtf_h__ */
