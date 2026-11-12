/*
 * c_smooth_rational_mtf.h
 *
 *  Created on: Mar 6, 2026
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_smooth_rational_mtf_h__
#define __c_smooth_rational_mtf_h__

#include <opencv2/opencv.hpp>

class c_smooth_rational_mtf
{
public:
  typedef c_smooth_rational_mtf this_class;
  typedef std::shared_ptr<this_class> sptr;

  struct Options {
    float lclip = 0.f;
    float hclip = 1.f;
    float shadows = .0f;
    float highlights = .0f;
    float midtones = 0.5f;
  };

  c_smooth_rational_mtf()
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
    _opts.midtones = std::clamp(v, 0.01f, 0.99f);
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

#if 0
  // Primary rational formula, k-midtones, s-shadows, h-highlighs
  // k, s, h are computed from user inputs inside of update_coeffs()
  __attribute__((always_inline))
  static inline float eval(float t, float k, float s, float h)
  {
    const float ts = std::pow(t, s);
    const float k1t_h = std::pow(k * (1.0f - t), h);
    return ts / (ts + k1t_h + 1e-9f);
  }

#else

  __attribute__((always_inline))
  static inline float fast_log2(float x)
  {
    // Clamp to smallest positive normal float to avoid -inf / NaN on x <= 0
    if ( x < 1.17549435e-38f ) {
      x = 1.17549435e-38f;
    }

    union { float f; uint32_t i; } vx = { x };
    union { uint32_t i; float f; } mx = { (vx.i & 0x007FFFFFu) | 0x3F000000u };

    // Minimax polynomial approximation of log2(m) on [0.5, 1.0)
    const float y = float(vx.i) * 1.1920928955078125e-7f; // 1.0f / (1 << 23)
    const float m = mx.f; // m in [0.5, 1.0)
    return y - 124.22551499f - 1.498030302f * m - 1.72587999f / (0.3520887068f + m);
  }

  __attribute__((always_inline))
  static inline float fast_exp2(float p)
  {
    // Clamp exponent to safe IEEE-754 float range [-126, 127]
    if ( p < -126.0f ) {
      return 0.0f;
    }
    if ( p > 127.0f ) {
      p = 127.0f;
    }

    // Minimax polynomial approximation of 2^z on [0, 1)
    const float offset = (p < 0.0f) ? 1.0f : 0.0f;
    const float z = p - int(p - offset); // fractional part in [0, 1)
    const float approx = p + 121.2740575f + 27.7280233f / (4.84252568f - z) - 1.49012907f * z;
    union { uint32_t i; float f; } v = { uint32_t((1 << 23) * approx) };
    return v.f;
  }

  __attribute__((always_inline))
  static inline float fast_pow(float x, float y)
  {
    return (x <= 0.0f) ? 0.0f : fast_exp2(y * fast_log2(x));
  }

  // Primary rational formula, k-midtones, s-shadows, h-highlighs
  // k, s, h are computed from user inputs inside of update_coeffs()
  __attribute__((always_inline))
  static inline float eval(float t, float k, float s, float h)
  {
    const float ts = fast_pow(t, s);
    const float k1t_h = fast_pow(k * (1.0f - t), h);
    return ts / (ts + k1t_h + 1e-9f);
  }
#endif

  __attribute__((always_inline))
  inline float eval(float t) const
  {
    return eval(t, _k, _s, _h);
  }

  // The same as primary eval() above but uses log/exp to replace std::pow()
  __attribute__((always_inline))
  static inline float eeval(float t, float log_k, float s, float h)
  {
    // pow(t, shadow) = exp(y * log(x))
    // pow(k * (1.0f - t), h) = exp(h * (log(k) + log(1-t)))
    const float ts = std::exp(s* std::log(t));
    const float log_1_minus_t = std::log(1.0f - t);
    const float k1t_h = std::exp(h * (log_k + log_1_minus_t));
    return ts / (ts + k1t_h + 1e-12f);
  }

  __attribute__((always_inline))
  inline float eeval(float t) const
  {
    return eval(t, _log_k, _s, _h);
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
  template<typename Tin, typename Tout>
  void parallel_apply(const cv::Mat & src, cv::Mat & dst) const;
  template<typename _Tp>
  void dispatch_output(const cv::Mat & src, cv::Mat & dst) const;

protected:
  Options _opts;
  float _imin = 0.f;
  float _imax = 1.f;
  float _omin = 0.f;
  float _omax = 1.f;
  float _k = 1.0f;
  float _s = 1.0f;
  float _h = 1.0f;
  float _log_k = 0;
  float _xmin = 0.f;
  float _xmax = 1.f;
  float _xscale = 1.f;
  mutable cv::Mat1b _lut8;
  mutable cv::Mat1w _lut16;
};


#endif // __c_smooth_rational_mtf_h__
