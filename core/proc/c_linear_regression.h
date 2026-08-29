/*
 * c_linear_regression.h
 *
 *  Created on: May 29, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_linear_regression_h__
#define __c_linear_regression_h__

#include <cfloat>


/**
 * The Least Squares estimate for linear regression
 *
 *  Zi = a0 * Xi + a1 * Yi
 *
 *
 * */

template<class T = double>
class c_linear_regression2
{
public:

  inline void reset()
  {
    X2 = 0;
    Y2 = 0;
    XY = 0;
    ZX = 0;
    ZY = 0;
  }

  inline void update(const T & xi, const T & yi, const T & zi)
  {
    X2 += xi * xi;
    Y2 += yi * yi;
    ZX += zi * xi;
    ZY += zi * yi;
    XY -= xi * yi;
  }

  inline bool compute(T & a0, T & a1) const
  {
    const T D = (X2 * Y2 - XY * XY);
    a0 = (Y2 * ZX + XY * ZY) / D;
    a1 = (XY * ZX + X2 * ZY) / D;
    return D != 0;
  }

protected:
  T X2 = 0;
  T Y2 = 0;
  T XY = 0;
  T ZX = 0;
  T ZY = 0;
};

typedef c_linear_regression2<float> c_linear_regression2f;
typedef c_linear_regression2<double> c_linear_regression2d;


/**
 * Weighted 2-factors linear regression problem solver (No intercept)
 *
 *    y[i] = a0 * x0[i] + a1 * x1[i];
 *
 *  Strictly minimizes the weighted residual sum of squares:
 *    RSS = SUM( w[i] * (y[i] - y_pred[i])^2 )
 *
 */
template<class _Tp>
class c_weighted_linear_regression2
{
public:

  c_weighted_linear_regression2()
  {
    reset();
  }

  void reset()
  {
    m00 = m01 = 0;
    m10 = m11 = 0;
    b0 = b1 = 0;
    sw = 0;
    n = 0;
  }

  // Add a new point with weight w (default 1)
  void update(_Tp x0, _Tp x1, _Tp y, _Tp w = 1)
  {
    const _Tp wx0 = w * x0;
    const _Tp wx1 = w * x1;

    m00 += wx0 * x0;
    m01 += wx0 * x1; // m01 == m10

    m10 += wx1 * x0;
    m11 += wx1 * x1;

    b0 += wx0 * y;
    b1 += wx1 * y;

    sw += w;
    n  += 1;
  }

  // Remove a point (useful for implementing a sliding window)
  void remove(_Tp x0, _Tp x1, _Tp y, _Tp w = 1)
  {
    const _Tp wx0 = w * x0;
    const _Tp wx1 = w * x1;

    m00 -= wx0 * x0;
    m01 -= wx0 * x1;

    m10 -= wx1 * x0;
    m11 -= wx1 * x1;

    b0 -= wx0 * y;
    b1 -= wx1 * y;

    sw -= w;
    n  -= 1;
  }

  // Compute coefficients a0 and a1 using Cramer's rule for 2x2 matrix
  bool compute(_Tp & a0, _Tp & a1) const
  {
    // Determinant of the 2x2 matrix
    const _Tp D = m00 * m11 - m01 * m10;

    if( D != 0 ) {
      const _Tp DEN = 1 / D;

      // Cramer's rule
      a0 = (b0 * m11 - m01 * b1) * DEN;
      a1 = (m00 * b1 - b0 * m10) * DEN;

      return true;
    }

    return false;
  }

  int pts() const { return n; }
  _Tp total_weight() const { return sw; }

protected:
  _Tp m00, m01;
  _Tp m10, m11;
  _Tp b0, b1;
  _Tp sw = 0;
  int n = 0;
};


/**
 * Simple utility to solve the 3-factors linear regression problem
 *
 *    y[i] = a0 * x0[i] + a1 * x1[i] + a2 * x2[i];
 *
 *
 * The example to use for hyperbolic regression estimation
 *   y(x) = (a * x + b) / (c * x +1)
 *
 *   [ x   1   -x*y ] [a]   [y]
 *                    [b] = [y]
 *                    [c]   [y]
 *
 *
 *  c_linear_regression3 l;
 *  for( size_t i = 0; i < n; ++i ) {
 *    l.update(x[i], 1, -x[i] * y[i], y[i]);
 *  }
 *  l.compute(a, b, c);
 *
 */
template<class _Tp = double>
class c_linear_regression3
{
public:

  c_linear_regression3()
  {
    reset();
  }

  void reset()
  {
    m00 = m01 = m02 = 0;
    m10 = m11 = m12 = 0;
    m20 = m21 = m22 = 0;
    b0 = b1 = b2 = 0;
  }

  void update(_Tp x0, _Tp x1, _Tp x2, _Tp y)
  {
    m00 += x0 * x0;
    m01 += x1 * x0;
    m02 += x2 * x0;

    m10 += x0 * x1;
    m11 += x1 * x1;
    m12 += x2 * x1;

    m20 += x0 * x2;
    m21 += x1 * x2;
    m22 += x2 * x2;

    b0 += x0 * y;
    b1 += x1 * y;
    b2 += x2 * y;
  }

  bool compute(_Tp & a0, _Tp & a1, _Tp & a2)
  {
    const _Tp a00 = m22 * m11 - m12 * m12;
    const _Tp a01 = m02 * m12 - m22 * m01;
    const _Tp a02 = m01 * m12 - m02 * m11;

    const _Tp a10 = a01;
    const _Tp a11 = m22 * m00 - m02 * m02;
    const _Tp a12 = m01 * m02 - m00 * m12;

    const _Tp a20 = a02;
    const _Tp a21 = a12;
    const _Tp a22 = m00 * m11 - m01 * m01;

    const _Tp D =
        m00 * a00 +
        m01 * a01 +
        m02 * a02;

    if( D != 0 ) {

      const _Tp DEN = _Tp(1) / D;

      a0 = (a00 * b0 + a01 * b1 + a02 * b2) * DEN;
      a1 = (a10 * b0 + a11 * b1 + a12 * b2) * DEN;
      a2 = (a20 * b0 + a21 * b1 + a22 * b2) * DEN;

      return true;
    }

    return false;
  }

protected:
  _Tp m00, m01, m02;
  _Tp m10, m11, m12;
  _Tp m20, m21, m22;
  _Tp b0, b1, b2;

};

typedef c_linear_regression3<float> c_linear_regression3f;
typedef c_linear_regression3<double> c_linear_regression3d;

/**
 * Simple utility to solve the weighted 3-factors linear regression problem
 * with weights w[i]
 *
 *  y[i] = a0 * x0[i] + a1 * x1[i] + a2 * x2[i];
 *
 * The example to use for hyperbolic regression estimation
 *   y(x) = (a * x + b) / (c * x +1)
 *
 *   [ x   1   -x*y ] [a]   [y]
 *                    [b] = [y]
 *                    [c]   [y]
 *
 *  c_weighted_linear_regression3 l;
 *  for( size_t i = 0; i < n; ++i ) {
 *    l.update(x[i], 1, -x[i] * y[i], y[i], w[i]);
 *  }
 *  l.compute(a, b, c);
 *
 */
template<class _Tp = double>
class c_weighted_linear_regression3
{
public:

  c_weighted_linear_regression3()
  {
    reset();
  }

  void reset()
  {
    m00 = m01 = m02 = 0;
    m10 = m11 = m12 = 0;
    m20 = m21 = m22 = 0;
    b0 = b1 = b2 = 0;
  }

  void update(_Tp x0, _Tp x1, _Tp x2, _Tp y, _Tp w = 1)
  {
    m00 += w * x0 * x0;
    m01 += w * x1 * x0;
    m02 += w * x2 * x0;

    m10 += w * x0 * x1;
    m11 += w * x1 * x1;
    m12 += w * x2 * x1;

    m20 += w * x0 * x2;
    m21 += w * x1 * x2;
    m22 += w * x2 * x2;

    b0 += w * x0 * y;
    b1 += w * x1 * y;
    b2 += w * x2 * y;
  }

  bool compute(_Tp & a0, _Tp & a1, _Tp & a2)
  {
    const _Tp a00 = m22 * m11 - m12 * m12;
    const _Tp a01 = m02 * m12 - m22 * m01;
    const _Tp a02 = m01 * m12 - m02 * m11;

    const _Tp a10 = a01;
    const _Tp a11 = m22 * m00 - m02 * m02;
    const _Tp a12 = m01 * m02 - m00 * m12;

    const _Tp a20 = a02;
    const _Tp a21 = a12;
    const _Tp a22 = m00 * m11 - m01 * m01;

    const _Tp D =
        m00 * a00 +
        m01 * a01 +
        m02 * a02;

    if( D != 0 ) {

      const _Tp DEN = _Tp(1) / D;

      a0 = (a00 * b0 + a01 * b1 + a02 * b2) * DEN;
      a1 = (a10 * b0 + a11 * b1 + a12 * b2) * DEN;
      a2 = (a20 * b0 + a21 * b1 + a22 * b2) * DEN;

      return true;
    }

    return false;
  }

protected:
  _Tp m00, m01, m02;
  _Tp m10, m11, m12;
  _Tp m20, m21, m22;
  _Tp b0, b1, b2;

};

typedef c_weighted_linear_regression3<float> c_weighted_linear_regression3f;
typedef c_weighted_linear_regression3<double> c_weighted_linear_regression3d;


/**
 * Simple utility to solve the 3-factors linear regression problem
 * with running statistical metrics estimation (stdev^2, R^2)
 *
 *    y[i] = a0 * x0[i] + a1 * x1[i] + a2 * x2[i];
 *
 * For polynomial parabola quadratic regression estimation:
 *    y(x) = a0 * x^2 + a1 * x + a2
 *    l.update(x*x, x, 1, y, w);
 *
 */
template<class _Tp = double>
class c_weighted_linear_regression3_with_stats
{
public:
  c_weighted_linear_regression3_with_stats()
  {
    reset();
  }

  void reset()
  {
    m00 = m01 = m02 = 0;
    m10 = m11 = m12 = 0;
    m20 = m21 = m22 = 0;
    b0 = b1 = b2 = 0;
    swy2 = swy = sw = 0;
    n = 0;
  }

  void update(_Tp x0, _Tp x1, _Tp x2, _Tp y, _Tp w = _Tp(1))
  {
    const _Tp wx0 = w * x0;
    const _Tp wx1 = w * x1;
    const _Tp wx2 = w * x2;
    const _Tp wy  = w * y;

    m00 += wx0 * x0;
    m01 += wx1 * x0;
    m02 += wx2 * x0;

    m10 += wx0 * x1;
    m11 += wx1 * x1;
    m12 += wx2 * x1;

    m20 += wx0 * x2;
    m21 += wx1 * x2;
    m22 += wx2 * x2;

    b0 += wx0 * y;
    b1 += wx1 * y;
    b2 += wx2 * y;

    swy2 += wy * y;
    swy  += wy;
    sw   += w;
    n    += 1;
  }

  bool compute(_Tp & a0, _Tp & a1, _Tp & a2) const
  {
    const _Tp a00 = m22 * m11 - m12 * m12;
    const _Tp a01 = m02 * m12 - m22 * m01;
    const _Tp a02 = m01 * m12 - m02 * m11;

    const _Tp a10 = a01;
    const _Tp a11 = m22 * m00 - m02 * m02;
    const _Tp a12 = m01 * m02 - m00 * m12;

    const _Tp a20 = a02;
    const _Tp a21 = a12;
    const _Tp a22 = m00 * m11 - m01 * m01;

    const _Tp D = m00 * a00 + m01 * a01 + m02 * a02;
    if( std::abs(D) > std::numeric_limits<_Tp>::epsilon() ) {
      const _Tp DEN = _Tp(1) / D;
      a0 = (a00 * b0 + a01 * b1 + a02 * b2) * DEN;
      a1 = (a10 * b0 + a11 * b1 + a12 * b2) * DEN;
      a2 = (a20 * b0 + a21 * b1 + a22 * b2) * DEN;
      return true;
    }

    return false;
  }

  // Coefficients + dispersion of parabolic residuals (stdev^2)
  bool compute(_Tp & a0, _Tp & a1, _Tp & a2, _Tp & _stdev2) const
  {
    if (n > 3 && compute(a0, a1, a2)) {
      // Residual sum of squares (RSS) for three-way linear regression:
      // RSS = sum(w * y^2) - a0*sum(w * x0 * y) - a1*sum(w * x1 * y) - a2*sum(w * x2 * y)
      // In terms of class accumulators: RSS = swy2 - a0 * b0 - a1 * b1 - a2 * b2
      // The stdev of residuals is normalized to the total weight (or sw, if the weights are normalized)

      const _Tp rss = swy2 - a0 * b0 - a1 * b1 - a2 * b2;
      _stdev2 = (rss > _Tp(0) && sw > _Tp(0)) ? (rss / sw) : _Tp(0);
      return true;
    }
    return false;
  }

  // Full regression calculation, including residual variance and R^2 coefficient of determination
  bool compute(_Tp & a0, _Tp & a1, _Tp & a2, _Tp & _stdev2, _Tp & _r2) const
  {
    if (n > 3 && compute(a0, a1, a2)) {
      const _Tp rss = swy2 - a0 * b0 - a1 * b1 - a2 * b2;
      _stdev2 = (rss > _Tp(0) && sw > _Tp(0)) ? (rss / sw) : _Tp(0);

      // Total sum of squared deviations (TSS) from the weighted mean of Y:
      // TSS_denom = sw * sum(w * y^2) - (sum(w * y))^2
      const _Tp tss_denom = sw * swy2 - swy * swy;

      if (tss_denom > _Tp(0) && rss > _Tp(0)) {
        // R^2 = 1 - (RSS * sw / TSS_denom)
        _r2 = _Tp(1) - (rss * sw / tss_denom);
      }
      else { // Perfect match of parabola with points
        _r2 = _Tp(1);
      }
      return true;
    }
    return false;
  }

  int pts() const
  {
    return n;
  }

  _Tp total_weight() const
  {
    return sw;
  }

protected:
  _Tp m00, m01, m02;
  _Tp m10, m11, m12;
  _Tp m20, m21, m22;
  _Tp b0, b1, b2;
  _Tp swy2, swy, sw;
  int n = 0;
};



/**
 * Exponentially Weighted 3-Factor Linear Regression
 *   Approximates y[i] = a0 * x0[i] + a1 * x1[i] + a2 * x2[i]
 *     with exponential decay of old data (analogous to a sliding window).
 * */
template<class _Tp = double>
class c_sliding_regression3
{
public:
  explicit c_sliding_regression3(_Tp win_size = 3)
  {
    reset(win_size);
  }

  void reset(_Tp win_size = 3)
  {
    m00 = m01 = m02 = 0;
    m10 = m11 = m12 = 0;
    m20 = m21 = m22 = 0;
    b0 = b1 = b2 = 0;
    n = 0;
    // Beta determines the speed of forgetting history
    beta = (win_size > 0) ? _Tp(1) / _Tp(win_size) : _Tp(1e6);
  }

  void update(_Tp x0, _Tp x1, _Tp x2, _Tp y, _Tp w = _Tp(1))
  {
    // Gamma determines the speed of history forgetting.
    // Rational approximation of attenuation: f(w) = 1 / (1 + w/win_size)
    const _Tp gamma = _Tp(1) / (_Tp(1) + beta * w);

    m00 = m00 * gamma + w * x0 * x0;
    m01 = m01 * gamma + w * x1 * x0;
    m02 = m02 * gamma + w * x2 * x0;

    m10 = m10 * gamma + w * x0 * x1;
    m11 = m11 * gamma + w * x1 * x1;
    m12 = m12 * gamma + w * x2 * x1;

    m20 = m20 * gamma + w * x0 * x2;
    m21 = m21 * gamma + w * x1 * x2;
    m22 = m22 * gamma + w * x2 * x2;

    b0 = b0 * gamma + w * x0 * y;
    b1 = b1 * gamma + w * x1 * y;
    b2 = b2 * gamma + w * x2 * y;

    ++n;
  }

  bool compute(_Tp & a0, _Tp & a1, _Tp & a2) const
  {
    if (n < 3) {
      return false;
    }

    const _Tp a00_matrix = m22 * m11 - m12 * m12;
    const _Tp a01_matrix = m02 * m12 - m22 * m01;
    const _Tp a02_matrix = m01 * m12 - m02 * m11;

    const _Tp a10_matrix = a01_matrix;
    const _Tp a11_matrix = m22 * m00 - m02 * m02;
    const _Tp a12_matrix = m01 * m02 - m00 * m12;

    const _Tp a20_matrix = a02_matrix;
    const _Tp a21_matrix = a12_matrix;
    const _Tp a22_matrix = m00 * m11 - m01 * m01;

    const _Tp D = m00 * a00_matrix + m01 * a01_matrix + m02 * a02_matrix;
    if( std::abs(D) > std::numeric_limits<_Tp>::epsilon() ) {
      const _Tp DEN = _Tp(1) / D;
      a0 = (a00_matrix * b0 + a01_matrix * b1 + a02_matrix * b2) * DEN;
      a1 = (a10_matrix * b0 + a11_matrix * b1 + a12_matrix * b2) * DEN;
      a2 = (a20_matrix * b0 + a21_matrix * b1 + a22_matrix * b2) * DEN;
      return true;
    }

    return false;
  }

  bool update(_Tp x0, _Tp x1, _Tp x2, _Tp y, _Tp w, _Tp & a0, _Tp & a1, _Tp & a2)
  {
    update(x0, x1, x2, y, w);
    return compute(a0, a1, a2);
  }

  int pts() const
  {
    return n;
  }

protected:
  _Tp beta;
  _Tp m00, m01, m02;
  _Tp m10, m11, m12;
  _Tp m20, m21, m22;
  _Tp b0, b1, b2;
  int n = 0;
};

/**
 * Exponentially Weighted 3-Factor Linear Regression (Coordinate-X Adaptive)
 *   Approximates y = a0 * x0 + a1 * x1 + a2 * x2
 *   History decay is determined by physical distance along the X axis.
 *
 *   for (int j = j_start; j < end_uniform_idx; ++j) {
 *     double x = x_min + (j * x_range / (N_uniform - 1));
 *     double y = smup[j];
 *     // Pass x itself as the first argument so that the class calculates the delta dx,
 *     // and then feed in the parabola factors: x^2, x, 1.0, and the value of y.
 *     sl_reg.update(x, 1.0, x, x * x, y);
 *   }
 *
 * */
template<class _Tp = double>
class c_sliding_regression3_x
{
public:
  // win_size_x sets the window radius/width directly in physical units of the X-axis (e.g. 2.0)
  explicit c_sliding_regression3_x(_Tp win_size_x)
  {
    reset(win_size_x);
  }

  void reset(_Tp win_size_x)
  {
    m00 = m01 = m02 = 0;
    m10 = m11 = m12 = 0;
    m20 = m21 = m22 = 0;
    b0 = b1 = b2 = 0;
    n = 0;

    // Initialize deep in the minus
    // Beta defines the rate of attenuation per unit physical distance X
    last_x = _Tp(-1e10);
    beta = (win_size_x > 0) ? _Tp(1) / win_size_x : _Tp(1e6);
  }

  // current physical x for calculating path delta
  void update(_Tp current_x, _Tp x0, _Tp x1, _Tp x2, _Tp y, _Tp w = _Tp(1))
  {
    _Tp gamma = _Tp(1);

    if (n > 0 && current_x > last_x) {
      // Physical step taken along the X-axis since the last update
      // Rational approximation of decay scaled by dx:
      //  The larger dx, the more the history decays.
      const _Tp dx = current_x - last_x;
      gamma = _Tp(1) / (_Tp(1) + beta * w * dx);
    }

    m00 = m00 * gamma + w * x0 * x0;
    m01 = m01 * gamma + w * x1 * x0;
    m02 = m02 * gamma + w * x2 * x0;

    m10 = m10 * gamma + w * x0 * x1;
    m11 = m11 * gamma + w * x1 * x1;
    m12 = m12 * gamma + w * x2 * x1;

    m20 = m20 * gamma + w * x0 * x2;
    m21 = m21 * gamma + w * x1 * x2;
    m22 = m22 * gamma + w * x2 * x2;

    b0 = b0 * gamma + w * x0 * y;
    b1 = b1 * gamma + w * x1 * y;
    b2 = b2 * gamma + w * x2 * y;

    last_x = current_x;
    ++n;
  }

  bool compute(_Tp & a0, _Tp & a1, _Tp & a2) const
  {
    if (n < 3) {
      return false;
    }

    const _Tp a00_matrix = m22 * m11 - m12 * m12;
    const _Tp a01_matrix = m02 * m12 - m22 * m01;
    const _Tp a02_matrix = m01 * m12 - m02 * m11;

    const _Tp a10_matrix = a01_matrix;
    const _Tp a11_matrix = m22 * m00 - m02 * m02;
    const _Tp a12_matrix = m01 * m02 - m00 * m12;

    const _Tp a20_matrix = a02_matrix;
    const _Tp a21_matrix = a12_matrix;
    const _Tp a22_matrix = m00 * m11 - m01 * m01;

    const _Tp D = m00 * a00_matrix + m01 * a01_matrix + m02 * a02_matrix;
    if( std::abs(D) > std::numeric_limits<_Tp>::epsilon() ) {
      const _Tp DEN = _Tp(1) / D;
      a0 = (a00_matrix * b0 + a01_matrix * b1 + a02_matrix * b2) * DEN;
      a1 = (a10_matrix * b0 + a11_matrix * b1 + a12_matrix * b2) * DEN;
      a2 = (a20_matrix * b0 + a21_matrix * b1 + a22_matrix * b2) * DEN;

      return true;
    }

    return false;
  }

  int pts() const
  {
    return n;
  }

protected:
  _Tp beta;
  _Tp last_x;
  _Tp m00, m01, m02;
  _Tp m10, m11, m12;
  _Tp m20, m21, m22;
  _Tp b0, b1, b2;
  int n = 0;
};

#endif /* __c_linear_regression_h__ */
