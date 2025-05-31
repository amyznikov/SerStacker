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
 * Simple utility to solve the 3-factors linear regression problem
 *
 *    y[i] = a0 * x0[i] + a1 * x1[i] + a2 * x2[i];
 *
 *
 * The example to use for hyperbolic regression estimation
 *   y(x) = (a * x + b) / ( c * x +1)
 *
 *   [ x   1   -x*y ] [a]   [y]
 *                    [b] = [y]
 *                    [c]   [y]
 *
 *
 *  c_linear_regression3d l;
 *  for( size_t i = 0; i < n; ++i ) {
 *    l.update(x[i], 1, -x[i] * y[i], y[i]);
 *  }
 *  l.compute(a, b, c);
 *
 */
template<class _Tp>
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

      const _Tp DEN = 1 / D;

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


#endif /* __c_linear_regression_h__ */
