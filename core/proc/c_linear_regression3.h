/*
 * c_linear_regression3.h
 *
 *  Created on: Jan 24, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_linear_regression3_h__
#define __c_linear_regression3_h__

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
    for ( int i = 0; i < 3; ++i ) {
      for ( int j = 0; j < 3; ++j ) {
        m[i][j] = 0;
      }
    }
    for ( int i = 0; i < 3; ++i ) {
      b[i] = 0;
    }
  }

  void update(_Tp x0, _Tp x1, _Tp x2, _Tp y)
  {
    m[0][0] += x0 * x0;
    m[0][1] += x1 * x0;
    m[0][2] += x2 * x0;


    m[1][0] += x0 * x1;
    m[1][1] += x1 * x1;
    m[1][2] += x2 * x1;

    m[2][0] += x0 * x2;
    m[2][1] += x1 * x2;
    m[2][2] += x2 * x2;

    b[0] += y * x0;
    b[1] += y * x1;
    b[2] += y * x2;
  }

  bool compute(_Tp & a0, _Tp & a1, _Tp & a2)
  {
    a[0][0] = m[2][2] * m[1][1] - m[1][2] * m[1][2];
    a[0][1] = m[0][2] * m[1][2] - m[2][2] * m[0][1];
    a[0][2] = m[0][1] * m[1][2] - m[0][2] * m[1][1];

    a[1][0] = a[0][1];
    a[1][1] = m[2][2] * m[0][0] - m[0][2] * m[0][2];
    a[1][2] = m[0][1] * m[0][2] - m[0][0] * m[1][2];

    a[2][0] = a[0][2];
    a[2][1] = a[1][2];
    a[2][2] = m[0][0] * m[1][1] - m[0][1] * m[0][1];

    const _Tp D =
        m[0][0] * a[0][0] +
        m[0][1] * a[0][1] +
        m[0][2] * a[0][2];

    if ( D != 0 ) {

      const _Tp S = 1 / D;

      a0 = (a[0][0] * b[0] + a[0][1] * b[1] + a[0][2] * b[2]) * S;
      a1 = (a[1][0] * b[0] + a[1][1] * b[1] + a[1][2] * b[2]) * S;
      a2 = (a[2][0] * b[0] + a[2][1] * b[1] + a[2][2] * b[2]) * S;

      return true;
    }

    return false;
  }

protected:
  _Tp m[3][3], a[3][3];
  _Tp b[3];

};

typedef c_linear_regression3<float> c_linear_regression3f;
typedef c_linear_regression3<double> c_linear_regression3d;


#endif /* __c_linear_regression3_h__ */
