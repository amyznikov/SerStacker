/*
 * c_quad_estimate.h
 *
 *  Created on: Mar 10, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_quad_estimate_h__
#define __c_quad_estimate_h__

/**
 * Incremental form of the quadratic regression
 *
 *    y = a0 + a1 * x + a2 * x**2
 *
 * http://erikerlandson.github.io/blog/2012/07/05/deriving-an-incremental-form-of-the-polynomial-regression-equations
 *
 */
template<class T = double>
class c_quad_estimate
{
public:

  c_quad_estimate()
  {
    reset();
  }

  void reset()
  {
    sx = sy = sx2 = sx3 = sx4 = 0;
    sxy = sx2y = 0 ;
    Z = 0;
    n = 0;
  }

  void update(T x, T y )
  {
    sx += x;
    sy += y;

    sx2 += x * x;
    sx3 += x * x * x;
    sx4 += x * x * x * x;

    sxy += x * y;
    sx2y += x * x * y;

    ++n;

    Z = n * sx3 * sx3
        - 2 * sx3 * sx2 * sx
        + sx2 * sx2 * sx2
        - (n * sx2 - sx * sx) * sx4;
  }

  int pts() const
  {
    return n;
  }

  double a0() const
  {
    return (-(sx3 * sx - sx2 * sx2) * sx2y
        + (sx4 * sx - sx3 * sx2) * sxy
        - (sx4 * sx2 - sx3 * sx3) * sy
        ) / Z;

  }

  double a1() const
  {
    return ( (n * sx3 - sx2 * sx) * sx2y
            - (n * sx4 - sx2 * sx2) * sxy
            + (sx4 * sx - sx3 * sx2) * sy
        ) / Z;
  }

  double a2() const
  {
    return (-(n * sx2 - sx * sx) * sx2y
        + (n * sx3 - sx2 * sx) * sxy
        - (sx3 * sx - sx2 * sx2) * sy
        ) / Z;
  }

  double z() const
  {
    return Z;
  }

protected:
  T sx, sy, sx2, sx3, sx4;
  T sxy, sx2y;
  T Z;
  int n;
};

#endif /* __c_quad_estimate_h__ */
