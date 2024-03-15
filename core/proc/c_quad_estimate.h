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
    zchanged = false;
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

    zchanged = true;
  }

  void remove(T x, T y)
  {
    sx -= x;
    sy -= y;

    sx2 -= x * x;
    sx3 -= x * x * x;
    sx4 -= x * x * x * x;

    sxy -= x * y;
    sx2y -= x * x * y;

    --n;

    zchanged = true;
  }

  int pts() const
  {
    return n;
  }

  T a0() const
  {
    updatez();

    return (-(sx3 * sx - sx2 * sx2) * sx2y
        + (sx4 * sx - sx3 * sx2) * sxy
        - (sx4 * sx2 - sx3 * sx3) * sy
        ) / Z;

  }

  T a1() const
  {
    updatez();

    return ( (n * sx3 - sx2 * sx) * sx2y
            - (n * sx4 - sx2 * sx2) * sxy
            + (sx4 * sx - sx3 * sx2) * sy
        ) / Z;
  }

  T a2() const
  {
    updatez();

    return (-(n * sx2 - sx * sx) * sx2y
        + (n * sx3 - sx2 * sx) * sxy
        - (sx3 * sx - sx2 * sx2) * sy
        ) / Z;
  }

  T z() const
  {
    updatez();
    return Z;
  }


protected:
  void updatez() const
  {
    if ( zchanged ) {

      zchanged = false;

      Z = n * sx3 * sx3
          - 2 * sx3 * sx2 * sx
          + sx2 * sx2 * sx2
          - (n * sx2 - sx * sx) * sx4;

    }
  }


protected:
  T sx, sy, sx2, sx3, sx4;
  T sxy, sx2y;
  mutable T Z;
  int n;
  mutable bool zchanged;
};


/**
 * Incremental form of the reduced quadratic regression with fixed A0 (value at y=0)
 *
 *    y = A0 + a1 * x + a2 * x**2
 *
 */
template<class T = double>
class c_quad0_estimate
{
public:

  c_quad0_estimate()
  {
    reset();
  }

  void reset()
  {
    sx = sx2 = sx3 = sx4 = sxy = sx2y = Z = 0;
    n = 0;
    zchanged = false;
  }

  void update(T x, T y )
  {
    sx += x;
    sx2 += x * x;
    sx3 += x * x * x;
    sx4 += x * x * x * x;
    sxy += x * y;
    sx2y += x * x * y;

    ++n;

    zchanged = true;
  }

  void remove(T x, T y)
  {
    sx -= x;
    sx2 -= x * x;
    sx3 -= x * x * x;
    sx4 -= x * x * x * x;
    sxy -= x * y;
    sx2y -= x * x * y;

    --n;

    zchanged = true;
  }

  int pts() const
  {
    return n;
  }

  void set_a0(T v)
  {
    A0 = v;
  }

  double a0() const
  {
    return A0;
  }

  T a1() const
  {
    updatez();
    return (sx4 * (sxy - A0 * sx)
        - sx3 * (sx2y - A0 * sx2)) / Z;
  }

  T a2() const
  {
    updatez();
    return (-sx3 * (sxy - A0 * sx)
        + sx2 * (sx2y - A0 * sx2)) / Z;
  }

  T z() const
  {
    updatez();
    return Z;
  }

protected:
  void updatez() const
  {
    if ( zchanged ) {
      zchanged = false;
      Z = sx2 * sx4 - sx3 * sx3;
    }
  }

protected:
  T A0 = 0;
  T sx, sx2, sx3, sx4, sxy, sx2y;
  mutable T Z;
  int n;
  mutable bool zchanged;
};


#endif /* __c_quad_estimate_h__ */
