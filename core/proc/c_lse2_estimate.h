/*
 * c_lse2_estimate.h
 *
 *  Created on: Jan 6, 2025
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_lse2_estimate_h__
#define __c_lse2_estimate_h__

#include <cfloat>
//#include <math.h>


/**
 * The Least Squares estimate for linear regression
 *
 *  Zi = a0 * Xi + a1 * Yi
 *
 *
 * */

template<class T = double>
class c_lse2_estimate
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

#endif /* __c_lse2_estimate_h__ */
