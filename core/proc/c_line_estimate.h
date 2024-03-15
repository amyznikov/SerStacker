/*
 * c_line_estimate.h
 *
 *  Created on: Nov 17, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_running_line_estimate_h__
#define __c_running_line_estimate_h__

/**
 * The Least Squares Line Regression
 *  appropriate for running estimation
 *
 *  y  = shift + slope * x
 *
 * */
template<class T = double>
class c_line_estimate
{
public:
  void reset()
  {
    sx = sy = sxy = sx2 = 0;
    n = 0;
  }

  void update(T x, T y )
  {
    sx += x;
    sy += y;
    sxy += x * y;
    sx2 += x * x;
    n += 1;
  }

  void remove(T x, T y )
  {
    sx -= x;
    sy -= y;
    sxy -= x * y;
    sx2 -= x * x;
    n -= 1;
  }

  T slope() const
  {
    return (n * sxy - sx * sy) / (n * sx2 - sx * sx);
  }

  T shift() const
  {
    return (sy - slope() * sx) / n;
  }

  T a0() const
  {
    return shift();
  }

  T a1() const
  {
    return slope();
  }

  int pts() const
  {
    return n;
  }

protected:
  T sx = 0, sy = 0, sxy = 0, sx2 = 0;
  int n = 0;
};

#endif /* __c_running_line_estimate_h__ */
