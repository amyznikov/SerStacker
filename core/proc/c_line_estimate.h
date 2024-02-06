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
 *  y  = slope * x + shift
 *
 * */
template<class T = double>
class c_line_estimate
{
public:

  T slope() const
  {
    return (sn * sxy - sx * sy) / (sn * sx2 - sx * sx);
  }

  T shift() const
  {
    return (sy - slope() * sx) / sn;
  }

  int pts() const
  {
    return sn;
  }

  void update(T x, T y )
  {
    sx += x;
    sy += y;
    sxy += x * y;
    sx2 += x * x;
    sn += 1;
  }

  void reset()
  {
    sx = sy = sxy = sx2 = 0;
    sn = 0;
  }

protected:
  T sx = 0, sy = 0, sxy = 0, sx2 = 0;
  int sn = 0;
};


#endif /* __c_running_line_estimate_h__ */
