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

  bool compute(T &_shift, T &_slope) const
  {
    if (n > 1) {
      _slope = slope();
      _shift = (sy - _slope * sx) / n;
      return true;
    }
    return false;
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

/**
 * The Weighted Least Squares Line Regression
 *  appropriate for running estimation
 *
 *  y  = shift + slope * x
 *
 * */
template<class T = double>
class c_weighted_line_estimate
{
public:
  // Reset all accumulators and the point counter
  void reset()
  {
    swx = swy = swxy = swx2 = swy2 = sw = 0;
    n = 0;
  }

  // Add a new point with weight w (default 1)
  void update(T x, T y, T w = 1)
  {
    const T wx = w * x;
    const T wy = w * y;
    swx  += wx;
    swy  += wy;
    swxy += wx * y;
    swx2 += wx * x;
    swy2 += wy * y;
    sw   += w;
    n    += 1;
  }

  // Remove a point (useful for implementing a sliding window)
  void remove(T x, T y, T w = 1)
  {
    const T wx = w * x;
    const T wy = w * y;
    swx  -= wx;
    swy  -= wy;
    swxy -= wx * y;
    swx2 -= wx * x;
    swy2 -= wy * y;
    sw   -= w;
    n    -= 1;
  }

  // Compute basic coefficients: shift and slope
  bool compute(T &_shift, T &_slope) const
  {
    const T denom = sw * swx2 - swx * swx;
    if (n > 1 && denom > 0 && sw > 0) {
      _slope = (sw * swxy - swx * swy) / denom;
      _shift = (swy - _slope * swx) / sw;
      return true;
    }
    return false;
  }

  // Compute the line parameters and the square of the standard deviation (stdev^2)
  bool compute(T &_shift, T &_slope, T &_stdev2) const
  {
    const T denom = sw * swx2 - swx * swx;
    if (n > 1 && denom > 0 && sw > 0) {
      _slope = (sw * swxy - swx * swy) / denom;
      _shift = (swy - _slope * swx) / sw;

      const T rss = swy2 - _slope * swxy - _shift * swy;
      _stdev2 = (rss > 0) ? (rss / sw) : 0;
      return true;
    }
    return false;
  }

  // Compute all, including the coefficient of determination R2.
  // The coefficient of determination R^2 shows how much of the data variation your model explains.
  // R^2 = 1 => Perfect fit.
  // R^2 = 0 =-> The straight line is no better than just the mean.
  bool compute(T &_shift, T &_slope, T &_stdev2, T &_r2) const
  {
    const T denom = sw * swx2 - swx * swx;
    if (n > 1 && denom > 0 && sw > 0) {
      _slope = (sw * swxy - swx * swy) / denom;
      _shift = (swy - _slope * swx) / sw;

      const T rss = swy2 - _slope * swxy - _shift * swy;
      _stdev2 = (rss > 0) ? (rss / sw) : 0;

      const T tss_denom = sw * swy2 - swy * swy;
      if (tss_denom > 0 && rss > 0) {
        _r2 = T(1) - (rss * sw / tss_denom);
      } else {
        _r2 = T(1); // Perfect match or horizontal line
      }
      return true;
    }
    return false;
  }

  // Fast slope calculation without offset calculation
  T slope() const
  {
    const T denom = sw * swx2 - swx * swx;
    return (denom > 0) ? (sw * swxy - swx * swy) / denom : 0;
  }

  // Number of processed points
  int pts() const
  {
    return n;
  }

  // Total weight of all points
  T total_weight() const
  {
    return sw;
  }

protected:
  T swx = 0, swy = 0, swxy = 0, swx2 = 0, swy2 = 0, sw = 0;
  int n = 0;
};


/**
 * Exponentially Weighted Linear Regression
*   Approximates the line y = shift + slope * x
*     with exponential decay of old data.
*  */
template<class T = double>
class c_sliding_line_estimate
{
public:
  explicit c_sliding_line_estimate(T win_size = 3)
  {
    reset(win_size);
  }

  void reset(T win_size = 3)
  {
    // Beta determines the speed of forgetting (analogous to a window)
    sw = swx = swy = swx2 = swxy = 0;
    n = 0;
    beta = (win_size > 0) ? T(1) / T(win_size) : T(1e6);
  }

  void update(T x, T y, T w = T(1) )
  {
    // Gamma determines the speed of history forgetting
    // Rational approximation of attenuation: f(w) = 1 / (1 + w/win_size)
    // 1. For w -> 0 (outlayer), adaptive_gamma -> 1.0 (memory is preserved)
    // 2. For w = 1.0 (normal), adaptive_gamma = 1 / (1 + 1/win_size) = gamma
    // 3. For w >> 1 (high confidence), adaptive_gamma -> 0 (fast update)

    const T gamma = T(1) / (T(1) + beta * w);
    sw = sw * gamma + w;
    swx = swx * gamma + w * x;
    swy = swy * gamma + w * y;
    swx2 = swx2 * gamma + w * x * x;
    swxy = swxy * gamma + w * x * y;
    ++n;
  }

  bool compute(T & _shift, T & _slope) const
  {
    const T denom = sw * swx2 - swx * swx;
    if( n > 1 && denom > 0 ) {
      _slope = (sw * swxy - swx * swy) / denom;
      _shift = (swy - _slope * swx) / sw;
      return true;
    }
    return false;
  }

  bool update(T x, T y, T w, T & _shift, T & _slope)
  {
    update(x, y, w);
    return compute(_shift,_slope);
  }

  T slope() const
  {
    const T denom = sw * swx2 - swx * swx;
    return (sw * swxy - swx * swy) / denom;
  }

  int pts() const
  {
    return n;
  }

protected:
  T beta, sw, swx, swy, swx2, swxy;
  int n = 0;
};


/**
 * Fast Time-Decay Weighted Average (Rational Approximation).
 *  The larger k_decay is, the faster old data decays as dt increases.
 */
template<class T = double>
class c_sliding_average_fast_time
{
public:
  explicit c_sliding_average_fast_time(T k_decay = 0.5)
  {
    reset(k_decay);
  }

  void reset(T k_decay = 0.5)
  {
    sw = swy = last_x = 0;
    k = k_decay;
    n = 0;
  }

  inline void update(T x, T y, T w = T(1) )
  {
    if (n > 0) {
      const T dt = x - last_x;
      const T gamma = T(1) / (T(1) + k * dt);
      sw  = sw  * gamma;
      swy = swy * gamma;
    }

    sw  += w;
    swy += w * y;
    last_x = x;
    ++n;
  }

  inline T average() const
  {
    return swy / sw;
  }

  int pts() const
  {
    return n;
  }

protected:
  T sw = 0;
  T swy = 0;
  T last_x = 0;
  T k = 0;
  int n = 0;
};

#endif /* __c_running_line_estimate_h__ */
