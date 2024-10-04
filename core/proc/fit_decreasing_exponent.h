/*
 * fit_decreasing_exponent.h
 *
 *  Created on: Oct 4, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __fit_decreasing_exponent_h__
#define __fit_decreasing_exponent_h__

#include "c_line_estimate.h"
#include <cmath>

/**
 * fit_decreasing_exponent()
 *
 * This routine estimates parameters of exponentially decreasing function
 * for MONOTONICALLY DECREASING sequence of x[i], y[i]
 *
 *    y(x) = a + b * exp( c * x )
 *
 * CRITICAL REQUIREMENT:
 *  The input data x[], y[], MUST be preliminary sorted by X
 *  and the numerical derivative dy/dx must be always positive.
 *
 *  Used for bloom estimation on Valeo scans for monotonically decreasing
 *  subsets of point intensity profiles.
 *
 */


template<class VectorType, class T>
bool fit_decreasing_exponent(const VectorType & x, const VectorType & y, T * a, T * b, T * c)
{
  c_line_estimate<T> line;

  const size_t n =
      x.size();

  //
  // Use logarithm of numerical derivative dy/dx for estimation of parameter 'C'
  //
  for( size_t i = 1; i < n; ++i ) {

    line.update((x[i] + x[i - 1]) / 2,
        std::log(-(y[i] - y[i - 1]) / (x[i] - x[i - 1])));
  }

  *c = line.slope();

  //
  // Estimate parameters 'A' and 'B' using parameter 'C' estimated above
  //
  line.reset();

  for( size_t i = 0; i < n; ++i ) {
    line.update(std::exp(*c * x[i]), y[i]);
  }

  *a = line.shift();
  *b = line.slope();

  return true;
}




#endif /* __fit_decreasing_exponent_h__ */
