/*
 * fit_exponent.h
 *
 *  Created on: May 25, 2024
 *      Author: amyznikov
 *
 * This routine uses direct (no guessed initial values required, no iterative process) method
 * of fitting exponential function
 *
 *    y(x) = a + b * exp( c * x )
 *
 * The method is described in the paper 'REGRESSIONS and INTEGRAL EQUATIONS' by Jean Jacquelin.
 *
 * <https://math.stackexchange.com/questions/1337601/fit-exponential-with-constant>
 * <https://www.scribd.com/doc/14674814/Regressions-et-equations-integrales>
 *
 * CRITICAL REQUIREMENT:
 *  The input data x[], y[], MUST be preliminary sorted by X.
 *
 */

#pragma once
#ifndef __fit_exponent_h__
#define __fit_exponent_h__

#include <cmath>

/**
 * fit_exponent()
 *
 * This routine uses direct (no guessed initial values required, no iterative process) method
 * of fitting exponential function
 *
 *    y(x) = a + b * exp( c * x )
 *
 * The method is described in the paper 'REGRESSIONS and INTEGRAL EQUATIONS' by Jean Jacquelin.
 *
 * <https://math.stackexchange.com/questions/1337601/fit-exponential-with-constant>
 * <https://www.scribd.com/doc/14674814/Regressions-et-equations-integrales>
 *
 * CRITICAL REQUIREMENT:
 *  The input data x[], y[], MUST be preliminary sorted by X.
 *
 */

template<class T>
bool fit_exponent(const std::vector<T> & x, const std::vector<T> & y, T * a, T * b, T * c)
{
  const size_t n =
      x.size();

  const T x1 =
      x[0];

  const T y1 =
      y[0];

  T M00, M01, M11;
  T I00, I01, I11;
  T V0, V1;
  T Sk;
  T D;

  //
  // Accumulate matrix
  //
  M00 = 0;
  M01 = 0;
  M11 = 0;
  V0 = 0;
  V1 = 0;
  Sk = 0;

  for ( size_t k = 1; k < n; ++k ) {

    Sk += (y[k] + y[k - 1]) * (x[k] - x[k - 1]) / 2;

    M00 += (x[k] - x1) * (x[k] - x1);
    M01 += (x[k] - x1) * Sk;
    M11 += Sk * Sk;

    V0 += (y[k] - y1) * (x[k] - x1);
    V1 += (y[k] - y1) * Sk;
  }

  //
  // Invert matrix
  //
  D = M00 * M11 - M01 * M01;
  I00 = +M11 / D;
  I01 = -M01 / D;
  I11 = +M00 / D;

  // Compute C2
  const T c2 =
      I01 * V0 + I11 * V1;


  //
  // Accumulate matrix
  //

  M00 = static_cast<T>(n);
  M01 = 0;
  M11 = 0;
  V0 = 0;
  V1 = 0;

  for ( size_t k = 0; k < n; ++k ) {

    const T thk =
        std::exp(c2 * x[k]);

    M01 += thk;
    M11 += thk * thk;
    V0 += y[k];
    V1 += y[k] * thk;
  }

  //
  // Invert matrix
  //
  D = M00 * M11 - M01 * M01;
  I00 = +M11 / D;
  I01 = -M01 / D;
  I11 = +M00 / D;


  // Return results

  *a = I00 * V0 + I01 * V1;
  *b = I01 * V0 + I11 * V1;
  *c = c2;

  return true;
}


#endif /* __fit_exponent_h__ */
