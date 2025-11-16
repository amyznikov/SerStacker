/*
 * levmar3.h
 *
 *  Created on: Nov 16, 2025
 *      Author: amyznikov
 *
 * Based on the paper
 *  "The Levenberg-Marquardt algorithm for nonlinear least squares curve-fitting problems"
 *    Henri P. Gavin, Department of Civil and Environmental Engineering Duke University
 *    May 5, 2024
 *
 *  curvefit-pro:
 *    https://www.standardsapplied.com/curvefit-pro.html
 */

#pragma once
#ifndef __levmar3_h__
#define __levmar3_h__

#include <opencv2/opencv.hpp>
#include <limits>
#include <core/debug.h>


class c_levmar3_solver
{
  cv::Mat1d H, Hp, v, deltap, temp_d;
  std::vector<double> newparams;

public:
  struct callback
  {
    /**
     * User Callback for Hessian Matrix
     *
     * M is number of parameters;
     *
     * Jacobian at point x for parameter Pi :
     *  J(i) += df(x)/dPi
     *
     * MxM symmetric Hessian Matrix
     *  H(i,j) += J(i)*J(j);
     *
     * Mx1 vector v
     *  v(i) += J(i)*(f(x) - y)
     *
     * Typical implementation for f(x) = P0 / (1 + P1 * x):
     *  df(x) / dP0 = 1 / (1 + P1 * x)
     *  df(x) / dP01= P0 * x / (1 + P1 * x)**2
     *
     *  const double P0 = params[0];
     *  const double P1 = params[1];
     *  const size_t n = vx.size();
     *  double rhs = 0;
     *  double h00 = 0, h01 = 0, h10 = 0, h11 = 0;
     *  double v0 = 0, v1 = 0;
     *
     *  for (size_t i = 0; i < n; ++i) {
     *
     *   const double xi = x[i];
     *   const double yi = y[i];
     *   const double wi = 1; // point weight
     *   const double den = 1 / (1 + P1 * xi);
     *   const double dy = wi * (P0 * den - yi); // wi * (f(xi) - yi)
     *   rhs += dy * dy;
     *
     *   if (H) {
     *    const double J0 =  wi * den // wi * df(xi)/dP0
     *    const double J1 = -wi * P0 * xi * den * den; // wi * df(xi)/dP1
     *    v0 += J0 * dy;
     *    v1 += J1 * dy;
     *    h00 += jb * jb;
     *    h01 += jb * jc;
     *    h10 += jc * jb;
     *    h11 += jc * jc;
     *   }
     *  }
     *
     *  if ( H ) {
     *   H->create(2, 2);
     *   v->create(2, 1);
     *   (*H)(0,0) = h00;
     *   (*H)(0,1) = h01;
     *   (*H)(1,0) = h10;
     *   (*H)(1,1) = h11;
     *   (*v)(0, 0) = v0;
     *   (*v)(1, 0) = v1;
     *  }
     *  return (rhs);
     */
    virtual double compute(const std::vector<double> & params, cv::Mat1d * H, cv::Mat1d * v) const = 0;
    virtual ~callback() = default;
  };


  c_levmar3_solver()
  {
  }

  c_levmar3_solver(int max_itertions, double eps = 1e-6) :
      _max_iterations(max_itertions),
      _epsfn(eps),
      _epsx(eps)
  {
  }

  virtual ~c_levmar3_solver() = default;

  void set_max_iterations(int v)
  {
    _max_iterations = v;
  }

  int max_iterations() const
  {
    return _max_iterations;
  }

  void set_epsx(double v)
  {
    _epsx = v;
  }

  double epsx() const
  {
    return _epsx;
  }

  void set_epsfn(double v)
  {
    _epsfn = v;
  }

  double epsfn() const
  {
    return _epsfn;
  }

  void set_update_step_scale(double v)
  {
    _update_step_scale = v;
  }

  double update_step_scale() const
  {
    return _update_step_scale;
  }

  void set_initial_lambda(double v)
  {
    _initial_lambda = v;
  }

  double initial_lambda() const
  {
    return _initial_lambda;
  }

  void set_decomp_type(cv::DecompTypes v)
  {
    _decomp_type = v;
  }

  cv::DecompTypes decomp_type() const
  {
    return _decomp_type;
  }

  double rhs() const
  {
    return _errfn;
  }

  virtual int run(callback & cb, std::vector<double> & params)
  {
    constexpr double machine_eps = std::numeric_limits<double>::epsilon();

    const int M = params.size();

    double lambda = _initial_lambda;
    double err = 0;
    double newerr = 0;
    double dp = 0;

    _iteration = 0;

    while (_iteration < _max_iterations) {

      err = cb.compute(params, &H, &v);
      H.copyTo(Hp);

      /*
       * Solve normal equation for given Jacobian and lambda
       * */
      do {

        ++_iteration;

        /*
         * Increase diagonal elements by lambda
         * */
        for( int i = 0; i < M; ++i ) {
          H[i][i] = (1 + lambda) * Hp[i][i];
        }

        /*
         * Solve system to define delta and define new value of params
         *  deltap = H.inv() * v
         *  */
        if( !cv::solve(H, v, deltap, _decomp_type) ) {
          CF_ERROR("cv::solve() fails");
          break;
        }


        cv::scaleAdd(deltap, -_update_step_scale, cv::Mat(params), newparams);

        /* Compute function for newparams */
        newerr = cb.compute(newparams, nullptr, nullptr);

        /* Check for increments in parameters  */
        if( (dp = cv::norm(deltap, cv::NORM_INF)) < _epsx ) {
          // CF_DEBUG("BREAK by eps = %g / %g ", dp, _epsx);
          break;
        }

        /*
         * Compute update to lambda
         * */

        cv::gemm(Hp, deltap, -1, v, 2, temp_d);

        const double dS = deltap.dot(temp_d);
        const double rho = (err - newerr) / (std::abs(dS) > machine_eps ? dS : 1);

        if( rho > 0.25 ) {
          /* Accept new params and decrease lambda ==> Gauss-Newton method */
          if( lambda > 1e-6 ) {
            lambda = std::max(1e-6, lambda / 5);
          }
        }
        else if( rho > 0.1 ) {
          // CF_DEBUG(" NO CHANGE lambda->%g", lambda);
        }
        else if( lambda < 1 ) { /** Try increase lambda ==> gradient descend */
          lambda = 1;
          // CF_DEBUG("  lambda->%g", lambda);
        }
        else {
          lambda *= 10;
          // CF_DEBUG("  lambda->%g", lambda);
        }

        if( newerr < err ) {
          // CF_DEBUG("  ACCEPT");
          break;
        }

      } while (_iteration < _max_iterations);

      if( newerr < err ) {
        /*
         * Accept new params if were not yet accepted
         * */
        err = newerr;
        std::swap(params, newparams);
      }

      if( dp < _epsx ) {
        CF_DEBUG("BREAK2 by dp=%g epsx_=%g", dp, _epsx);
        break;
      }
    }

    _errx = dp;
    _errfn = err;
    _rmse = err;// sqrt(err / (_rhs.size() - M));

    return _iteration;
  }

protected:
  double _epsfn = 1e-6;
  double _epsx = 1e-6;
  double _update_step_scale = 1;
  double _initial_lambda = 1e-2;

  double _errx = -1;
  double _errfn = -1;
  double _rmse = -1;

  int _max_iterations = 100;
  int _iteration = -1;
  cv::DecompTypes _decomp_type = cv::DECOMP_EIG;
};



#endif /* __levmar3_h__ */
