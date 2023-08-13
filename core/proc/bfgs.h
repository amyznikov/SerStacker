/*
 * bfgs.h
 *
 *  Created on: Aug 12, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __bfgs_h__
#define __bfgs_h__

#include <opencv2/opencv.hpp>

/**
 * Broyden-Fletcher-Goldfarb-Shanno (BFGS)
 *
 * Numerical Recipies in C
 *  Non-linear-Equations
 *  10.7 Variable Metric Methods in Multidimensions
 *
 */
class c_bfgs
{
public:
  struct callback
  {
    virtual ~callback() = default;

    /**
     Computes error and optionally gradient for the specified vector of parameters.

     When g = nullptr it means that it does not need to be computed.

     If analytical gradient is available then function must fill g[] array
     and set *have_g = true,
     otherwise it will be computed by solver using central differences for
     numerical approximation of partial derivatives.

     The callback should explicitly allocate each output array (unless it's nullptr).
     */
    virtual bool compute(const std::vector<double> & p, double * f,
        std::vector<double> * g, bool * have_g) const = 0;
  };

public:

  c_bfgs(int max_itertions);
  virtual ~c_bfgs();

  virtual int run(const callback & cb, std::vector<double> & params);

protected:
  bool compute(const callback & cb, const std::vector<double> & p,
      double * f, std::vector<double> * g) const;

  void lnsrch(int n, const std::vector<double> & xold, double fold,
      const std::vector<double> & g, const std::vector<double> & p,
      std::vector<double> & x, double * f,
      double stpmax, int * check, const callback & cb);


protected:
  // Maximum allowed number of iterations
  int max_iterations_ = 100;

  // Machine precision
  double eps_ = 3.0e-8;

  // Convergence criterion on x values
  double tolx_  = 4 * 3.0e-8;

  // Convergence criterion on gradient values
  double gtol_ = 1e-6;

  // Scaled maximum step length allowed in line searches
  double stpmx_  = 100.0;

};

#endif /* __bfgs_h__ */
