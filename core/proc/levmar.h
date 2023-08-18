/*
 * levmar.h
 *
 *  Created on: Mar 7, 2023
 *      Author: amyznikov
 *
 * Based on cv::
 */

#pragma once
#ifndef __c_levmar_solver_h__
#define __c_levmar_solver_h__

#include <opencv2/opencv.hpp>

class c_levmar_solver
{
public:
  struct callback
  {
    virtual ~callback() = default;

    /**
     Computes error and optionally Jacobian for the specified vector of parameters.

     @param params the current vector of parameters
     @param rhs output vector of errors: rhs_i = actual_f_i - ideal_f_i
     @param jac output Jacobian: J_ij = d(ideal_f_i)/d(param_j)

     When jac = nullptr it means that it does not need to be computed.

     If analytical Jacobian is available then function must fill jac array
     rhs.size rows x params.size cols and set *have_analytical_jac = true,
     otherwise it will be computed by c_levmar_solver using central differences for
     numerical approximation of partial derivatives.

     The callback should explicitly allocate each output array (unless it's nullptr).
     */
    virtual bool compute(const std::vector<double> & params, std::vector<double> & rhs,
        cv::Mat1d * jac, bool * have_analytical_jac) const = 0;
  };


  c_levmar_solver();
  c_levmar_solver(int max_itertions, double eps = 1e-6);
  virtual ~c_levmar_solver() = default;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_epsx(double v);
  double epsx() const;

  void set_epsf(double v);
  double epsf() const;

  virtual int run(const callback & cb, std::vector<double> & params);

  double rmse() const;
  const std::vector<double> & rhs() const;


protected:
  bool compute(const callback & cb, const std::vector<double> & params,
      std::vector<double> & rhs, cv::Mat1d * J) const;

protected:
  std::vector<double> rhs_;
  double rmse_ = -1;
  int max_iterations_ = 100;
  double epsf_ = 1e-6;
  double epsx_ = 1e-6;
};

#endif /* __c_levmar_solver_h__ */
