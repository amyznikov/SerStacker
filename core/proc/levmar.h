/*
 * levmar3.h
 *
 *  Created on: Aug 17, 2024
 *      Author: amyznikov
 *
 * Based on the paper
 *  "The Levenberg-Marquardt algorithm for nonlinear least squares curve-fitting problems"
 *    Henri P. Gavin, Department of Civil and Environmental Engineering Duke University
 *    May 5, 2024
 */

#pragma once
#ifndef __levmar3_h__
#define __levmar3_h__

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
     @param jac output Jacobian matrix of size params.size() rows x rhs.size() columns (row-wise layout)
                J_ji = d(f_i)/d(param_j)

     When jac = nullptr it means that it does not need to be computed.

     If analytical Jacobian is available then function must fill jac array
     params.size() rows x rhs.size() cols and set *have_analytical_jac = true,
     otherwise it will be computed by c_levmar_solver using central differences for
     numerical approximation of partial derivatives.

     The callback should explicitly allocate each output array (unless it's nullptr).
     */
    virtual bool compute(const std::vector<double> & params, std::vector<double> & rhs,
        cv::Mat1d * J, bool * have_analytical_jac) = 0;

    /* Return true if compute() can be called in parallel.
     * Can be useful to speedup numerical differentiation with central differences. */
    virtual bool allow_tbb() const
    {
      return false;
    }
  };


  c_levmar_solver();
  c_levmar_solver(int max_itertions, double eps = (double)(1e-6));
  virtual ~c_levmar_solver() = default;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_epsx(double v);
  double epsx() const;

  void set_epsfn(double v);
  double epsfn() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  void set_initial_lambda(double v);
  double initial_lambda() const;

  void set_decomp_type(cv::DecompTypes v);
  cv::DecompTypes decomp_type() const;

  double rmse() const;
  const std::vector<double> & rhs() const;

  virtual int run(callback & cb, std::vector<double> & params);

protected:
  double compute_rhs(callback & cb, std::vector<double> & params);
  double compute_hessian(callback & cb, const std::vector<double> & params, cv::Mat1d & A, cv::Mat1d & v);
  static bool compute(callback & cb, const std::vector<double> & params, std::vector<double> & rhs, cv::Mat1d * J);

protected:
  std::vector<double> _rhs;
  cv::Mat1d _J;
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
