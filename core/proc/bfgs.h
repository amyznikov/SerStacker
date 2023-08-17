/*
 * bfgs.h
 *
 *  Created on: Aug 12, 2023
 *      Author: amyznikov
 *
 * Broyden-Fletcher-Goldfarb-Shanno (BFGS)
 *
 * Adapted from "Limited memory BFGS (L-BFGS)"
 *  <https://github.com/chokkan/liblbfgs.git>
 *
 * @sa <https://habr.com/ru/articles/333356>
 */

#pragma once
#ifndef __bfgs_h__
#define __bfgs_h__

#include <opencv2/opencv.hpp>

/**
 * Broyden-Fletcher-Goldfarb-Shanno (BFGS)
 *
 * Adapted from "Limited memory BFGS (L-BFGS)"
 *  <https://github.com/chokkan/liblbfgs.git>
 *
 * @sa <https://habr.com/ru/articles/333356>
 */
class c_bfgs
{
public:
  typedef c_bfgs this_class;

  enum STATUS {
    /** L-BFGS reaches convergence. */
    LBFGS_SUCCESS = 0,
    LBFGS_CONVERGENCE = 0,
    LBFGS_STOP,
    /** The initial variables already minimize the objective function. */
    LBFGS_ALREADY_MINIMIZED,

    /** Unknown error. */
    LBFGSERR_UNKNOWNERROR = -1024,
    /** Logic error. */
    LBFGSERR_LOGICERROR,
    /** Insufficient memory. */
    LBFGSERR_OUTOFMEMORY,
    /** The minimization process has been canceled. */
    LBFGSERR_CANCELED,
    /** Invalid number of variables specified. */
    LBFGSERR_INVALID_N,
    /** Invalid number of variables (for SSE) specified. */
    LBFGSERR_INVALID_N_SSE,
    /** The array x must be aligned to 16 (for SSE). */
    LBFGSERR_INVALID_X_SSE,
    /** Invalid parameter lbfgs_parameter_t::epsilon specified. */
    LBFGSERR_INVALID_EPSILON,
    /** Invalid parameter lbfgs_parameter_t::past specified. */
    LBFGSERR_INVALID_TESTPERIOD,
    /** Invalid parameter lbfgs_parameter_t::delta specified. */
    LBFGSERR_INVALID_DELTA,
    /** Invalid parameter lbfgs_parameter_t::linesearch specified. */
    LBFGSERR_INVALID_LINESEARCH,
    /** Invalid parameter lbfgs_parameter_t::max_step specified. */
    LBFGSERR_INVALID_MINSTEP,
    /** Invalid parameter lbfgs_parameter_t::max_step specified. */
    LBFGSERR_INVALID_MAXSTEP,
    /** Invalid parameter lbfgs_parameter_t::ftol specified. */
    LBFGSERR_INVALID_FTOL,
    /** Invalid parameter lbfgs_parameter_t::wolfe specified. */
    LBFGSERR_INVALID_WOLFE,
    /** Invalid parameter lbfgs_parameter_t::gtol specified. */
    LBFGSERR_INVALID_GTOL,
    /** Invalid parameter lbfgs_parameter_t::xtol specified. */
    LBFGSERR_INVALID_XTOL,
    /** Invalid parameter lbfgs_parameter_t::max_linesearch specified. */
    LBFGSERR_INVALID_MAXLINESEARCH,
    /** Invalid parameter lbfgs_parameter_t::orthantwise_c specified. */
    LBFGSERR_INVALID_ORTHANTWISE,
    /** Invalid parameter lbfgs_parameter_t::orthantwise_start specified. */
    LBFGSERR_INVALID_ORTHANTWISE_START,
    /** Invalid parameter lbfgs_parameter_t::orthantwise_end specified. */
    LBFGSERR_INVALID_ORTHANTWISE_END,
    /** The line-search step went out of the interval of uncertainty. */
    LBFGSERR_OUTOFINTERVAL,
    /** A logic error occurred; alternatively, the interval of uncertainty
     became too small. */
    LBFGSERR_INCORRECT_TMINMAX,
    /** A rounding error occurred; alternatively, no line-search step
     satisfies the sufficient decrease and curvature conditions. */
    LBFGSERR_ROUNDING_ERROR,
    /** The line-search step became smaller than lbfgs_parameter_t::min_step. */
    LBFGSERR_MINIMUMSTEP,
    /** The line-search step became larger than lbfgs_parameter_t::max_step. */
    LBFGSERR_MAXIMUMSTEP,
    /** The line-search routine reaches the maximum number of evaluations. */
    LBFGSERR_MAXIMUMLINESEARCH,
    /** The algorithm routine reaches the maximum number of iterations. */
    LBFGSERR_MAXIMUMITERATION,
    /** Relative width of the interval of uncertainty is at most
     lbfgs_parameter_t::xtol. */
    LBFGSERR_WIDTHTOOSMALL,
    /** A logic error (negative line-search step) occurred. */
    LBFGSERR_INVALIDPARAMETERS,
    /** The current search direction increases the objective function value. */
    LBFGSERR_INCREASEGRADIENT,
  };

  /**
   * Line search algorithms.
   */
  enum LINESEARCH {
      /** The default algorithm (MoreThuente method). */
      LBFGS_LINESEARCH_DEFAULT = 0,
      /** MoreThuente method proposd by More and Thuente. */
      LBFGS_LINESEARCH_MORETHUENTE = 0,
      /**
       * Backtracking method with the Armijo condition.
       *  The backtracking method finds the step length such that it satisfies
       *  the sufficient decrease (Armijo) condition,
       *    - f(x + a * d) <= f(x) + lbfgs_parameter_t::ftol * a * g(x)^T d,
       *
       *  where x is the current point, d is the current search direction, and
       *  a is the step length.
       */
      LBFGS_LINESEARCH_BACKTRACKING_ARMIJO = 1,
      /** The backtracking method with the defualt (regular Wolfe) condition. */
      LBFGS_LINESEARCH_BACKTRACKING = 2,
      /**
       * Backtracking method with regular Wolfe condition.
       *  The backtracking method finds the step length such that it satisfies
       *  both the Armijo condition (LBFGS_LINESEARCH_BACKTRACKING_ARMIJO)
       *  and the curvature condition,
       *    - g(x + a * d)^T d >= lbfgs_parameter_t::wolfe * g(x)^T d,
       *
       *  where x is the current point, d is the current search direction, and
       *  a is the step length.
       */
      LBFGS_LINESEARCH_BACKTRACKING_WOLFE = 2,
      /**
       * Backtracking method with strong Wolfe condition.
       *  The backtracking method finds the step length such that it satisfies
       *  both the Armijo condition (LBFGS_LINESEARCH_BACKTRACKING_ARMIJO)
       *  and the following condition,
       *    - |g(x + a * d)^T d| <= lbfgs_parameter_t::wolfe * |g(x)^T d|,
       *
       *  where x is the current point, d is the current search direction, and
       *  a is the step length.
       */
      LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE = 3,
  };


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

  virtual STATUS run(const callback & cb, std::vector<double> & x);

  STATUS status() const;

  int iterations() const;

  int m() const;
  void set_m(int v);

  double epsilon() const;
  void set_epsilon(double v);

  int past() const;
  void set_past(int v);

  double delta() const;
  void set_delta(double v);

  int max_iterations() const;
  void set_max_iterations(int v);

  LINESEARCH linesearch() const;
  void set_linesearch(LINESEARCH v);

  int max_linesearch() const;
  void set_max_linesearch(int v);

  double min_step() const;
  void set_min_step(double v);

  double max_step() const;
  void set_max_step(double v);

  double ftol() const;
  void set_ftol(double v);

  double wolfe() const;
  void set_wolfe(double v);

  double gtol() const;
  void set_gtol(double v);

  double xtol() const;
  void set_xtol(double v);

  double orthantwise_c() const;
  void set_orthantwise_c(double v);

  int orthantwise_start() const;
  void set_orthantwise_start(double v);

  int orthantwise_end() const;
  void set_orthantwise_end(double v);

protected:

  bool compute(const callback & cb, const std::vector<double> & x,
      double * f, std::vector<double> * g) const;

  STATUS line_search_morethuente(const callback & cb,
      std::vector<double> & x, /* [n] */
      double *f,
      std::vector<double> & g, /* [n] */
      const std::vector<double> & s, /* [n] */
      double * stp,
      const std::vector<double> & xp, /* [n] */
      const std::vector<double> & gp, /* [n] */
      std::vector<double> & wa /* [n] */);

  STATUS line_search_backtracking_owlqn(const callback & cb,
      std::vector<double> & x, /* [n] */
      double *f,
      std::vector<double> & g, /* [n] */
      const std::vector<double> & s, /* [n] */
      double * stp,
      const std::vector<double> & xp, /* [n] */
      const std::vector<double> & gp, /* [n] */
      std::vector<double> & wp /* [n] */);

  STATUS line_search_backtracking(const callback & cb,
      std::vector<double> & x, /* [n] */
      double *f,
      std::vector<double> & g, /* [n] */
      const std::vector<double> & p, /* [n] */
      double * stp,
      const std::vector<double> & xp, /* [n] */
      const std::vector<double> & gp, /* [n] */
      std::vector<double> & wa /* [n] */);

  static int update_trial_interval(
      double * x,
      double * fx,
      double * dx,
      double * y,
      double * fy,
      double * dy,
      double * t,
      double * ft,
      double * dt,
      const double tmin,
      const double tmax,
      int * brackt);

protected:
  STATUS status_ = LBFGS_SUCCESS;
  int num_iterations_ = 0;

  /**
   * The number of corrections to approximate the inverse hessian matrix.
   *  The L-BFGS routine stores the computation results of previous \ref m
   *  iterations to approximate the inverse hessian matrix of the current
   *  iteration. This parameter controls the size of the limited memories
   *  (corrections). The default value is \c 6. Values less than \c 3 are
   *  not recommended. Large values will result in excessive computing time.
   */
  int m_ = 6;

  /**
   * Epsilon for convergence test.
   *  This parameter determines the accuracy with which the solution is to
   *  be found. A minimization terminates when
   *      ||g|| < \ref epsilon * max(1, ||x||),
   *  where ||.|| denotes the Euclidean (L2) norm. The default value is
   *  \c 1e-5.
   */
  double epsilon_ = 1e-5;

  /**
   * Distance for delta-based convergence test.
   *  This parameter determines the distance, in iterations, to compute
   *  the rate of decrease of the objective function. If the value of this
   *  parameter is zero, the library does not perform the delta-based
   *  convergence test. The default value is \c 0.
   */
  int past_ = 0;

  /**
   * Delta for convergence test.
   *  This parameter determines the minimum rate of decrease of the
   *  objective function. The library stops iterations when the
   *  following condition is met:
   *      (f' - f) / f < \ref delta,
   *  where f' is the objective value of \ref past iterations ago, and f is
   *  the objective value of the current iteration.
   *  The default value is \c 1e-5.
   */
  double delta_ = 1e-5;

  /**
   * The maximum number of iterations.
   *  The lbfgs() function terminates an optimization process with
   *  ::LBFGSERR_MAXIMUMITERATION status code when the iteration count
   *  exceedes this parameter. Setting this parameter to zero continues an
   *  optimization process until a convergence or error. The default value
   *  is \c 0.
   */
  int max_iterations_ = 0;

  /**
   * The line search algorithm.
   *  This parameter specifies a line search algorithm to be used by the
   *  L-BFGS routine.
   */
  LINESEARCH linesearch_ = LBFGS_LINESEARCH_DEFAULT;

  /**
   * The maximum number of trials for the line search.
   *  This parameter controls the number of function and gradients evaluations
   *  per iteration for the line search routine. The default value is \c 40.
   */
  int max_linesearch_ = 40;

  /**
   * The minimum step of the line search routine.
   *  The default value is \c 1e-20. This value need not be modified unless
   *  the exponents are too large for the machine being used, or unless the
   *  problem is extremely badly scaled (in which case the exponents should
   *  be increased).
   */
  double min_step_ = 1e-20;

  /**
   * The maximum step of the line search.
   *  The default value is \c 1e+20. This value need not be modified unless
   *  the exponents are too large for the machine being used, or unless the
   *  problem is extremely badly scaled (in which case the exponents should
   *  be increased).
   */
  double max_step_ = 1e20;

  /**
   * A parameter to control the accuracy of the line search routine.
   *  The default value is \c 1e-4. This parameter should be greater
   *  than zero and smaller than \c 0.5.
   */
  double ftol_ = 1e-4;

  /**
   * A coefficient for the Wolfe condition.
   *  This parameter is valid only when the backtracking line-search
   *  algorithm is used with the Wolfe condition,
   *  ::LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE or
   *  ::LBFGS_LINESEARCH_BACKTRACKING_WOLFE .
   *  The default value is \c 0.9. This parameter should be greater
   *  the \ref ftol parameter and smaller than \c 1.0.
   */
  double wolfe_ = 0.9;

  /**
   * A parameter to control the accuracy of the line search routine.
   *  The default value is \c 0.9. If the function and gradient
   *  evaluations are inexpensive with respect to the cost of the
   *  iteration (which is sometimes the case when solving very large
   *  problems) it may be advantageous to set this parameter to a small
   *  value. A typical small value is \c 0.1. This parameter should be
   *  greater than the \ref ftol parameter (\c 1e-4) and smaller than
   *  \c 1.0.
   */
  double gtol_ = 0.9;

  /**
   * The machine precision for floating-point values.
   *  This parameter must be a positive value set by a client program to
   *  estimate the machine precision. The line search routine will terminate
   *  with the status code (::LBFGSERR_ROUNDING_ERROR) if the relative width
   *  of the interval of uncertainty is less than this parameter.
   */
  double xtol_ = 1.0e-16;

  /**
   * Coeefficient for the L1 norm of variables.
   *  This parameter should be set to zero for standard minimization
   *  problems. Setting this parameter to a positive value activates
   *  Orthant-Wise Limited-memory Quasi-Newton (OWL-QN) method, which
   *  minimizes the objective function F(x) combined with the L1 norm |x|
   *  of the variables, {F(x) + C |x|}. This parameter is the coeefficient
   *  for the |x|, i.e., C. As the L1 norm |x| is not differentiable at
   *  zero, the library modifies function and gradient evaluations from
   *  a client program suitably; a client program thus have only to return
   *  the function value F(x) and gradients G(x) as usual. The default value
   *  is zero.
   */
  double orthantwise_c_ = 0.0;

  /**
   * Start index for computing L1 norm of the variables.
   *  This parameter is valid only for OWL-QN method
   *  (i.e., \ref orthantwise_c != 0). This parameter b (0 <= b < N)
   *  specifies the index number from which the library computes the
   *  L1 norm of the variables x,
   *      |x| := |x_{b}| + |x_{b+1}| + ... + |x_{N}| .
   *  In other words, variables x_1, ..., x_{b-1} are not used for
   *  computing the L1 norm. Setting b (0 < b < N), one can protect
   *  variables, x_1, ..., x_{b-1} (e.g., a bias term of logistic
   *  regression) from being regularized. The default value is zero.
   */
  int orthantwise_start_ = 0;

  /**
   * End index for computing L1 norm of the variables.
   *  This parameter is valid only for OWL-QN method
   *  (i.e., \ref orthantwise_c != 0). This parameter e (0 < e <= N)
   *  specifies the index number at which the library stops computing the
   *  L1 norm of the variables x,
   */
  int orthantwise_end_ = -1;
};

#endif /* __bfgs_h__ */
