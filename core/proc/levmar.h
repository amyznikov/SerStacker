/*
 * levmar.h
 *
 *  Created on: Aug 17, 2024
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
#ifndef __levmar_h__
#define __levmar_h__

#include <opencv2/opencv.hpp>
#include <limits>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif

#include <core/debug.h>

template<class _Tp>
class c_levmar_solver
{
public:

#if HAVE_TBB
  typedef tbb::blocked_range<int>
    tbb_range;
#endif

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
    virtual bool compute(const std::vector<_Tp> & params, std::vector<_Tp> & rhs, cv::Mat_<_Tp> * J, bool * have_jac) = 0;

    /* Return true if compute() can be called in parallel.
     * Can be useful to speedup numerical differentiation with central differences. */
    virtual bool allow_tbb() const
    {
      return false;
    }
  };

  enum STOP_REASON {
    STOP_REASON_UNKNOW,
    STOP_REASON_CONVERGED_RHS,
    STOP_REASON_CONVERGED_DP,
    STOP_REASON_CONVERGED_ERR,
    STOP_REASON_BAD_HESSIAN,
    STOP_REASON_BAD_RHS,
    STOP_REASON_BAD_DP,
    STOP_REASON_BAD_LAMBDA,
    STOP_REASON_MAX_ITERATIONS,
  };

  c_levmar_solver()
  {
  }

  c_levmar_solver(int max_itertions, _Tp eps = (_Tp)(1e-6)) :
    _max_iterations(max_itertions),
    _epsfn(eps),
    _epsx(eps)
  {
  }

  virtual ~c_levmar_solver() = default;

  void set_max_iterations(int v)
  {
    _max_iterations = v;
  }

  int max_iterations() const
  {
    return _max_iterations;
  }

  void set_epsx(_Tp v)
  {
    _epsx = v;
  }

  _Tp epsx() const
  {
    return _epsx;
  }

  void set_epsfn(_Tp v)
  {
    _epsfn = v;
  }

  _Tp epsfn() const
  {
    return _epsfn;
  }

  void set_update_step_scale(_Tp v)
  {
    _update_step_scale = v;
  }

  _Tp update_step_scale() const
  {
    return _update_step_scale;
  }

  void set_initial_lambda(_Tp v)
  {
    _initial_lambda = v;
  }

  _Tp initial_lambda() const
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

  bool converged() const
  {
    return _converged;
  }

  STOP_REASON stop_reason() const
  {
    return _stop_reason;
  }

  _Tp rmse() const
  {
    return _rmse;
  }

  const std::vector<_Tp> & rhs() const
  {
    return _rhs;
  }

#if 0
  virtual int run(callback & cb, std::vector<_Tp> & params)
  {
    INSTRUMENT_REGION("");

    const int M = params.size();
    constexpr _Tp  machine_eps = std::numeric_limits<_Tp >::epsilon();

    _Tp lambda = _initial_lambda;

    _Tp err = 0, newerr = 0, dp = 0;

    cv::Mat_<_Tp> H, Hp, v, deltap, temp_d;
    std::vector<_Tp > newparams;

    _iteration = 0;

    while (_iteration < _max_iterations) {

      if( (err = compute_hessian(cb, params, H, v)) < 0 ) {
        CF_ERROR("compute_hessian() fails");
        return -1;
      }

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

        /* Solve system to define delta and define new value of params */
        if( !cv::solve(H, v, deltap, _decomp_type) ) {
          CF_ERROR("cv::solve() fails");
          break;
        }

        cv::scaleAdd(deltap, -_update_step_scale, cv::Mat(params),
            newparams);

        /* Compute function for newparams */
        if( (newerr = compute_rhs(cb, newparams)) < 0 ) {
          CF_ERROR("compute_rhs() fails");
          return -1;
        }


        /* Check for increments in parameters  */
        if( (dp = cv::norm(deltap, cv::NORM_INF)) < _epsx ) {
          // CF_DEBUG("BREAK by eps = %g / %g ", dp, _epsx);
          break;
        }

        /*
         * Compute update to lambda
         * */

        cv::gemm(Hp, deltap, -1, v, 2, temp_d);

        const double dS =
            deltap.dot(temp_d);

        const double rho =
            (err - newerr) / (std::abs(dS) > machine_eps ? dS : 1);

        if( rho > 0.25 ) {
          /* Accept new params and decrease lambda ==> Gauss-Newton method */
          if( lambda > 1e-6 ) {
            lambda = std::max((_Tp) 1e-6, (_Tp) (lambda / 5));
          }
          // CF_DEBUG("  lambda->%g", lambda);
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
        break;
      }
    }

    _errx = dp;
    _errfn = err;
    _rmse = std::sqrt(err / (_rhs.size() - M));

    return _iteration;
  }
#else
  virtual int run(callback & cb, std::vector<_Tp> & params)
  {
    INSTRUMENT_REGION("");

    const int M = params.size();
    constexpr _Tp  machine_eps = std::numeric_limits<_Tp >::epsilon();

    _Tp lambda = _initial_lambda;

    cv::Mat_<_Tp> H, Hp, v, deltap, temp_d;
    std::vector<_Tp > newparams;

    const double epsx = _epsx;
    const double epsfn = _epsfn;
    const int max_iterations = std::max(1, _max_iterations);

    int iteration = 0;
    double dp = -1;
    double err = -1;

    _converged = false;
    _stop_reason = STOP_REASON_UNKNOW;

    while (iteration < max_iterations) {

      if( (err = compute_hessian(cb, params, H, v)) < 0 || !std::isfinite(err) ) {
        _stop_reason = STOP_REASON_BAD_HESSIAN;
        CF_ERROR("compute_hessian() fails: err=%g at iteration %d", err, iteration);
        break;
      }

      if ( err <= machine_eps ) {
        _converged = true;
        _stop_reason = STOP_REASON_CONVERGED_RHS;
        break;
      }

      H.copyTo(Hp);

      /*
       * Solve normal equation for given Jacobian and lambda
       * */
      while (iteration++ < max_iterations) {

        /*
         * Increase diagonal elements by lambda
         * */
        for( int i = 0; i < M; ++i ) {
          H[i][i] = (1 + lambda) * Hp[i][i];
        }

        /* Solve system to compute parameters update
         *  deltap = H.inv() * v
         *  */
        if( !cv::solve(H, v, deltap, _decomp_type) ) {
          CF_ERROR("cv::solve() fails at iteration %d", iteration);
          break;
        }

        cv::scaleAdd(deltap, -_update_step_scale, cv::Mat(params), newparams);

        /* Check for increment in parameters  */
        dp = cv::norm(deltap, cv::NORM_INF);
        if ( !isfinite(dp) ) {
          _converged = false;
          _stop_reason = STOP_REASON_BAD_DP;
          CF_ERROR("BAD dp=%g at iteration %d", dp, iteration);
          break;
        }
        if( dp <= epsx ) {
          // accept new params
          std::swap(params, newparams);
          _converged = true;
          _stop_reason = STOP_REASON_CONVERGED_DP;
          CF_ERROR("CONVERGED_DP dp=%g <= epsx=%g at iteration %d", dp, epsx, iteration);
          break;
        }

        const double newerr = compute_rhs(cb, newparams);
        if ( newerr < 0 || !std::isfinite(err) ) {
          CF_ERROR("compute_rhs() fails: err=%g at iteration %d", newerr, iteration);
          _stop_reason = STOP_REASON_BAD_RHS;
          break; // no convergence
        }

        if( newerr > err ) {
          if ( lambda > _Tp(1e8) ) {
            CF_ERROR("lambda too big =%g at iteration %d", lambda, iteration);
            _stop_reason = STOP_REASON_BAD_LAMBDA;
            break; // no convergence
          }
          err = newerr;
          lambda *= 10;
          continue;
        }

        const _Tp diff = err - newerr;

        // accept new params
        std::swap(params, newparams);
        err = newerr;

        /* Check Function Tolerance */
        if (diff < err * epsfn ) {
          CF_DEBUG("diff=%lf", diff);
          _converged = true;
          _stop_reason = STOP_REASON_CONVERGED_ERR;
          break;
        }

        /*
         * Compute step quality (rho) and update to lambda
         * Predicted improvement dS
         * rho = (actual improvement) / (predicted improvement)
         * */
        cv::gemm(Hp, deltap, -1, v, 2, temp_d);
        const double dS = deltap.dot(temp_d);
        const double rho = std::abs(dS) > _Tp(1e-9f) ? diff / std::abs(dS)  : diff;
        if (rho > 0.25 ) { /* Good step, decrease lambda ==> Gauss-Newton */
          lambda = std::max(1e-8, 0.2 * lambda);
        }
        else if (rho < 0.1) { /* The Taylor model looks poor ==> gradient descend*/
          lambda = (lambda < 1.0) ? 1.0 : lambda * 10.0;
        }
        else {
        }

        break;
      }

      if (_converged || _stop_reason != STOP_REASON_UNKNOW ) {
        break;
      }
    }

    _errx = dp;
    _errfn = err;
    _rmse = std::sqrt(err / (_rhs.size() - M));

    if ( _stop_reason == STOP_REASON_UNKNOW  &&  iteration >= max_iterations ) {
      _stop_reason = STOP_REASON_MAX_ITERATIONS;
    }

    return (_iteration = iteration);
  }
#endif

protected:
  static bool compute(callback & cb, const std::vector<_Tp> & params, std::vector<_Tp> & rhs, cv::Mat_<_Tp> * J)
  {
    bool have_jac =
        false;

    if( !cb.compute(params, rhs, J, &have_jac) ) {
      CF_ERROR("cb.compute() fails");
      return false;
    }

    if( rhs.size() < params.size() ) {
      CF_ERROR("c_levmar_solver: cb.compute() returns invalid rhs.size=%zu < params.size=%zu", rhs.size(), params.size());
      return false;
    }

    if( J ) {

      // check if J was compute by callback
      if( have_jac ) {
        if( J->cols != rhs.size() || J->rows != params.size() ) {
          CF_ERROR("c_levmar_solver: Invalid Jac size %dx%d. Expected size is %zu cols x %zu rows", J->cols, J->rows, rhs.size(), params.size());
          return false;
        }
        return true;
      }

      // Compute numerical approximation of partial derivatives

  #if HAVE_TBB
      const bool use_tbb = cb.allow_tbb();
  #else
      const bool use_tbb = false;
  #endif

      J->create((int) params.size(),(int) rhs.size());

      if( !use_tbb ) {

        std::vector<_Tp> P = params;
        std::vector<_Tp> rhs1, rhs2;

        for( int j = 0, n = P.size(); j < n; ++j ) {

          const _Tp v =
              P[j];

          const _Tp delta =
              (std::max)((_Tp) (1e-6), (_Tp) (1e-4) * std::abs(v));

          P[j] = v + delta;
          if( !cb.compute(P, rhs1, nullptr, nullptr) ) {
            CF_ERROR("cb.compute() fails");
            return false;
          }

          if( rhs1.size() != rhs.size() ) {
            CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",  rhs.size(), rhs1.size());
            return false;
          }

          P[j] = v - delta;
          if( !cb.compute(P, rhs2, nullptr, nullptr) ) {
            CF_ERROR("cb.compute() fails");
            return false;
          }

          if( rhs2.size() != rhs.size() ) {
            CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal",
                rhs.size(), rhs1.size());
            return false;
          }

          const _Tp den = (_Tp) (0.5) / delta;
          for( size_t k = 0, nk = rhs.size(); k < nk; ++k ) {
            (*J)[j][k] = den * (rhs1[k] - rhs2[k]);
          }

          P[j] = v;
        }

      }
  #if HAVE_TBB
      else {

        const int rhs_size = rhs.size();

        bool fail[params.size()] = { false };

        tbb::parallel_for(0, (int) params.size(), 1,
            [&params, &J, &cb, rhs_size, &fail](int j) {

              std::vector<_Tp> P = params;
              std::vector<_Tp> rhs1, rhs2;

              const _Tp v = P[j];

              const _Tp delta = (std::max)((_Tp)(1e-6), (_Tp)(1e-4) * std::abs(v));

              P[j] = v + delta;
              if( !cb.compute(P, rhs1, nullptr, nullptr) ) {
                fail[j] = true;
                CF_ERROR("cb.compute() fails");
                return;   // false;
              }

              if( rhs1.size() != rhs_size ) {
                fail[j] = true;
                CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal", rhs_size, rhs1.size());
                return;   // false;
              }

              P[j] = v - delta;
              if( !cb.compute(P, rhs2, nullptr, nullptr) ) {
                fail[j] = true;
                CF_ERROR("cb.compute() fails");
                return;   // false;
              }

              if( rhs2.size() != rhs_size ) {
                fail[j] = true;
                CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal", rhs_size, rhs1.size());
                return;   // false;
              }

              const _Tp den = (_Tp)(0.5) / delta;
              for( size_t k = 0, nk = rhs_size; k < nk; ++k ) {
                (*J)[j][k] = den * (rhs1[k] - rhs2[k]);
              }

              P[j] = v;
            });

        for( size_t j = 0, n = params.size(); j < n; ++j ) {
          if( fail[j] ) {
            CF_ERROR("Compute Jac fails");
            return false;
          }
        }

      }
  #endif // HAVE_TBB
    }

    return true;
  }


  _Tp compute_rhs(callback & cb, std::vector<_Tp> & params)
  {
    if( !compute(cb, params, _rhs, nullptr) ) {
      return -1;
    }

    return (_Tp)cv::norm(_rhs, cv::NORM_L2SQR);
  }


  double compute_hessian(callback & cb, const std::vector<_Tp> & params, cv::Mat_<_Tp> & H, cv::Mat_<_Tp> & v)
  {
    if( !compute(cb, params, _rhs, &_J) ) {
      CF_ERROR("compute() fails");
      return -1;
    }

    const int N = _rhs.size();
    const int M = params.size();

    v.create(M, 1);

#if HAVE_TBB
    const bool use_tbb = cb.allow_tbb();
#else
    const bool use_tbb = false;
#endif

    if( !use_tbb ) {
      for( int i = 0; i < M; ++i ) {
        v(i, 0) = _J.row(i).dot(_rhs);
      }
    }
#if HAVE_TBB
    else {
      tbb::parallel_for(tbb_range(0, M),
          [&](const tbb_range & r) {

            for( int i = r.begin(); i < r.end(); ++i ) {
              v(i, 0) = _J.row(i).dot(_rhs);
            }
          });
    }
#endif

    H.create(M, M);

    if( !use_tbb ) {
      for( int i = 0; i < M; ++i ) {
        for( int j = 0; j <= i; ++j ) {
          H[i][j] = _J.row(i).dot(_J.row(j));
        }
      }
    }
#if HAVE_TBB
    else {
      tbb::parallel_for(tbb_range(0, M),
          [&](const tbb_range & r) {

            for( int i = r.begin(); i < r.end(); ++i ) {
              for( int j = 0; j <= i; ++j ) {
                H[i][j] = _J.row(i).dot(_J.row(j));
              }
            }
          });
    }
#endif

    for( int i = 0; i < M; ++i ) {
      for( int j = i + 1; j < M; ++j ) {
        H[i][j] = H[j][i];
      }
    }

    return cv::norm(_rhs, cv::NORM_L2SQR);
  }



protected:
  std::vector<_Tp> _rhs;
  cv::Mat_<_Tp> _J;
  _Tp _epsfn = (_Tp)(1e-6);
  _Tp _epsx = (_Tp)(1e-6);
  _Tp _update_step_scale = (_Tp)(1);
  _Tp _initial_lambda = (_Tp)(1e-1);

  _Tp _errx = -1;
  _Tp _errfn = -1;
  _Tp _rmse = -1;

  int _max_iterations = 100;
  int _iteration = -1;
  bool _converged = false;
  STOP_REASON _stop_reason = STOP_REASON_UNKNOW;
  cv::DecompTypes _decomp_type = cv::DECOMP_EIG;
};


typedef c_levmar_solver<float> c_levmarf_solver;
typedef c_levmar_solver<double> c_levmard_solver;

#endif /* __levmar_h__ */
