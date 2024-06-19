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
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif


template<class _Tp>
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
    virtual bool compute(const std::vector<_Tp> & params, std::vector<_Tp> & rhs,
        cv::Mat_<_Tp> * jac, bool * have_analytical_jac) const = 0;

    /* Return true if compute() can be called in parallel.
     * Can be useful to speedup numerical differentiation with central differences. */
    virtual bool thread_safe_compute() const
    {
      return false;
    }
  };


  c_levmar_solver()
  {
  }

  c_levmar_solver(int max_itertions, _Tp eps = (_Tp)(1e-6)) :
      max_iterations_(max_itertions),
      epsf_(eps),
      epsx_(eps)
  {
  }

  virtual ~c_levmar_solver() = default;

  void set_max_iterations(int v)
  {
    max_iterations_ = v;
  }

  int max_iterations() const
  {
    return max_iterations_;
  }

  void set_epsx(_Tp v)
  {
    epsx_ = v;
  }

  _Tp epsx() const
  {
    return epsx_;
  }

  void set_epsf(_Tp v)
  {
    epsf_ = v;
  }

  _Tp epsf() const
  {
    return epsf_;
  }

  _Tp rmse() const
  {
    return rmse_;
  }

  const std::vector<_Tp> & rhs() const
  {
    return rhs_;
  }


  virtual int run(const callback & cb, std::vector<_Tp> & params)
  {
    

    std::vector<_Tp> xd, rd, D;

    cv::Mat_<_Tp> J, A, Ap;
    cv::Mat v, temp_d, d;

    std::vector<_Tp> x =
        params;

    constexpr _Tp eps =  // DBL_EPSILON
        std::numeric_limits<_Tp>::epsilon();

    const size_t lx =
        x.size();

    if( !compute(cb, x, rhs_, &J) ) {
      CF_ERROR("compute() fails");
      return -1;
    }

    
    rmse_ =
        (_Tp)cv::norm(rhs_, cv::NORM_L2SQR);

    
    cv::mulTransposed(J, A, true);
    cv::gemm(J, cv::Mat_<_Tp>(rhs_), 1, cv::noArray(), 0, v, cv::GEMM_1_T);

    A.diag().copyTo(D);

    const _Tp Rlo = (_Tp)(0.25);
    const _Tp Rhi = (_Tp)(0.75);

    _Tp lambda = 1;
    _Tp lc = (_Tp)(0.75);

    int iteration = 0;

    while( 42 ) {

      
      A.copyTo(Ap);

      for( int i = 0; i < lx; i++ ) {
        Ap[i][i] += lambda * D[i];
      }

      
      cv::solve(Ap, v, d, cv::DECOMP_EIG);
      cv::subtract(cv::Mat_<_Tp>(x), d, xd);
      
      if( !compute(cb, xd, rd, nullptr) ) {
        CF_ERROR("compute() fails");
        return -1;
      }

      const _Tp Sd =
          cv::norm(rd, cv::NORM_L2SQR);

      cv::gemm(A, d, -1, v, 2, temp_d);

      const _Tp dS =
          d.dot(temp_d);

      const _Tp R =
          (rmse_ - Sd) / (std::abs(dS) > eps ? dS : 1);

      if( R > Rhi ) {
        lambda /= 2;
        if( lambda < lc ) {
          lambda = 0;
        }
      }
      else if( R < Rlo ) {
        // find new nu if R too low

        
        _Tp t =
            d.dot(v);

        _Tp nu =
            (Sd - rmse_) / (std::abs(t) > eps ? t : 1) + 2;

        nu = (std::min)((std::max)(nu, (_Tp)(2)), (_Tp)(10));

        if( lambda == 0 ) {

          cv::invert(A, Ap, cv::DECOMP_EIG);

          _Tp maxval = eps;

          for( int i = 0; i < lx; i++ ) {
            maxval = (std::max)(maxval, std::abs(Ap[i][i]));
          }

          lambda = lc = 1 / maxval;

          nu /= 2;
        }

        lambda *= nu;
        
      }

      const bool accepted =
          Sd < rmse_;

      if( accepted ) {

        std::swap(x, xd);
        rmse_ = Sd;

        if( std::sqrt(rmse_ / rhs_.size()) <= epsf_ ) {
          // CF_DEBUG("BREAK: std::sqrt(rmse_ / rhs_.size()) <= epsf_");
          break;
        }
      }

      
      if ( ++iteration > max_iterations_ ) {
        // CF_DEBUG("BREAK: ++iteration > max_iterations_");
        break;
      }

      
      double dp =
          cv::norm(d, cv::NORM_INF);

      if ( dp <= epsx_ ) {
        // CF_DEBUG("BREAK: dp <= epsx_: dp=%g epsx_=%g", dp, epsx_);
        break;
      }


      if ( accepted ) {

        if( !compute(cb, x, rhs_, &J) ) {
          CF_ERROR("compute() fails");
          return -1;
        }


        cv::mulTransposed(J, A, true);
        cv::gemm(J, cv::Mat_<_Tp>(rhs_), 1, cv::noArray(), 0, v, cv::GEMM_1_T);
        A.diag().copyTo(D);
      }
    }

    params = std::move(x);
    rmse_ = std::sqrt(rmse_ / (rhs_.size()-params.size()) );

    
    return iteration;
  }

protected:

  bool compute(const callback & cb, const std::vector<_Tp> & params,
      std::vector<_Tp> & rhs, cv::Mat_<_Tp> * jac) const
  {
    

    bool have_jac =
        false;

    if( !cb.compute(params, rhs, jac, &have_jac) ) {
      CF_ERROR("cb.compute() fails");
      return false;
    }

    
    if( rhs.size() < params.size() ) {
      CF_ERROR("c_levmar_solver: "
          "cb.compute() returns invalid rhs.size=%zu < params.size=%zu",
          rhs.size(), params.size());
      return false;
    }

    
    if( jac ) {

      // check if J was compute by callback
      if( have_jac ) {
        if( jac->rows != rhs.size() || jac->cols != params.size() ) {
          CF_ERROR("c_levmar_solver: "
              "Invalid Jac size %dx%d. Expected size is %zu rows x %zu columns",
              jac->rows, jac->cols, rhs.size(), params.size());
          return false;
        }
        return true;
      }

      // Compute numerical approximation of partial derivatives

  #if HAVE_TBB
      const bool use_tbb = cb.thread_safe_compute();
  #else
      const bool use_tbb = false;
  #endif

      jac->create((int)rhs.size(), (int)params.size());

      if ( !use_tbb ) {
        

        std::vector<_Tp> P = params;
        std::vector<_Tp> rhs1, rhs2;

        for( int j = 0, n = P.size(); j < n; ++j ) {

          const _Tp v =
              P[j];

          const _Tp delta =
              (std::max)((_Tp)(1e-6), (_Tp)(1e-4) * std::abs(v));

          P[j] = v + delta;
          if( !cb.compute(P, rhs1, nullptr, nullptr) ) {
            CF_ERROR("cb.compute() fails");
            return false;
          }

          if( rhs1.size() != rhs.size() ) {
            CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",
                rhs.size(), rhs1.size());
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
            (*jac)[k][j] = den * (rhs1[k] - rhs2[k]);
          }

          P[j] = v;
        }
        
      }
  #if HAVE_TBB
      else {
        

        const int rhs_size = rhs.size();

        bool fail[params.size()] = { false };


        tbb::parallel_for(0, (int) params.size(), 1,
            [&params, &jac, &cb, rhs_size, &fail](int j) {

              std::vector<_Tp> P = params;
              std::vector<_Tp> rhs1, rhs2;

              const _Tp v = P[j];

              const _Tp delta = (std::max)((_Tp)(1e-6), (_Tp)(1e-4) * std::abs(v));

              P[j] = v + delta;
              if( !cb.compute(P, rhs1, nullptr, nullptr) ) {
                fail[j] = true;
                CF_ERROR("cb.compute() fails");
                return;// false;
              }

              if( rhs1.size() != rhs_size ) {
                fail[j] = true;
                CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",
                    rhs_size, rhs1.size());
                return;// false;
              }

              P[j] = v - delta;
              if( !cb.compute(P, rhs2, nullptr, nullptr) ) {
                fail[j] = true;
                CF_ERROR("cb.compute() fails");
                return;// false;
              }

              if( rhs2.size() != rhs_size ) {
                fail[j] = true;
                CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal",
                    rhs_size, rhs1.size());
                return;// false;
              }

              const _Tp den = (_Tp)(0.5) / delta;
              for( size_t  k = 0, nk = rhs_size; k < nk; ++k ) {
                (*jac)[k][j] = den * (rhs1[k] - rhs2[k]);
              }

              P[j] = v;
            });

        for ( size_t j = 0, n = params.size(); j < n; ++j ) {
          if ( fail[j] ) {
            CF_ERROR("Compute Jac fails");
            return false;
          }
        }
        
      }
  #endif // HAVE_TBB
    }
    

    return true;
  }

protected:
  std::vector<_Tp> rhs_;
  _Tp rmse_ = -1;
  int max_iterations_ = 100;
  _Tp epsf_ = (_Tp)1e-6;
  _Tp epsx_ = (_Tp)1e-6;
};


#endif /* __c_levmar_solver_h__ */
