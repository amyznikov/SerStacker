/*
 * levmar2.h
 *
 *  Created on: Jun 16, 2024
 *      Author: amyznikov
 */

#pragma once
#ifndef __levmar2_h__
#define __levmar2_h__


#include <opencv2/opencv.hpp>
#include <core/debug.h>

#if HAVE_TBB
# include <tbb/tbb.h>
#endif


template<class _Tp>
class c_levmar2_solver
{
public:
  struct callback
  {
    virtual ~callback() = default;

    virtual int num_equations() = 0;
    virtual bool set_params(const std::vector<_Tp> & params, bool jac) = 0;
    virtual bool compute(int i, _Tp * rhsi, _Tp Ji[/* params.size*/] );

//    virtual _Tp compute(const std::vector<_Tp> & params, std::vector<_Tp> & rhs,
//        cv::Mat_<_Tp> * jac, bool * have_analytical_jac) const = 0;

    /* Return true if compute() can be called in parallel.
     * Can be useful to speedup numerical differentiation with central differences. */
    virtual bool thread_safe_compute() const
    {
      return false;
    }

  };


  c_levmar2_solver()
  {
  }

  c_levmar2_solver(int max_itertions, _Tp eps = (_Tp)(1e-6)) :
      max_iterations_(max_itertions),
      epsf_(eps),
      epsx_(eps)
  {
  }

  virtual ~c_levmar2_solver() = default;

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

  _Tp compute_av(callback & cb, std::vector<_Tp> & params,
      cv::Mat_<_Tp> & A, cv::Mat_<_Tp> & v)
  {
    cb.set_params(params, true);

    const int M =
        params.size();

    _Tp rhs;
    _Tp J[M];

    _Tp rms = 0;

    for( int k = 0, N = cb.num_equations(); k < N; ++k ) {

      cb.compute(k, &rhs, J);

      rms += rhs * rhs;

      for( int i = 0; i < M; ++i ) {
        for( int j = 0; j < M; ++j ) {
          A[i][j] += J[i] * J[j];
        }
        v[i][0] += J[i] * rhs;
      }
    }

    return rms;
  }

  _Tp compute_rhs(callback & cb, std::vector<_Tp> & params)
  {
    _Tp rhs;
    _Tp rms = 0;

    cb.set_params(params, false);

    for( int k = 0, N = cb.num_equations(); k < N; ++k ) {
      cb.compute(k, &rhs, nullptr);
      rms += rhs * rhs;
    }

    return rms;
  }


  virtual int run(callback & cb, std::vector<_Tp> & params)
  {
    std::vector<_Tp> xd,  D;

    std::vector<_Tp> x =
        params;

    const int M =
        x.size();

    cv::Mat_<_Tp> A(M, M, (_Tp)(0));
    cv::Mat_<_Tp> v(M, 1, (_Tp)(0));

    cv::Mat_<_Tp> J, Ap;
    cv::Mat temp_d, d;


    constexpr _Tp eps =  // DBL_EPSILON
        std::numeric_limits<_Tp>::epsilon();

    compute_av(cb, x, A, v);

//    if( (rmse_ = compute(cb, x, rhs_, &J)) < 0 ) {
//      CF_ERROR("compute() fails: rmse_ = %g", rmse_);
//      return -1;
//    }
//    cv::mulTransposed(J, A, true);
//    cv::gemm(J, cv::Mat_<_Tp>(rhs_), 1, cv::noArray(), 0, v, cv::GEMM_1_T);

    A.diag().copyTo(D);

    const _Tp Rlo = (_Tp)(0.25);
    const _Tp Rhi = (_Tp)(0.75);

    _Tp lambda = 1;
    _Tp lc = (_Tp)(0.75);

    int iteration = 0;

    while( 42 ) {

      A.copyTo(Ap);

      for( int i = 0; i < M; i++ ) {
        Ap[i][i] += lambda * D[i];
      }

      cv::solve(Ap, v, d, cv::DECOMP_EIG);

      cv::subtract(cv::Mat_<_Tp>(x), d, xd);

      const _Tp Sd =
          compute_rhs(cb, xd);

      if( Sd < 0 ) {
        CF_ERROR("compute() fails: Sd=%g", Sd);
        return -1;
      }

      cv::gemm(A, d, -1, v, 2, temp_d);

      _Tp dS =
          d.dot(temp_d);

      _Tp R =
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

          for( int i = 0; i < M; i++ ) {
            maxval = (std::max)(maxval, std::abs(Ap[i][i]));
          }

          lambda = lc = 1 / maxval;

          nu /= 2;
        }

        lambda *= nu;
      }

      if( Sd < rmse_ ) {

        rmse_ = Sd;
        std::swap(x, xd);

        compute_av(cb, x, A, v);

//        if( compute(cb, x, rhs_, &J) < 0 ) {
//          CF_ERROR("compute() fails");
//          return -1;
//        }
//
//        cv::mulTransposed(J, A, true);
//        cv::gemm(J, cv::Mat_<_Tp>(rhs_), 1, cv::noArray(), 0, v, cv::GEMM_1_T);
      }


      ++iteration;

      // XXX: Check if converged
      //      const bool proceed =
      //          iteration < max_iterations_ &&
      //              (_Tp)cv::norm(d, cv::NORM_INF) >= epsx_ &&
      //              (_Tp)cv::norm(rhs_, cv::NORM_INF) >= epsf_;
      //
      //      if( !proceed ) {
      //        break;
      //      }
    }

    params = x;
    //rmse_ = std::sqrt(rmse_ / rhs_.size());

    return iteration;
  }



protected:
//
//  _Tp compute(const callback & cb, const std::vector<_Tp> & params,
//      std::vector<_Tp> & rhs, cv::Mat_<_Tp> * jac) const
//  {
//
//    bool have_jac =
//        false;
//
//    const _Tp rms =
//        cb.compute(params, rhs, jac,
//            &have_jac);
//
//    if( rms < 0 ) {
//      CF_ERROR("cb.compute() fails: rhs2=%g", rms);
//      return false;
//    }
//
//    if( rhs.size() < params.size() ) {
//      CF_ERROR("c_levmar2_solver: "
//          "cb.compute() returns invalid rhs.size=%zu < params.size=%zu",
//          rhs.size(), params.size());
//      return false;
//    }
//
//    if( jac ) {
//
//      // check if J was compute by callback
//      if( have_jac ) {
//        if( jac->rows != rhs.size() || jac->cols != params.size() ) {
//          CF_ERROR("c_levmar2_solver: "
//              "Invalid Jac size %dx%d. Expected size is %zu rows x %zu columns",
//              jac->rows, jac->cols, rhs.size(), params.size());
//          return false;
//        }
//        return true;
//      }
//
//      // Compute numerical approximation of partial derivatives
//
//  #if HAVE_TBB
//      const bool use_tbb = cb.thread_safe_compute();
//  #else
//      const bool use_tbb = false;
//  #endif
//
//      jac->create((int)rhs.size(), (int)params.size());
//
//      if ( !use_tbb ) {
//
//        std::vector<_Tp> P = params;
//        std::vector<_Tp> rhs1, rhs2;
//
//        for( int j = 0, n = P.size(); j < n; ++j ) {
//
//          const _Tp v =
//              P[j];
//
//          const _Tp delta =
//              (std::max)((_Tp)(1e-6), (_Tp)(1e-4) * std::abs(v));
//
//          P[j] = v + delta;
//          if( cb.compute(P, rhs1, nullptr, nullptr) < 0 ) {
//            CF_ERROR("cb.compute() fails");
//            return -1;
//          }
//
//          if( rhs1.size() != rhs.size() ) {
//            CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",
//                rhs.size(), rhs1.size());
//            return false;
//          }
//
//          P[j] = v - delta;
//          if( cb.compute(P, rhs2, nullptr, nullptr) < 0 ) {
//            CF_ERROR("cb.compute() fails");
//            return -1;
//          }
//
//          if( rhs2.size() != rhs.size() ) {
//            CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal",
//                rhs.size(), rhs1.size());
//            return false;
//          }
//
//
//          const _Tp den = (_Tp) (0.5) / delta;
//          for( size_t k = 0, nk = rhs.size(); k < nk; ++k ) {
//            (*jac)[k][j] = den * (rhs1[k] - rhs2[k]);
//          }
//
//          P[j] = v;
//        }
//      }
//  #if HAVE_TBB
//      else {
//
//        const int rhs_size = rhs.size();
//
//        bool fail[params.size()] = { false };
//
//
//        tbb::parallel_for(0, (int) params.size(), 1,
//            [&params, &jac, &cb, rhs_size, &fail](int j) {
//
//              std::vector<_Tp> P = params;
//              std::vector<_Tp> rhs1, rhs2;
//
//              const _Tp v = P[j];
//
//              const _Tp delta = (std::max)((_Tp)(1e-6), (_Tp)(1e-4) * std::abs(v));
//
//              P[j] = v + delta;
//              if( cb.compute(P, rhs1, nullptr, nullptr) < 0 ) {
//                fail[j] = true;
//                CF_ERROR("cb.compute() fails");
//                return;// false;
//              }
//
//              if( rhs1.size() != rhs_size ) {
//                fail[j] = true;
//                CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",
//                    rhs_size, rhs1.size());
//                return;// false;
//              }
//
//              P[j] = v - delta;
//              if( cb.compute(P, rhs2, nullptr, nullptr) < 0 ) {
//                fail[j] = true;
//                CF_ERROR("cb.compute() fails");
//                return;// false;
//              }
//
//              if( rhs2.size() != rhs_size ) {
//                fail[j] = true;
//                CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal",
//                    rhs_size, rhs1.size());
//                return;// false;
//              }
//
//              const _Tp den = (_Tp)(0.5) / delta;
//              for( size_t  k = 0, nk = rhs_size; k < nk; ++k ) {
//                (*jac)[k][j] = den * (rhs1[k] - rhs2[k]);
//              }
//
//              P[j] = v;
//            });
//
//        for ( size_t j = 0, n = params.size(); j < n; ++j ) {
//          if ( fail[j] ) {
//            CF_ERROR("Compute Jac fails");
//            return -1;
//          }
//        }
//      }
//  #endif // HAVE_TBB
//    }
//
//    return rms;
//  }

protected:
  //std::vector<_Tp> rhs_;
  _Tp rmse_ = -1;
  int max_iterations_ = 100;
  _Tp epsf_ = (_Tp)1e-6;
  _Tp epsx_ = (_Tp)1e-6;
};



#endif /* __levmar2_h__ */
