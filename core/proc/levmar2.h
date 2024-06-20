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
#include <limits>
#include <core/debug.h>


#if HAVE_TBB
# define LM_USE_TBB 1
# include <tbb/tbb.h>
#endif


class c_levmar2_solver
{
protected:
  double rmse_ = -1;
  double epsx_ = 1e-6;
  double epsf_ = 1e-6;
  int max_iterations_ = 100;

public:
  struct callback
  {
    virtual ~callback()
    {
    }

    virtual int num_equations() = 0;
    virtual bool set_params(const std::vector<double> & params, bool fjac) = 0;
    virtual double compute(int i, double J[/* params.size*/] = nullptr) = 0;
  };


  c_levmar2_solver()
  {
  }

  c_levmar2_solver(int max_itertions, double eps = 1e-6) :
      max_iterations_(max_itertions),
      epsf_(eps),
      epsx_(eps)
  {
  }

  virtual ~c_levmar2_solver()
  {
  }

  void set_max_iterations(int v)
  {
    max_iterations_ = v;
  }

  int max_iterations() const
  {
    return max_iterations_;
  }

  void set_epsx(double v)
  {
    epsx_ = v;
  }

  double epsx() const
  {
    return epsx_;
  }

  void set_epsf(double v)
  {
    epsf_ = v;
  }

  double epsf() const
  {
    return epsf_;
  }

  double rmse() const
  {
    return rmse_;
  }

  double compute_rhs(callback & cb, std::vector<double> & params)
  {
    cb.set_params(params, false);

#if !LM_USE_TBB
    double rms = 0;

    for( int k = 0, N = cb.num_equations(); k < N; ++k ) {

      const double rhs =
          cb.compute(k);

      rms += rhs * rhs;
    }

    return rms;

#else

    typedef tbb::blocked_range<int>
      tbb_range;

    struct Comp
    {
      callback * cb;
      const std::vector<double> * p;
      double rms;

      Comp(callback * _cb, const std::vector<double> * params) :
          cb(_cb),
          p(params),
          rms(0)
      {
      }

      Comp(const Comp & x, tbb::split) :
          cb(x.cb),
          p(x.p),
          rms(0)
      {
      }

      void operator()(const tbb_range & r)
      {
        for( int k = r.begin(); k < r.end(); ++k ) {

          const double rhs =
              cb->compute(k);

          rms += rhs * rhs;
        }
      }

      void join(const Comp & rhs)
      {
        rms += rhs.rms;
      }

    };

    Comp comp(&cb, &params);
    tbb::parallel_reduce(tbb_range(0, cb.num_equations()), comp);

    return comp.rms;
#endif
  }

  double compute_jac(callback & cb, const std::vector<double> & params,
      cv::Mat1d & A, cv::Mat1d & v)
  {
    cb.set_params(params, true);

#if !LM_USE_TBB

    const int M =
        params.size();

    double J[M];
    double rms = 0;

    A.create(M, M);
    A.setTo(0);

    v.create(M, 1);
    v.setTo(0);

    for( int k = 0, N = cb.num_equations(); k < N; ++k ) {

      const double rhs =
          cb.compute(k, J);

      rms += rhs * rhs;

      for( int i = 0; i < M; ++i ) {

        v[i][0] += J[i] * rhs;

        for( int j = 0; j <= i; ++j ) {
          A[i][j] += J[i] * J[j];
        }

      }
    }

    for( int i = 0; i < A.rows; ++i ) {
      for( int j = i + 1; j < A.cols; ++j ) {
        A[i][j] = A[j][i];
      }
    }

    return rms;
#else
    typedef tbb::blocked_range<int>
      tbb_range;

    struct Comp
    {
      callback * const cb ;
      const std::vector<double> * const p;
      cv::Mat1d A;
      cv::Mat1d v;
      double rms;

      Comp(callback * _cb, const std::vector<double> * params) :
        cb(_cb),
        p(params),
        A(params->size(), params->size(), 0.0),
        v(params->size(), 1, 0.0),
        rms(0)
      {
      }

      Comp (const Comp &x, tbb::split) :
        cb(x.cb),
        p(x.p),
        A(x.A.size(), 0.0),
        v(x.v.size(), 0.0),
        rms(0)
      {
      }

      void operator()(const tbb_range & r)
      {
        const int M =
            p->size();

        double J[M];
        double rhs;

        for( int k = r.begin(); k < r.end(); ++k ) {

          rhs = cb->compute(k, J);
          rms += rhs * rhs;

          for( int i = 0; i < M; ++i ) {

            v[i][0] += J[i] * rhs;

            for( int j = 0; j <= i; ++j ) {
              A[i][j] += J[i] * J[j];
            }

          }
        }
      }

      void join (const Comp & rhs)
      {
        A += rhs.A;
        v += rhs.v;
        rms += rhs.rms;
      }

    };

    Comp comp(&cb, &params);
    tbb::parallel_reduce(tbb_range(0, cb.num_equations()), comp);

    A = std::move(comp.A);
    v = std::move(comp.v);

    for( int i = 0; i < A.rows; ++i ) {
      for( int j = i + 1; j < A.cols; ++j ) {
        A[i][j] = A[j][i];
      }
    }

    return comp.rms;
#endif
  }

  virtual int run(callback & cb, std::vector<double> & params)
  {
    const int M =
        params.size();

    const int N =
        cb.num_equations();

    const int NM =
        N - M;

    constexpr double eps =
        std::numeric_limits<double>::epsilon();

    const double epsfn =
        epsf_ * epsf_ * NM;


    cv::Mat1d H, Hp, v, deltap, temp_d;
    std::vector<double> newparams;


    double lambda = 0.1;
    double err = 0, newerr = 0, dp = 0;

    int iteration = 0;

    while (iteration < max_iterations_) {

      err = compute_jac(cb, params, H, v);
      H.copyTo(Hp);

      /* Solve normal equation for given Jacobian and lambda */
      do {

        ++iteration;

        /* Increase diagonal elements by lambda */
        for( int i = 0; i < M; ++i ) {
          H[i][i] = (1 + lambda) * Hp[i][i];
        }

        /* Solve system to define delta and define new value of params */
        cv::solve(H, v, deltap, cv::DECOMP_EIG);
        cv::subtract(cv::Mat(params), deltap, newparams);

        /* Compute function for newparams */
        newerr = compute_rhs(cb, newparams);

        /* Check for increments in parameters  */
        if( (dp = cv::norm(deltap, cv::NORM_INF)) < epsx_ ) {
          break;
        }

        /*
         * Compute update to lambda
         * */

        cv::gemm(Hp, deltap, -1, v, 2,
            temp_d);

        const double dS =
            deltap.dot(temp_d);

        const double rho =
            (err - newerr) / (std::abs(dS) > eps ? dS : 1);

        if( rho > 0.25 ) {

          /*
           * Accept new params and decrease lambda ==> Gauss-Newton method
           * */

          err = newerr;
          std::swap(params, newparams);
          lambda = std::max(1e-4, lambda / 5);

          break;
        }

        /**
         * Try increase lambda ==> gradient descend
         * */
        if( lambda < 1 ) {
          lambda = 1;
        }
        else {
          lambda *= 10;
        }

      } while (iteration < max_iterations_);

      if( newerr < err ) {
        /*
         * Accept new params if were not yet accepted
         * */
        err = newerr;
        std::swap(params, newparams);
      }

      if( err <= epsfn || dp < epsx_ ) {
        break;
      }
    }

    rmse_ = sqrt(err / NM);

    return iteration;
  }

};



#endif /* __levmar2_h__ */
