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

  double compute_av(callback & cb, std::vector<double> & params,
      cv::Mat1d & A, cv::Mat1d & v)
  {
    cb.set_params(params, true);

#if HAVE_TBB

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

            for( int j = 0; j < M; ++j ) {
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

    return comp.rms;
#else

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
        for( int j = 0; j < M; ++j ) {
          A[i][j] += J[i] * J[j];
        }

        v[i][0] +=
            J[i] * rhs;
      }
    }

    return rms;
#endif
  }

  double compute_rhs(callback & cb, std::vector<double> & params)
  {
    cb.set_params(params, false);

#if HAVE_TBB
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
        rms(0)      {
      }

      Comp(const Comp &x, tbb::split) :
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

      void join (const Comp & rhs)
      {
        rms += rhs.rms;
      }

    };

    Comp comp(&cb, &params);
    tbb::parallel_reduce(tbb_range(0, cb.num_equations()), comp);

    return comp.rms;

#else

    double rms = 0;

    for( int k = 0, N = cb.num_equations(); k < N; ++k ) {

      const double rhs =
          cb.compute(k);

      rms += rhs * rhs;
    }

    return rms;
#endif
  }


  virtual int run(callback & cb, std::vector<double> & params)
  {
    const int M =
        params.size();

    const double Rlo = 0.25;
    const double Rhi = 0.75;

    constexpr double eps =
        std::numeric_limits<double>::epsilon();

    std::vector<double> xd,  D;
    cv::Mat1d A, Ap, v;
    cv::Mat1d d, temp_d;

    double lambda = 1;
    double lc = 0.75;

    rmse_ =
        compute_av(cb, params, A, v);

    A.diag().copyTo(D);

    int iteration = 0;

    while( 42 ) {

      A.copyTo(Ap);

      for( int i = 0; i < M; i++ ) {
        Ap[i][i] += lambda * D[i];
      }

      cv::solve(Ap, v, d, cv::DECOMP_EIG);

      cv::subtract(cv::Mat(params), d, xd);

      const double Sd =
          compute_rhs(cb, xd);

      if( Sd < 0 ) {
        CF_ERROR("compute() fails: Sd=%g", Sd);
        return -1;
      }

      cv::gemm(A, d, -1, v, 2, temp_d);

      const double dS =
          d.dot(temp_d);

      const double R =
          (rmse_ - Sd) / (std::abs(dS) > eps ? dS : 1);

      if( R > Rhi ) {
        lambda /= 2;
        if( lambda < lc ) {
          lambda = 0;
        }
      }
      else if( R < Rlo ) {
        // find new nu if R too low

        const double t =
            d.dot(v);

        double nu =
            (Sd - rmse_) / (std::abs(t) > eps ? t : 1) + 2;

        nu = (std::min)((std::max)(nu, 2.), 10.);

        if( lambda == 0 ) {

          cv::invert(A, Ap, cv::DECOMP_EIG);

          double maxval = eps;

          for( int i = 0; i < M; i++ ) {
            maxval = (std::max)(maxval, std::abs(Ap[i][i]));
          }

          lambda = lc = 1 / maxval;

          nu /= 2;
        }

        lambda *= nu;
      }

      if( Sd < rmse_ ) {

        std::swap(params, xd);

        if( std::sqrt(Sd / cb.num_equations()) <= epsf_ ) {
          rmse_ = Sd;
          break;
        }
      }

      if ( ++iteration > max_iterations_ ) {
        rmse_ = std::min(rmse_, Sd);
        break;
      }

      if ( cv::norm(d, cv::NORM_INF) <= epsx_ ) {
        rmse_ = std::min(rmse_, Sd);
        break;
      }

      if( Sd < rmse_ ) {
        rmse_ =
            compute_av(cb, params, A, v);
      }

    }

    rmse_ =
        std::sqrt(rmse_ / cb.num_equations());

    return iteration;
  }

};



#endif /* __levmar2_h__ */
