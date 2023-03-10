/*
 * levmar.cc
 *
 *  Created on: Mar 7, 2023
 *      Author: amyznikov
 */

#include "levmar.h"
#include <core/debug.h>

c_levmar_solver::c_levmar_solver()
{

}

c_levmar_solver::c_levmar_solver(int max_itertions, double eps) :
    max_iterations_(max_itertions),
    epsf_(eps),
    epsx_(eps)
{
}




int c_levmar_solver::run(const callback & cb, std::vector<double> & params) const
{
  std::vector<double> xd, r, rd, D;

  cv::Mat1d J, A, Ap;
  cv::Mat v, temp_d, d;

  std::vector<double> x =
      params;

  const int lx =
      x.size();

  if( !compute(cb, x, r, &J) ) {
    return -1;
  }

  double S =
      cv::norm(r, cv::NORM_L2SQR);

  int nfJ = 2;

  cv::mulTransposed(J, A, true);

  cv::gemm(J, cv::Mat1d(r), 1, cv::noArray(), 0, v, cv::GEMM_1_T);

  A.diag().copyTo(D);

  const double Rlo = 0.25;
  const double Rhi = 0.75;

  double lambda = 1;
  double lc = 0.75;

  int iteration = 0;

  while( 42 ) {

    A.copyTo(Ap);

    for( int i = 0; i < lx; i++ ) {
      Ap[i][i] += lambda * D[i];
    }

    cv::solve(Ap, v, d, cv::DECOMP_EIG);

    cv::subtract(cv::Mat1d(x), d, xd);

    if( !compute(cb, xd, rd, nullptr) ) {
      return -1;
    }

    nfJ++;

    double Sd =
        cv::norm(rd, cv::NORM_L2SQR);

    cv::gemm(A, d, -1, v, 2, temp_d);

    double dS =
        d.dot(temp_d);

    double R =
        (S - Sd) / (std::abs(dS) > DBL_EPSILON ? dS : 1);

    if( R > Rhi ) {
      lambda *= 0.5;
      if( lambda < lc ) {
        lambda = 0;
      }
    }
    else if( R < Rlo ) {
      // find new nu if R too low

      double t =
          d.dot(v);

      double nu =
          (Sd - S) / (std::abs(t) > DBL_EPSILON ? t : 1) + 2;

      nu = (std::min)((std::max)(nu, 2.), 10.);

      if( lambda == 0 ) {

        cv::invert(A, Ap, cv::DECOMP_EIG);

        double maxval = DBL_EPSILON;

        for( int i = 0; i < lx; i++ ) {
          maxval = (std::max)(maxval, std::abs(Ap[i][i]));
        }

        lambda = lc = 1. / maxval;

        nu *= 0.5;
      }

      lambda *= nu;
    }

    if( Sd < S ) {

      nfJ++;
      S = Sd;
      std::swap(x, xd);

      if( !compute(cb, x, r, &J) ) {
        return -1;
      }

      cv::mulTransposed(J, A, true);
      cv::gemm(J, cv::Mat1d(r), 1, cv::noArray(), 0, v, cv::GEMM_1_T);
    }


    ++iteration;

    const bool proceed =
        iteration < max_iterations_ &&
            cv::norm(d, cv::NORM_INF) >= epsx_ &&
            cv::norm(r, cv::NORM_INF) >= epsf_;

    if( !proceed ) {
      break;
    }
  }

  params = x;

  return iteration;
}

bool c_levmar_solver::compute(const callback & cb, const std::vector<double> & params,
    std::vector<double> & rhs, cv::Mat1d * jac) const
{
  bool have_jac =
      false;

  if( !cb.compute(params, rhs, jac, &have_jac) ) {
    return false;
  }

  if( jac ) {

    // check if J was compute by callback
    if( have_jac ) {
      if( jac->rows != rhs.size() || jac->cols != params.size() ) {
        CF_ERROR("Invalid Jac size %dx%d. Expected size is %zu rows x %zu columns",
            jac->rows, jac->cols, rhs.size(), params.size());
        return false;
      }
      return true;
    }

    // Compute numerical approximation of partial derivatives


    std::vector<double> P = params;
    std::vector<double> rhs1, rhs2;

    jac->create((int)rhs.size(), (int)P.size());

    for( int j = 0, n = P.size(); j < n; ++j ) {

      const double v =
          P[j];

      const double delta =
          (std::max)(1e-6, 1e-4 * std::abs(v));

      P[j] = v + delta;
      if( !cb.compute(P, rhs1, nullptr, nullptr) ) {
        return false;
      }

      if( rhs1.size() != rhs.size() ) {
        CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",
            rhs.size(), rhs1.size());
        return false;
      }



      P[j] = v - delta;
      if( !cb.compute(P, rhs2, nullptr, nullptr) ) {
        return false;
      }

      if( rhs2.size() != rhs.size() ) {
        CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal",
            rhs.size(), rhs1.size());
        return false;
      }


      const double den = 0.5 / delta;
      for( uint k = 0, nk = rhs.size(); k < nk; ++k ) {
        (*jac)[k][j] = den * (rhs1[k] - rhs2[k]);
      }

      P[j] = v;
    }
  }

  return true;
}
