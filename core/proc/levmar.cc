/*
 * levmar3.cc
 *
 *  Created on: Aug 17, 2024
 *      Author: amyznikov
 */

#include <limits>
#include "levmar.h"

#if HAVE_TBB

# include <tbb/tbb.h>

typedef tbb::blocked_range<int>
  tbb_range;

#endif


#include <core/debug.h>

c_levmar_solver::c_levmar_solver()
{
}

c_levmar_solver::c_levmar_solver(int max_itertions, double eps) :
    _max_iterations(max_itertions),
    _epsfn(eps),
    _epsx(eps)
{
}

void c_levmar_solver::set_max_iterations(int v)
{
  _max_iterations = v;
}

int c_levmar_solver::max_iterations() const
{
  return _max_iterations;
}

void c_levmar_solver::set_epsx(double v)
{
  _epsx = v;
}

double c_levmar_solver::epsx() const
{
  return _epsx;
}

void c_levmar_solver::set_epsfn(double v)
{
  _epsfn = v;
}

double c_levmar_solver::epsfn() const
{
  return _epsfn;
}

void c_levmar_solver::set_update_step_scale(double v)
{
  _update_step_scale = v;
}

double c_levmar_solver::update_step_scale() const
{
  return _update_step_scale;
}

void c_levmar_solver::set_initial_lambda(double v)
{
  _initial_lambda = v;
}

double c_levmar_solver::initial_lambda() const
{
  return _initial_lambda;
}

void c_levmar_solver::set_decomp_type(cv::DecompTypes v)
{
  _decomp_type = v;
}

cv::DecompTypes c_levmar_solver::decomp_type() const
{
  return _decomp_type;
}

double c_levmar_solver::rmse() const
{
  return _rmse;
}

const std::vector<double>& c_levmar_solver::rhs() const
{
  return _rhs;
}

int c_levmar_solver::run(callback & cb, std::vector<double> & params)
{
  INSTRUMENT_REGION("");

  const int M =
      params.size();

  constexpr double machine_eps =
      std::numeric_limits<double>::epsilon();

  double lambda = _initial_lambda;

  double err = 0, newerr = 0, dp = 0;

  cv::Mat1d H, Hp, v, deltap, temp_d;
  std::vector<double> newparams;

  //  CF_DEBUG("\n---------------------------------------------");

  //  CF_DEBUG("initial parameters: T={\n"
  //      "%+g %+g\n"
  //      "}\n",
  //      params[0][0],
  //      params[1][0]
  //      );

  //  CF_DEBUG("initial parameters: T={\n"
  //      "%+g %+g %+g\n"
  //      "%+g %+g %+g\n"
  //      "}\n",
  //      params[0][0],
  //      params[1][0],
  //      params[2][0],
  //      params[3][0],
  //      params[4][0],
  //      params[5][0]
  //  );

  //  CF_DEBUG("initial parameters: T={\n"
  //      "%+g %+g %+g\n"
  //      "%+g %+g %+g\n"
  //      "%+g %+g\n"
  //      "}\n",
  //      params[0][0],
  //      params[1][0],
  //      params[2][0],
  //      params[3][0],
  //      params[4][0],
  //      params[5][0],
  //      params[6][0],
  //      params[7][0]
  //  );

  _iteration = 0;

  while (_iteration < _max_iterations) {

    //CF_DEBUG("> IT %d model_->compute_jac()", iteration);

    err =
        compute_hessian(cb, params, H, v);

    //    CF_DEBUG("* JJ > IT %d lambda=%g err=%g\n",
    //        iteration,
    //        lambda,
    //        err);

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

//         CF_DEBUG("IT %d H: {\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//             "}\n",
//             _iteration,
//             H[0][0], H[0][1], H[0][2], H[0][3], H[0][4], H[0][5], H[0][6], H[0][7],
//             H[1][0], H[1][1], H[1][2], H[1][3], H[1][4], H[1][5], H[1][6], H[1][7],
//             H[2][0], H[2][1], H[2][2], H[2][3], H[2][4], H[2][5], H[2][6], H[2][7],
//             H[3][0], H[3][1], H[3][2], H[3][3], H[3][4], H[3][5], H[3][6], H[3][7],
//             H[4][0], H[4][1], H[4][2], H[4][3], H[4][4], H[4][5], H[4][6], H[4][7],
//             H[5][0], H[5][1], H[5][2], H[5][3], H[5][4], H[5][5], H[5][6], H[5][7],
//             H[6][0], H[6][1], H[6][2], H[6][3], H[6][4], H[6][5], H[6][6], H[6][7],
//             H[7][0], H[7][1], H[7][2], H[7][3], H[7][4], H[7][5], H[7][6], H[7][7]
//         );

//        CF_DEBUG("IT %d v: {\n"
//            "%+12g %+12g %+12g %+12g %+12g %+12g %+12g %+12g\n"
//            "}\n",
//            _iteration,
//            v(0,0),v(1,0),v(2,0),v(3,0),v(4,0),v(5,0),v(6,0),v(7,0)
//            );

      /* Solve system to define delta and define new value of params */
      if( !cv::solve(H, v, deltap, _decomp_type) ) {
        CF_ERROR("cv::solve() fails");
        break;
      }

      //deltap = H.inv() * v;

      cv::scaleAdd(deltap, -_update_step_scale, cv::Mat(params), newparams);

//         CF_DEBUG("IT %d Compute function for newparams: \n"
//             "deltap = { \n"
//             "  %+20g %+20g %+20g %+20g %+20g %+20g %+20g %+20g\n"
//             "}\n"
//             "newparams = {\n"
//             "  %+20g %+20g\n"
//             "}"
//             "\n",
//             _iteration,
//             deltap[0][0], deltap[1][0],deltap[2][0],deltap[3][0],deltap[4][0],deltap[5][0],deltap[6][0],deltap[7][0],
//             newparams[0], newparams[1]);

      /* Compute function for newparams */
      newerr =
          compute_rhs(cb, newparams);

      /* Check for increments in parameters  */
      if( (dp = cv::norm(deltap, cv::NORM_INF)) < _epsx ) {
        CF_DEBUG("BREAK by eps = %g / %g ", dp, _epsx);
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

      //      CF_DEBUG("IT %d err=%g newerr=%g dp=%g lambda=%g rho=%+g\n",
      //          _iteration,
      //          err, newerr,
      //          dp,
      //          lambda,
      //          rho);

      if( rho > 0.25 ) {
        /* Accept new params and decrease lambda ==> Gauss-Newton method */
        if( lambda > 1e-6 ) {
          lambda = std::max(1e-6, lambda / 5);
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
      CF_DEBUG("BREAK2 by dp=%g epsx_=%g", dp, _epsx);
      break;
    }
  }

  //  CF_DEBUG("newparams: T={%+g %+g}\n",
  //      params[0][0],
  //      params[1][0]);

  _errx = dp;
  _errfn = err;
  _rmse = sqrt(err / (_rhs.size() - M));

  return _iteration;
}

double c_levmar_solver::compute_rhs(callback & cb, std::vector<double> & params)
{
  if( !compute(cb, params, _rhs, nullptr) ) {
    return -1;
  }

  return cv::norm(_rhs, cv::NORM_L2SQR);
}

double c_levmar_solver::compute_hessian(callback & cb, const std::vector<double> & params,
    cv::Mat1d & H, cv::Mat1d & v)
{
  if( !compute(cb, params, _rhs, &_J) ) {
    return -1;
  }

  const int N =
      _rhs.size();

  const int M =
      params.size();

  v.create(M, 1);

#if HAVE_TBB
  const bool use_tbb =
      cb.allow_tbb();
#else
  const bool use_tbb =
      false;
#endif

  if ( !use_tbb ) {
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

  if ( !use_tbb ) {
    for( int i = 0; i < M; ++i ) {
      for( int j = 0; j <= i; ++j ) {
        H[i][j] = _J.row(i).dot(_J.row(j));
      }
    }
  }
#if HAVE_TBB
  else  {
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

bool c_levmar_solver::compute(const callback & cb, const std::vector<double> & params,
    std::vector<double> & rhs, cv::Mat1d * J)
{

  bool have_jac =
      false;

  if( !cb.compute(params, rhs, J, &have_jac) ) {
    CF_ERROR("cb.compute() fails");
    return false;
  }

  if( rhs.size() < params.size() ) {
    CF_ERROR("c_levmar_solver: "
        "cb.compute() returns invalid rhs.size=%zu < params.size=%zu",
        rhs.size(), params.size());
    return false;
  }

  if( J ) {

    // check if J was compute by callback
    if( have_jac ) {
      if( J->cols != rhs.size() || J->rows != params.size() ) {
        CF_ERROR("c_levmar_solver: "
            "Invalid Jac size %dx%d. Expected size is %zu cols x %zu rows",
            J->cols, J->rows, rhs.size(), params.size());
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

      std::vector<double> P = params;
      std::vector<double> rhs1, rhs2;

      for( int j = 0, n = P.size(); j < n; ++j ) {

        const double v =
            P[j];

        const double delta =
            (std::max)((double) (1e-6), (double) (1e-4) * std::abs(v));

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

        const double den = (double) (0.5) / delta;
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

            std::vector<double> P = params;
            std::vector<double> rhs1, rhs2;

            const double v = P[j];

            const double delta = (std::max)((double)(1e-6), (double)(1e-4) * std::abs(v));

            P[j] = v + delta;
            if( !cb.compute(P, rhs1, nullptr, nullptr) ) {
              fail[j] = true;
              CF_ERROR("cb.compute() fails");
              return;   // false;
            }

            if( rhs1.size() != rhs_size ) {
              fail[j] = true;
              CF_ERROR("APP BUG in caller-specified callback: rhs1.size=%zu and rhs.size=%zu not equal",
                  rhs_size, rhs1.size());
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
              CF_ERROR("APP BUG in caller-specified callback: rhs2.size=%zu and rhs.size=%zu not equal",
                  rhs_size, rhs1.size());
              return;   // false;
            }

            const double den = (double)(0.5) / delta;
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
