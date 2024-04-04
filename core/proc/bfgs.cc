/*
 * bfgs.cc
 *
 *  Created on: Aug 12, 2023
 *      Author: amyznikov
 *
 * Adapted from "Limited memory BFGS (L-BFGS)"
 *  <https://github.com/chokkan/liblbfgs.git>
 *
 * @sa <https://habr.com/ru/articles/333356>
 */
#include "bfgs.h"
#include <core/ssprintf.h>
#include <core/debug.h>

template<>
const c_enum_member* members_of<c_bfgs::STATUS>()
{
  static const c_enum_member members[] =
      {
          { c_bfgs::LBFGS_SUCCESS, "SUCCESS", "SUCCESS" },
          { c_bfgs::LBFGS_CONVERGENCE, "CONVERGENCE", "CONVERGENCE" },
          { c_bfgs::LBFGS_STOP, "STOP", "STOP" },
          { c_bfgs::LBFGS_ALREADY_MINIMIZED, "LBFGS_ALREADY_MINIMIZED",
              "The initial variables already minimize the objective function." },
          { c_bfgs::LBFGSERR_UNKNOWNERROR, "LBFGSERR_UNKNOWNERROR", "Unknown error" },
          { c_bfgs::LBFGSERR_LOGICERROR, "LBFGSERR_LOGICERROR", "Logic error. " },
          { c_bfgs::LBFGSERR_OUTOFMEMORY, "LBFGSERR_OUTOFMEMORY", "Insufficient memory. " },
          { c_bfgs::LBFGSERR_CANCELED, "LBFGSERR_CANCELED", "The minimization process has been canceled. " },
          { c_bfgs::LBFGSERR_INVALID_N, "LBFGSERR_INVALID_N", "Invalid number of variables specified. " },
          { c_bfgs::LBFGSERR_INVALID_N_SSE, "LBFGSERR_INVALID_N_SSE",
              "Invalid number of variables (for SSE) specified. " },
          { c_bfgs::LBFGSERR_INVALID_X_SSE, "LBFGSERR_INVALID_X_SSE", "The array x must be aligned to 16 (for SSE). " },
          { c_bfgs::LBFGSERR_INVALID_EPSILON, "LBFGSERR_INVALID_EPSILON",
              "Invalid parameter lbfgs_parameter_t::epsilon specified. " },
          { c_bfgs::LBFGSERR_INVALID_TESTPERIOD, "LBFGSERR_INVALID_TESTPERIOD",
              "Invalid parameter lbfgs_parameter_t::past specified. " },
          { c_bfgs::LBFGSERR_INVALID_DELTA, "LBFGSERR_INVALID_DELTA",
              "Invalid parameter lbfgs_parameter_t::delta specified. " },
          { c_bfgs::LBFGSERR_INVALID_LINESEARCH, "LBFGSERR_INVALID_LINESEARCH",
              "Invalid parameter lbfgs_parameter_t::linesearch specified. " },
          { c_bfgs::LBFGSERR_INVALID_MINSTEP, "LBFGSERR_INVALID_MINSTEP",
              "Invalid parameter lbfgs_parameter_t::max_step specified." },
          { c_bfgs::LBFGSERR_INVALID_MAXSTEP, "LBFGSERR_INVALID_MAXSTEP",
              "Invalid parameter lbfgs_parameter_t::max_step specified." },
          { c_bfgs::LBFGSERR_INVALID_FTOL, "LBFGSERR_INVALID_FTOL",
              "Invalid parameter lbfgs_parameter_t::ftol specified. " },
          { c_bfgs::LBFGSERR_INVALID_WOLFE, "LBFGSERR_INVALID_WOLFE",
              "Invalid parameter lbfgs_parameter_t::wolfe specified. " },
          { c_bfgs::LBFGSERR_INVALID_GTOL, "LBFGSERR_INVALID_GTOL",
              "Invalid parameter lbfgs_parameter_t::gtol specified. " },
          { c_bfgs::LBFGSERR_INVALID_XTOL, "LBFGSERR_INVALID_XTOL",
              "Invalid parameter lbfgs_parameter_t::xtol specified. " },
          { c_bfgs::LBFGSERR_INVALID_MAXLINESEARCH, "LBFGSERR_INVALID_MAXLINESEARCH",
              "Invalid parameter lbfgs_parameter_t::max_linesearch specified. " },
          { c_bfgs::LBFGSERR_INVALID_ORTHANTWISE, "LBFGSERR_INVALID_ORTHANTWISE",
              "Invalid parameter lbfgs_parameter_t::orthantwise_c specified. " },
          { c_bfgs::LBFGSERR_INVALID_ORTHANTWISE_START, "LBFGSERR_INVALID_ORTHANTWISE_START",
              "Invalid parameter lbfgs_parameter_t::orthantwise_start specified. " },
          { c_bfgs::LBFGSERR_INVALID_ORTHANTWISE_END, "LBFGSERR_INVALID_ORTHANTWISE_END",
              "Invalid parameter lbfgs_parameter_t::orthantwise_end specified. " },
          { c_bfgs::LBFGSERR_OUTOFINTERVAL, "LBFGSERR_OUTOFINTERVAL",
              "The line-search step went out of the interval of uncertainty. " },
          { c_bfgs::LBFGSERR_INCORRECT_TMINMAX, "LBFGSERR_INCORRECT_TMINMAX",
              "A logic error occurred; alternatively, the interval of uncertainty became too small." },
          { c_bfgs::LBFGSERR_ROUNDING_ERROR, "LBFGSERR_ROUNDING_ERROR",
              "A rounding error occurred; alternatively, no line-search step satisfies the sufficient decrease and curvature conditions." },
          { c_bfgs::LBFGSERR_MINIMUMSTEP, "LBFGSERR_MINIMUMSTEP",
              "The line-search step became smaller than lbfgs_parameter_t::min_step. " },
          { c_bfgs::LBFGSERR_MAXIMUMSTEP, "LBFGSERR_MAXIMUMSTEP",
              "The line-search step became larger than lbfgs_parameter_t::max_step. " },
          { c_bfgs::LBFGSERR_MAXIMUMLINESEARCH, "LBFGSERR_MAXIMUMLINESEARCH",
              "The line-search routine reaches the maximum number of evaluations. " },
          { c_bfgs::LBFGSERR_MAXIMUMITERATION, "LBFGSERR_MAXIMUMITERATION",
              "The algorithm routine reaches the maximum number of iterations. " },
          { c_bfgs::LBFGSERR_WIDTHTOOSMALL, "LBFGSERR_WIDTHTOOSMALL",
              "Relative width of the interval of uncertainty is at most lbfgs_parameter_t::xtol. " },
          { c_bfgs::LBFGSERR_INVALIDPARAMETERS, "LBFGSERR_INVALIDPARAMETERS",
              "A logic error (negative line-search step) occurred. " },
          { c_bfgs::LBFGSERR_INCREASEGRADIENT, "LBFGSERR_INCREASEGRADIENT",
              "The current search direction increases the objective function value. " },
          { c_bfgs::LBFGSERR_UNKNOWNERROR },
      };

  return members;
}

//static inline double SQR(double x)
//{
//  return x * x;
//}

static inline double max3(double a, double b, double c)
{
  return (std::max)((std::max(a, b)), c);
}

static inline void veccpy(std::vector<double> & dst, const std::vector<double> & src)
{
  dst = src;
}

static inline void vecncpy(std::vector<double> & dst, const std::vector<double> & src)
{
  for( int i = 0, n = dst.size(); i < n; ++i ) {
    dst[i] = -src[i];
  }
}

static inline void vecadd(std::vector<double> & y, const std::vector<double> & x, const double c)
{
  for( int i = 0, n = y.size(); i < n; ++i ) {
    y[i] += c * x[i];
  }
}

static inline void vecdiff(std::vector<double> & z, const std::vector<double> & x, const std::vector<double> & y)
{
  const int n = x.size();

  if( (int) z.size() != n ) {
    z.resize(n);
  }

  for( int i = 0; i < n; ++i ) {
    z[i] = x[i] - y[i];
  }
}

static inline double vecdot(const std::vector<double> & x, const std::vector<double> & y)
{
  double s = 0;
  for( int i = 0, n = x.size(); i < n; ++i ) {
    s += x[i] * y[i];
  }
  return s;
}

static inline double vec2norm(const std::vector<double> & x)
{
  return sqrt(vecdot(x, x));
}

static inline void vecscale(std::vector<double> & y, const double c)
{
  for( int i = 0, n = y.size(); i < n; ++i ) {
    y[i] *= c;
  }
}

static double owlqn_x1norm(const std::vector<double> & x, const int start, const int n)
{
  double norm = 0.;

  for( int i = start; i < n; ++i ) {
    norm += std::abs(x[i]);
  }

  return norm;
}

static void owlqn_pseudo_gradient(std::vector<double> & pg,
    const std::vector<double> & x,
    const std::vector<double> & g,
    const int n,
    const double c,
    const int start,
    const int end)
{
  int i;

  /* Compute the negative of gradients. */
  for( i = 0; i < start; ++i ) {
    pg[i] = g[i];
  }

  /* Compute the psuedo-gradients. */
  for( i = start; i < end; ++i ) {
    if( x[i] < 0. ) {
      /* Differentiable. */
      pg[i] = g[i] - c;
    }
    else if( 0. < x[i] ) {
      /* Differentiable. */
      pg[i] = g[i] + c;
    }
    else {
      if( g[i] < -c ) {
        /* Take the right partial derivative. */
        pg[i] = g[i] + c;
      }
      else if( c < g[i] ) {
        /* Take the left partial derivative. */
        pg[i] = g[i] - c;
      }
      else {
        pg[i] = 0.;
      }
    }
  }

  for( i = end; i < n; ++i ) {
    pg[i] = g[i];
  }
}

static void owlqn_project(std::vector<double> & d, const std::vector<double> & sign,
    const int start, const int end)
{
  for( int i = start; i < end; ++i ) {
    if( d[i] * sign[i] <= 0 ) {
      d[i] = 0;
    }
  }
}

c_bfgs::c_bfgs(int max_iterations) :
    max_iterations_(max_iterations)
{
}

c_bfgs::~c_bfgs()
{
}

int c_bfgs::m() const
{
  return m_;
}

void c_bfgs::set_m(int v)
{
  m_ = v;
}

double c_bfgs::epsilon() const
{
  return epsilon_;
}

void c_bfgs::set_epsilon(double v)
{
  epsilon_ = v;
}

int c_bfgs::past() const
{
  return past_;
}

void c_bfgs::set_past(int v)
{
  past_ = v;
}

double c_bfgs::delta() const
{
  return delta_;
}

void c_bfgs::set_delta(double v)
{
  delta_ = v;
}

int c_bfgs::max_iterations() const
{
  return max_iterations_;
}

void c_bfgs::set_max_iterations(int v)
{
  max_iterations_ = v;
}

c_bfgs::LINESEARCH c_bfgs::linesearch() const
{
  return linesearch_;
}

void c_bfgs::set_linesearch(LINESEARCH v)
{
  linesearch_ = v;
}

int c_bfgs::max_linesearch() const
{
  return max_linesearch_;
}

void c_bfgs::set_max_linesearch(int v)
{
  max_linesearch_ = v;
}

double c_bfgs::min_step() const
{
  return min_step_;
}

void c_bfgs::set_min_step(double v)
{
  min_step_ = v;
}

double c_bfgs::max_step() const
{
  return max_step_;
}

void c_bfgs::set_max_step(double v)
{
  max_step_ = v;
}

double c_bfgs::ftol() const
{
  return ftol_;
}

void c_bfgs::set_ftol(double v)
{
  ftol_ = v;
}

double c_bfgs::wolfe() const
{
  return wolfe_;
}

void c_bfgs::set_wolfe(double v)
{
  wolfe_ = v;
}

double c_bfgs::gtol() const
{
  return gtol_;
}

void c_bfgs::set_gtol(double v)
{
  gtol_ = v;
}

double c_bfgs::xtol() const
{
  return xtol_;
}

void c_bfgs::set_xtol(double v)
{
  xtol_ = v;
}

double c_bfgs::orthantwise_c() const
{
  return orthantwise_c_;
}

void c_bfgs::set_orthantwise_c(double v)
{
  orthantwise_c_ = v;
}

int c_bfgs::orthantwise_start() const
{
  return orthantwise_start_;
}

void c_bfgs::set_orthantwise_start(double v)
{
  orthantwise_start_ = v;
}

int c_bfgs::orthantwise_end() const
{
  return orthantwise_end_;
}

void c_bfgs::set_orthantwise_end(double v)
{
  orthantwise_end_ = v;
}


c_bfgs::STATUS c_bfgs::status() const
{
  return status_;
}

int c_bfgs::iterations() const
{
  return num_iterations_;
}

bool c_bfgs::compute(const callback & cb, const std::vector<double> & p,
    double * f, std::vector<double> * g) const
{

  bool have_g = false;

  if( !cb.compute(p, f, g, &have_g) ) {
    CF_ERROR("cb.compute() fails");
    return false;
  }

  if( g ) {

    const int n = p.size();

    if( have_g ) {

      if( g->size() != n ) {
        CF_ERROR("cb.compute() retins invalid gradient vector size %zu. Expected %zu",
            g->size(), p.size());
        return false;
      }

    }
    else {

      std::vector<double> P = p;

      double f1, f2;

      g->resize(n);

      for( int j = 0; j < n; ++j ) {

        const double v = P[j];
        const double delta = (std::max)(1e-6, 1e-4 * std::abs(v));

        P[j] = v + delta;
        if( !cb.compute(P, &f1, nullptr, nullptr) ) {
          CF_ERROR("cb.compute() fails");
          return false;
        }

        P[j] = v - delta;
        if( !cb.compute(P, &f2, nullptr, nullptr) ) {
          CF_ERROR("cb.compute() fails");
          return false;
        }

        (*g)[j] = 0.5 * (f1 - f2) / delta;
      }
    }
  }

  return true;
}

c_bfgs::STATUS c_bfgs::run(const callback & cb, std::vector<double> & x)
{

  typedef STATUS (this_class::*line_search_proc)(const callback & cb,
      std::vector<double> & x, /* [n] */
      double * f,
      std::vector<double> & g, /* [n] */
      const std::vector<double> & s, /* [n] */
      double * stp,
      const std::vector<double> & xp, /* [n] */
      const std::vector<double> & gp, /* [n] */
      std::vector<double> & wa /* [n] */);

  struct iteration_data_t {
    double alpha;
    double ys; /* vecdot(y, s) */
    std::vector<double> s; /* [n] */
    std::vector<double> y; /* [n] */
  };

  int i, j, end, bound;
  double step;

  const int m = this->m_;

  double ys, yy;
  double xnorm, gnorm, beta;
  double fx = 0.;
  double rate = 0.;

  line_search_proc linesearch =
      &this_class::line_search_morethuente;

  const int n =
      x.size();

  num_iterations_ = 0;

  /* Check the input parameters for errors. */
  if( n <= 0 ) {
    CF_ERROR("Invalid number of adjustable parameters: n=%d", n);
    return (status_ = LBFGSERR_INVALID_N);
  }
  if( epsilon_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_EPSILON");
    return (status_ = LBFGSERR_INVALID_EPSILON);
  }
  if( past_ < 0 ) {
    CF_ERROR("LBFGSERR_INVALID_TESTPERIOD");
    return (status_ = LBFGSERR_INVALID_TESTPERIOD);
  }
  if( delta_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_DELTA");
    return (status_ = LBFGSERR_INVALID_DELTA);
  }
  if( min_step_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_MINSTEP");
    return (status_ = LBFGSERR_INVALID_MINSTEP);
  }
  if( max_step_ < min_step_ ) {
    CF_ERROR("LBFGSERR_INVALID_MAXSTEP");
    return (status_ = LBFGSERR_INVALID_MAXSTEP);
  }
  if( ftol_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_FTOL");
    return (status_ = LBFGSERR_INVALID_FTOL);
  }
  if( linesearch_ == LBFGS_LINESEARCH_BACKTRACKING_WOLFE ||
      linesearch_ == LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE ) {
    if( wolfe_ <= ftol_ || 1. <= wolfe_ ) {
      CF_ERROR("LBFGSERR_INVALID_WOLFE");
      return (status_ = LBFGSERR_INVALID_WOLFE);
    }
  }
  if( gtol_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_GTOL");
    return (status_ = LBFGSERR_INVALID_GTOL);
  }
  if( xtol_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_XTOL");
    return (status_ = LBFGSERR_INVALID_XTOL);
  }
  if( max_linesearch_ <= 0 ) {
    CF_ERROR("LBFGSERR_INVALID_MAXLINESEARCH");
    return (status_ = LBFGSERR_INVALID_MAXLINESEARCH);
  }
  if( orthantwise_c_ < 0. ) {
    CF_ERROR("LBFGSERR_INVALID_ORTHANTWISE");
    return (status_ = LBFGSERR_INVALID_ORTHANTWISE);
  }
  if( orthantwise_start_ < 0 || n < orthantwise_start_ ) {
    CF_ERROR("LBFGSERR_INVALID_ORTHANTWISE_START");
    return (status_ = LBFGSERR_INVALID_ORTHANTWISE_START);
  }
  if( orthantwise_end_ < 0 ) {
    orthantwise_end_ = n;
  }
  if( n < orthantwise_end_ ) {
    return (status_ = LBFGSERR_INVALID_ORTHANTWISE_END);
  }
  if( orthantwise_c_ != 0. ) {
    switch (linesearch_) {
      case LBFGS_LINESEARCH_BACKTRACKING:
        linesearch = &this_class::line_search_backtracking_owlqn;
        break;
      default:
        /* Only the backtracking method is available. */
        CF_ERROR("LBFGSERR_INVALID_LINESEARCH");
        return (status_ = LBFGSERR_INVALID_LINESEARCH);
    }
  }
  else {
    switch (linesearch_) {
      case LBFGS_LINESEARCH_MORETHUENTE:
        linesearch = &this_class::line_search_morethuente;
        break;
      case LBFGS_LINESEARCH_BACKTRACKING_ARMIJO:
        case LBFGS_LINESEARCH_BACKTRACKING_WOLFE:
        case LBFGS_LINESEARCH_BACKTRACKING_STRONG_WOLFE:
        linesearch = &this_class::line_search_backtracking;
        break;
      default:
        CF_ERROR("LBFGSERR_INVALID_LINESEARCH");
        return (status_ = LBFGSERR_INVALID_LINESEARCH);
    }
  }

  /* Allocate working space. */
  std::vector<double> xp(n);
  std::vector<double> g(n);
  std::vector<double> gp(n);
  std::vector<double> d(n);
  std::vector<double> w(n);

  std::vector<double> pg;
  std::vector<double> pf;

  /* Allocate working space for OW-LQN. */
  if( orthantwise_c_ != 0. ) {
    pg.resize(n);
  }

  /* Allocate an array for storing previous values of the objective function. */
  if( past_ > 0 ) {
    pf.resize(past_);
  }

  /* Allocate limited memory storage. */
  std::vector<iteration_data_t> lm(m);
  iteration_data_t *it = nullptr;

  /* Initialize the limited memory. */
  for( i = 0; i < m; ++i ) {
    it = &lm[i];
    it->alpha = 0;
    it->ys = 0;
    it->s.resize(n);
    it->y.resize(n);
  }

  /* Evaluate the function value and its gradient. */
  // fx = cd.proc_evaluate(cd.instance, x, g, cd.n, 0);
  if( !compute(cb, x, &fx, &g) ) {
    CF_ERROR("compute() fails");
    return (status_ = LBFGSERR_UNKNOWNERROR);
  }

  if( orthantwise_c_ != 0 ) {
    /* Compute the L1 norm of the variable and add it to the object value. */
    xnorm = owlqn_x1norm(x, orthantwise_start_, orthantwise_end_);
    fx += xnorm * orthantwise_c_;
    owlqn_pseudo_gradient(pg, x, g, n, orthantwise_c_, orthantwise_start_, orthantwise_end_);
  }

  /* Store the initial value of the objective function. */
  if( !pf.empty() ) {
    pf[0] = fx;
  }

  /* Compute the direction;
   * we assume the initial hessian matrix H_0 as the identity matrix.
   * */
  if( orthantwise_c_ == 0 ) {
    vecncpy(d, g);
  }
  else {
    vecncpy(d, pg);
  }

  /*
   *  Make sure that the initial variables are not a minimizer.
   */
  xnorm = vec2norm(x);
  if( orthantwise_c_ == 0 ) {
    gnorm = vec2norm(g);
  }
  else {
    gnorm = vec2norm(pg);
  }

  if( xnorm < 1 ) {
    xnorm = 1;
  }

  if( gnorm / xnorm <= epsilon_ ) {
    CF_DEBUG("LBFGS_ALREADY_MINIMIZED");
    return (status_ = LBFGS_ALREADY_MINIMIZED);
  }

  /* Compute the initial step:
   *   step = 1.0 / sqrt(vecdot(d, d, n))
   */
  step = 1.0 / vec2norm(d);
  end = 0;

  int & k = num_iterations_;
  for( k = 1;; ) {
    /* Store the current position and gradient vectors. */
    veccpy(xp, x);
    veccpy(gp, g);

    /* Search for an optimal step. */
    if( orthantwise_c_ == 0 ) {
      if( (status_ = (this->*linesearch)(cb, x, &fx, g, d, &step, xp, gp, w)) < 0 ) {
        CF_ERROR("linesearch() fails: status_=%s", toCString(status_));
        return (status_);
      }
    }
    else {
      if( (status_ = (this->*linesearch)(cb, x, &fx, g, d, &step, xp, pg, w)) < 0 ) {
        CF_ERROR("linesearch() fails: status_=%s", toCString(status_));
        return (status_);
      }
      owlqn_pseudo_gradient(pg, x, g, n, orthantwise_c_, orthantwise_start_, orthantwise_end_);
    }

    /* Compute x and g norms. */
    xnorm = vec2norm(x);
    if( orthantwise_c_ == 0. ) {
      gnorm = vec2norm(g);
    }
    else {
      gnorm = vec2norm(pg);
    }

    /* Report the progress. */
//    if( cd.proc_progress ) {
//      if( (ret = cd.proc_progress(cd.instance, x, g, fx, xnorm, gnorm, step, cd.n, k, ls)) ) {
//        goto lbfgs_exit;
//      }
//    }
    /*
     Convergence test.
     The criterion is given by the following formula:
     |g(x)| / \max(1, |x|) < \epsilon
     */
    if( xnorm < 1 ) {
      xnorm = 1;
    }

    if( gnorm / xnorm <= epsilon_ ) { /* Convergence. */
      status_ = LBFGS_SUCCESS;
      break;
    }

    /*
     * Test for stopping criterion.
     * The criterion is given by the following formula:
     * |(f(past_x) - f(x))| / f(x) < \delta
     */
    if( !pf.empty() ) { /* We don't test the stopping criterion while k < past. */
      if( past_ <= k ) { /* Compute the relative improvement from the past. */

        rate = (pf[k % past_] - fx) / fx;

        if( fabs(rate) < delta_ ) { /* The stopping criterion. */
          status_ = LBFGS_STOP;
          break;
        }
      }

      /* Store the current value of the objective function. */
      pf[k % past_] = fx;
    }

    if( max_iterations_ > 0 && max_iterations_ < k + 1 ) { /* Maximum number of iterations. */
      status_ = LBFGSERR_MAXIMUMITERATION;
      break;
    }

    /*
     *   Update vectors s and y:
     *       s_{k+1} = x_{k+1} - x_{k} = \step * d_{k}.
     *       y_{k+1} = g_{k+1} - g_{k}.
     */
    it = &lm[end];
    vecdiff(it->s, x, xp);
    vecdiff(it->y, g, gp);

    /*
     *   Compute scalars ys and yy:
     *       ys = y^t \cdot s = 1 / \rho.
     *       yy = y^t \cdot y.
     *   Notice that yy is used for scaling the hessian matrix H_0 (Cholesky factor).
     */
    ys = vecdot(it->y, it->s);
    yy = vecdot(it->y, it->y);
    it->ys = ys;

    /*
     Recursive formula to compute dir = -(H \cdot g).
     This is described in page 779 of:
     Jorge Nocedal.
     Updating Quasi-Newton Matrices with Limited Storage.
     Mathematics of Computation, Vol. 35, No. 151,
     pp. 773--782, 1980.
     */
    bound = (m <= k) ? m : k;
    ++k;
    end = (end + 1) % m;

    /* Compute the steepest direction. */
    if( orthantwise_c_ == 0 ) { /* Compute the negative of gradients. */
      vecncpy(d, g);
    }
    else {
      vecncpy(d, pg);
    }

    j = end;

    for( i = 0; i < bound; ++i ) {
      j = (j + m - 1) % m; /* if (--j == -1) j = m-1; */
      it = &lm[j];
      /* \alpha_{j} = \rho_{j} s^{t}_{j} \cdot q_{k+1}. */
      it->alpha = vecdot(it->s, d);
      it->alpha /= it->ys;
      /* q_{i} = q_{i+1} - \alpha_{i} y_{i}. */
      vecadd(d, it->y, -it->alpha);
    }

    vecscale(d, ys / yy);

    for( i = 0; i < bound; ++i ) {
      it = &lm[j];
      /* \beta_{j} = \rho_{j} y^t_{j} \cdot \gamma_{i}. */
      beta = vecdot(it->y, d) / it->ys;
      /* \gamma_{i+1} = \gamma_{i} + (\alpha_{j} - \beta_{j}) s_{j}. */
      vecadd(d, it->s, it->alpha - beta);
      j = (j + 1) % m; /* if (++j == m) j = 0; */
    }

    /*
     *  Constrain the search direction for orthant-wise updates.
     */
    if( orthantwise_c_ != 0 ) {
      for( i = orthantwise_start_; i < orthantwise_end_; ++i ) {
        if( d[i] * pg[i] >= 0 ) {
          d[i] = 0;
        }
      }
    }

    /*
     * Now the search direction d is ready.
     * We try step = 1 first.
     */
    step = 1.0;
  }

  return status_;
}

c_bfgs::STATUS c_bfgs::line_search_morethuente(const callback & cb,
    std::vector<double> & x, /* [n] */
    double * f,
    std::vector<double> & g, /* [n] */
    const std::vector<double> & s, /* [n] */
    double * stp,
    const std::vector<double> & xp, /* [n] */
    const std::vector<double>& /*gp*/, /* [n] */
    std::vector<double>& /*wa*//* [n] */)
{
  int count = 0;
  int brackt, stage1, uinfo = 0;
  double dg;
  double stx, fx, dgx;
  double sty, fy, dgy;
  double fxm, dgxm, fym, dgym, fm, dgm;
  double finit, ftest1, dginit, dgtest;
  double width, prev_width;
  double stmin, stmax;

  /* Check the input parameters for errors. */
  if( *stp <= 0 ) {
    return LBFGSERR_INVALIDPARAMETERS;
  }

  /* Compute the initial gradient in the search direction. */
  dginit = vecdot(g, s);

  /* Make sure that s points to a descent direction. */
  if( 0 < dginit ) {
    CF_ERROR("LBFGSERR_INCREASEGRADIENT: dginit=%+g", dginit);
    return LBFGSERR_INCREASEGRADIENT;
  }

  /* Initialize local variables. */
  brackt = 0;
  stage1 = 1;
  finit = *f;
  dgtest = ftol_ * dginit;
  width = max_step_ - min_step_;
  prev_width = 2.0 * width;

  /*
   The variables stx, fx, dgx contain the values of the step,
   function, and directional derivative at the best step.
   The variables sty, fy, dgy contain the value of the step,
   function, and derivative at the other endpoint of
   the interval of uncertainty.
   The variables stp, f, dg contain the values of the step,
   function, and derivative at the current step.
   */
  stx = sty = 0.;
  fx = fy = finit;
  dgx = dgy = dginit;

  for( ;; ) {
    /*
     Set the minimum and maximum steps to correspond to the
     present interval of uncertainty.
     */
    if( brackt ) {
      stmin = (std::min)(stx, sty);
      stmax = (std::max)(stx, sty);
    }
    else {
      stmin = stx;
      stmax = *stp + 4.0 * (*stp - stx);
    }

    /* Clip the step in the range of [stpmin, stpmax]. */
    if( *stp < min_step_ ) {
      *stp = min_step_;
    }
    if( max_step_ < *stp ) {
      *stp = max_step_;
    }

    /*
     If an unusual termination is to occur then let
     stp be the lowest point obtained so far.
     */
    if( (brackt && ((*stp <= stmin || stmax <= *stp) || max_linesearch_ <= count + 1 || uinfo != 0))
        || (brackt && (stmax - stmin <= xtol_ * stmax)) ) {
      *stp = stx;
    }

    /*
     Compute the current value of x:
     x <- x + (*stp) * s.
     */
    veccpy(x, xp);
    vecadd(x, s, *stp);

    /* Evaluate the function and gradient values. */
    //*f = cd->proc_evaluate(cd->instance, x, g, cd->n, *stp);
    if( !compute(cb, x, f, &g) ) {
      CF_ERROR("compute() fails");
      return LBFGSERR_UNKNOWNERROR;
    }

    dg = vecdot(g, s);

    ftest1 = finit + *stp * dgtest;
    ++count;

    /* Test for errors and convergence. */
    if( brackt && ((*stp <= stmin || stmax <= *stp) || uinfo != 0) ) {
      /* Rounding errors prevent further progress. */
      return LBFGSERR_ROUNDING_ERROR;
    }
    if( *stp == max_step_ && *f <= ftest1 && dg <= dgtest ) {
      /* The step is the maximum value. */
      return LBFGSERR_MAXIMUMSTEP;
    }
    if( *stp == min_step_ && (ftest1 < *f || dgtest <= dg) ) {
      /* The step is the minimum value. */
      return LBFGSERR_MINIMUMSTEP;
    }
    if( brackt && (stmax - stmin) <= xtol_ * stmax ) {
      /* Relative width of the interval of uncertainty is at most xtol. */
      return LBFGSERR_WIDTHTOOSMALL;
    }
    if( max_linesearch_ <= count ) {
      /* Maximum number of iteration. */
      return LBFGSERR_MAXIMUMLINESEARCH;
    }
    if( *f <= ftest1 && fabs(dg) <= gtol_ * (-dginit) ) {
      /* The sufficient decrease condition and the directional derivative condition hold. */
      return LBFGS_SUCCESS; // count;
    }

    /*
     In the first stage we seek a step for which the modified
     function has a nonpositive value and nonnegative derivative.
     */
    if( stage1 && *f <= ftest1 && (std::min)(ftol_, gtol_) * dginit <= dg ) {
      stage1 = 0;
    }

    /*
     A modified function is used to predict the step only if
     we have not obtained a step for which the modified
     function has a nonpositive function value and nonnegative
     derivative, and if a lower function value has been
     obtained but the decrease is not sufficient.
     */
    if( stage1 && ftest1 < *f && *f <= fx ) {
      /* Define the modified function and derivative values. */
      fm = *f - *stp * dgtest;
      fxm = fx - stx * dgtest;
      fym = fy - sty * dgtest;
      dgm = dg - dgtest;
      dgxm = dgx - dgtest;
      dgym = dgy - dgtest;

      /*
       Call update_trial_interval() to update the interval of
       uncertainty and to compute the new step.
       */
      uinfo = update_trial_interval(
          &stx, &fxm, &dgxm,
          &sty, &fym, &dgym,
          stp, &fm, &dgm,
          stmin, stmax, &brackt
          );

      /* Reset the function and gradient values for f. */
      fx = fxm + stx * dgtest;
      fy = fym + sty * dgtest;
      dgx = dgxm + dgtest;
      dgy = dgym + dgtest;
    }
    else {
      /*
       Call update_trial_interval() to update the interval of
       uncertainty and to compute the new step.
       */
      uinfo = update_trial_interval(
          &stx, &fx, &dgx,
          &sty, &fy, &dgy,
          stp, f, &dg,
          stmin, stmax, &brackt
          );
    }

    /*
     Force a sufficient decrease in the interval of uncertainty.
     */
    if( brackt ) {
      if( 0.66 * prev_width <= fabs(sty - stx) ) {
        *stp = stx + 0.5 * (sty - stx);
      }
      prev_width = width;
      width = fabs(sty - stx);
    }
  }

  return LBFGS_SUCCESS;
}

c_bfgs::STATUS c_bfgs::line_search_backtracking_owlqn(const callback & cb,
    std::vector<double> & x, /* [n] */
    double * f,
    std::vector<double> & g, /* [n] */
    const std::vector<double> & s, /* [n] */
    double * stp,
    const std::vector<double> & xp, /* [n] */
    const std::vector<double> & gp, /* [n] */
    std::vector<double> & wp /* [n] */)
{

  int i, count = 0;
  double width = 0.5, norm = 0.;
  double finit = *f, dgtest;

  /* Check the input parameters for errors. */
  if( *stp <= 0. ) {
    return LBFGSERR_INVALIDPARAMETERS;
  }

  const int n = x.size();

  /* Choose the orthant for the new point. */
  for( i = 0; i < n; ++i ) {
    wp[i] = (xp[i] == 0.) ? -gp[i] : xp[i];
  }

  for( ;; ) {
    /* Update the current point. */
    veccpy(x, xp);
    vecadd(x, s, *stp);

    /* The current point is projected onto the orthant. */
    owlqn_project(x, wp, orthantwise_start_, orthantwise_end_);

    /* Evaluate the function and gradient values. */
    // *f = cd->proc_evaluate(cd->instance, x, g, cd->n, *stp);
    if( !compute(cb, x, f, &g) ) {
      CF_ERROR("compute() fails");
      return LBFGSERR_UNKNOWNERROR;
    }

    /* Compute the L1 norm of the variables and add it to the object value. */
    norm = owlqn_x1norm(x, orthantwise_start_, orthantwise_end_);
    *f += norm * orthantwise_c_;

    ++count;

    dgtest = 0.;
    for( i = 0; i < n; ++i ) {
      dgtest += (x[i] - xp[i]) * gp[i];
    }

    if( *f <= finit + ftol_ * dgtest ) {
      /* The sufficient decrease condition. */
      return LBFGS_SUCCESS; // count;
    }

    if( *stp < min_step_ ) {
      /* The step is the minimum value. */
      return LBFGSERR_MINIMUMSTEP;
    }
    if( *stp > max_step_ ) {
      /* The step is the maximum value. */
      return LBFGSERR_MAXIMUMSTEP;
    }
    if( max_linesearch_ <= count ) {
      /* Maximum number of iteration. */
      return LBFGSERR_MAXIMUMLINESEARCH;
    }

    (*stp) *= width;
  }
  return LBFGS_SUCCESS;
}

c_bfgs::STATUS c_bfgs::line_search_backtracking(const callback & cb,
    std::vector<double> & x, /* [n] */
    double * f,
    std::vector<double> & g, /* [n] */
    const std::vector<double> & s, /* [n] */
    double * stp,
    const std::vector<double> & xp, /* [n] */
    const std::vector<double>& /*gp*/, /* [n] */
    std::vector<double>& /*wp*//* [n] */)
{
//  (void)gp;
//  (void)wp;

  int count = 0;
  double width, dg;
  double finit, dginit = 0., dgtest;
  const double dec = 0.5, inc = 2.1;

  /* Check the input parameters for errors. */
  if( *stp <= 0. ) {
    return LBFGSERR_INVALIDPARAMETERS;
  }

  /* Compute the initial gradient in the search direction. */
  dginit = vecdot(g, s);

  /* Make sure that s points to a descent direction. */
  if( 0 < dginit ) {
    CF_ERROR("LBFGSERR_INCREASEGRADIENT: dginit=%+g", dginit);
    return LBFGSERR_INCREASEGRADIENT;
  }

  /* The initial value of the objective function. */
  finit = *f;
  dgtest = ftol_ * dginit;

  for( ;; ) {
    veccpy(x, xp);
    vecadd(x, s, *stp);

    /* Evaluate the function and gradient values. */
    //*f = cd->proc_evaluate(cd->instance, x, g, cd->n, *stp);
    if( !compute(cb, x, f, &g) ) {
      CF_ERROR("compute() fails");
      return LBFGSERR_UNKNOWNERROR;
    }

    ++count;

    if( *f > finit + *stp * dgtest ) {
      width = dec;
    }
    else {
      /* The sufficient decrease condition (Armijo condition). */
      if( linesearch_ == LBFGS_LINESEARCH_BACKTRACKING_ARMIJO ) {
        /* Exit with the Armijo condition. */
        return LBFGS_SUCCESS; // count;
      }

      /* Check the Wolfe condition. */
      dg = vecdot(g, s);
      if( dg < wolfe_ * dginit ) {
        width = inc;
      }
      else {
        if( linesearch_ == LBFGS_LINESEARCH_BACKTRACKING_WOLFE ) {
          /* Exit with the regular Wolfe condition. */
          return LBFGS_SUCCESS; // count;
        }

        /* Check the strong Wolfe condition. */
        if( dg > -wolfe_ * dginit ) {
          width = dec;
        }
        else {
          /* Exit with the strong Wolfe condition. */
          return LBFGS_SUCCESS; // count;
        }
      }
    }

    if( *stp < min_step_ ) {
      /* The step is the minimum value. */
      return LBFGSERR_MINIMUMSTEP;
    }
    if( *stp > max_step_ ) {
      /* The step is the maximum value. */
      return LBFGSERR_MAXIMUMSTEP;
    }
    if( max_linesearch_ <= count ) {
      /* Maximum number of iteration. */
      return LBFGSERR_MAXIMUMLINESEARCH;
    }

    (*stp) *= width;
  }

  return LBFGS_SUCCESS;
}

/**
 * Update a safeguarded trial value and interval for line search.
 *
 *  The parameter x represents the step with the least function value.
 *  The parameter t represents the current step. This function assumes
 *  that the derivative at the point of x in the direction of the step.
 *  If the bracket is set to true, the minimizer has been bracketed in
 *  an interval of uncertainty with endpoints between x and y.
 *
 *  @param  x       The pointer to the value of one endpoint.
 *  @param  fx      The pointer to the value of f(x).
 *  @param  dx      The pointer to the value of f'(x).
 *  @param  y       The pointer to the value of another endpoint.
 *  @param  fy      The pointer to the value of f(y).
 *  @param  dy      The pointer to the value of f'(y).
 *  @param  t       The pointer to the value of the trial value, t.
 *  @param  ft      The pointer to the value of f(t).
 *  @param  dt      The pointer to the value of f'(t).
 *  @param  tmin    The minimum value for the trial value, t.
 *  @param  tmax    The maximum value for the trial value, t.
 *  @param  brackt  The pointer to the predicate if the trial value is
 *                  bracketed.
 *  @retval int     Status value. Zero indicates a normal termination.
 *
 *  @see
 *      Jorge J. More and David J. Thuente. Line search algorithm with
 *      guaranteed sufficient decrease. ACM Transactions on Mathematical
 *      Software (TOMS), Vol 20, No 3, pp. 286-307, 1994.
 */
int c_bfgs::update_trial_interval(
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
    int * brackt
    )
{
  /**
   * Find a minimizer of an interpolated cubic function.
   *  @param  cm      The minimizer of the interpolated cubic.
   *  @param  u       The value of one point, u.
   *  @param  fu      The value of f(u).
   *  @param  du      The value of f'(u).
   *  @param  v       The value of another point, v.
   *  @param  fv      The value of f(v).
   *  @param  du      The value of f'(v).
   */
#define CUBIC_MINIMIZER(cm, u, fu, du, v, fv, dv) \
      d = (v) - (u); \
      theta = ((fu) - (fv)) * 3 / d + (du) + (dv); \
      p = fabs(theta); \
      q = fabs(du); \
      r = fabs(dv); \
      s = max3(p, q, r); \
      /* gamma = s*sqrt((theta/s)**2 - (du/s) * (dv/s)) */ \
      a = theta / s; \
      gamma = s * sqrt(a * a - ((du) / s) * ((dv) / s)); \
      if ((v) < (u)) gamma = -gamma; \
      p = gamma - (du) + theta; \
      q = gamma - (du) + gamma + (dv); \
      r = p / q; \
      (cm) = (u) + r * d;

  /**
   * Find a minimizer of an interpolated cubic function.
   *  @param  cm      The minimizer of the interpolated cubic.
   *  @param  u       The value of one point, u.
   *  @param  fu      The value of f(u).
   *  @param  du      The value of f'(u).
   *  @param  v       The value of another point, v.
   *  @param  fv      The value of f(v).
   *  @param  dv      The value of f'(v).
   *  @param  xmin    The minimum value.
   *  @param  xmax    The maximum value.
   */
#define CUBIC_MINIMIZER2(cm, u, fu, du, v, fv, dv, xmin, xmax) \
      d = (v) - (u); \
      theta = ((fu) - (fv)) * 3 / d + (du) + (dv); \
      p = fabs(theta); \
      q = fabs(du); \
      r = fabs(dv); \
      s = max3(p, q, r); \
      /* gamma = s*sqrt((theta/s)**2 - (du/s) * (dv/s)) */ \
      a = theta / s; \
      gamma = s * sqrt((std::max)(0., a * a - ((du) / s) * ((dv) / s))); \
      if ((u) < (v)) gamma = -gamma; \
      p = gamma - (dv) + theta; \
      q = gamma - (dv) + gamma + (du); \
      r = p / q; \
      if (r < 0. && gamma != 0.) { \
          (cm) = (v) - r * d; \
      } else if (d > 0) { \
          (cm) = (xmax); \
      } else { \
          (cm) = (xmin); \
      }

  /**
   * Find a minimizer of an interpolated quadratic function.
   *  @param  qm      The minimizer of the interpolated quadratic.
   *  @param  u       The value of one point, u.
   *  @param  fu      The value of f(u).
   *  @param  du      The value of f'(u).
   *  @param  v       The value of another point, v.
   *  @param  fv      The value of f(v).
   */
#define QUAD_MINIMIZER(qm, u, fu, du, v, fv) \
      a = (v) - (u); \
      (qm) = (u) + (du) / (((fu) - (fv)) / a + (du)) / 2 * a;

  /**
   * Find a minimizer of an interpolated quadratic function.
   *  @param  qm      The minimizer of the interpolated quadratic.
   *  @param  u       The value of one point, u.
   *  @param  du      The value of f'(u).
   *  @param  v       The value of another point, v.
   *  @param  dv      The value of f'(v).
   */
#define QUAD_MINIMIZER2(qm, u, du, v, dv) \
      a = (u) - (v); \
      (qm) = (v) + (dv) / ((dv) - (du)) * a;

  // FIXME : what is this ?
  // #define fsigndiff(x, y) (*(x) * (*(y) / fabs(*(y))) < 0.)
  static const auto fsigndiff =
      [](double * x, double * y) -> int {
        return (*(x) * (*(y) / fabs(*(y))) < 0.);
      };

  int bound;
  int dsign = fsigndiff(dt, dx);
  double mc; /* minimizer of an interpolated cubic. */
  double mq; /* minimizer of an interpolated quadratic. */
  double newt; /* new trial value. */

  /* for CUBIC_MINIMIZER and QUAD_MINIMIZER. */
  double a, d, gamma, theta, p, q, r, s;

  /* Check the input parameters for errors. */
  if( *brackt ) {
    if( *t <= (std::min)(*x, *y) || (std::max)(*x, *y) <= *t ) {
      /* The trivial value t is out of the interval. */
      return LBFGSERR_OUTOFINTERVAL;
    }

    double tmp__ = *dx * (*t - *x);

    if( 0. <= *dx * (*t - *x) ) {
      /* The function must decrease from x. */
      CF_ERROR("LBFGSERR_INCREASEGRADIENT: tmp__=%+g", tmp__);
      return LBFGSERR_INCREASEGRADIENT;
    }
    if( tmax < tmin ) {
      /* Incorrect tmin and tmax specified. */
      return LBFGSERR_INCORRECT_TMINMAX;
    }
  }

  /*
   Trial value selection.
   */
  if( *fx < *ft ) {
    /*
     Case 1: a higher function value.
     The minimum is brackt. If the cubic minimizer is closer
     to x than the quadratic one, the cubic one is taken, else
     the average of the minimizers is taken.
     */
    *brackt = 1;
    bound = 1;
    CUBIC_MINIMIZER(mc, *x, *fx, *dx, *t, *ft, *dt);
    QUAD_MINIMIZER(mq, *x, *fx, *dx, *t, *ft);
    if( fabs(mc - *x) < fabs(mq - *x) ) {
      newt = mc;
    }
    else {
      newt = mc + 0.5 * (mq - mc);
    }
  }
  else if( dsign ) {
    /*
     Case 2: a lower function value and derivatives of
     opposite sign. The minimum is brackt. If the cubic
     minimizer is closer to x than the quadratic (secant) one,
     the cubic one is taken, else the quadratic one is taken.
     */
    *brackt = 1;
    bound = 0;
    CUBIC_MINIMIZER(mc, *x, *fx, *dx, *t, *ft, *dt);
    QUAD_MINIMIZER2(mq, *x, *dx, *t, *dt);
    if( fabs(mc - *t) > fabs(mq - *t) ) {
      newt = mc;
    }
    else {
      newt = mq;
    }
  }
  else if( fabs(*dt) < fabs(*dx) ) {
    /*
     Case 3: a lower function value, derivatives of the
     same sign, and the magnitude of the derivative decreases.
     The cubic minimizer is only used if the cubic tends to
     infinity in the direction of the minimizer or if the minimum
     of the cubic is beyond t. Otherwise the cubic minimizer is
     defined to be either tmin or tmax. The quadratic (secant)
     minimizer is also computed and if the minimum is brackt
     then the the minimizer closest to x is taken, else the one
     farthest away is taken.
     */
    bound = 1;
    CUBIC_MINIMIZER2(mc, *x, *fx, *dx, *t, *ft, *dt, tmin, tmax);
    QUAD_MINIMIZER2(mq, *x, *dx, *t, *dt);
    if( *brackt ) {
      if( fabs(*t - mc) < fabs(*t - mq) ) {
        newt = mc;
      }
      else {
        newt = mq;
      }
    }
    else {
      if( fabs(*t - mc) > fabs(*t - mq) ) {
        newt = mc;
      }
      else {
        newt = mq;
      }
    }
  }
  else {
    /*
     Case 4: a lower function value, derivatives of the
     same sign, and the magnitude of the derivative does
     not decrease. If the minimum is not brackt, the step
     is either tmin or tmax, else the cubic minimizer is taken.
     */
    bound = 0;
    if( *brackt ) {
      CUBIC_MINIMIZER(newt, *t, *ft, *dt, *y, *fy, *dy);
    }
    else if( *x < *t ) {
      newt = tmax;
    }
    else {
      newt = tmin;
    }
  }

  /*
   Update the interval of uncertainty. This update does not
   depend on the new step or the case analysis above.

   - Case a: if f(x) < f(t),
   x <- x, y <- t.
   - Case b: if f(t) <= f(x) && f'(t)*f'(x) > 0,
   x <- t, y <- y.
   - Case c: if f(t) <= f(x) && f'(t)*f'(x) < 0,
   x <- t, y <- x.
   */
  if( *fx < *ft ) {
    /* Case a */
    *y = *t;
    *fy = *ft;
    *dy = *dt;
  }
  else {
    /* Case c */
    if( dsign ) {
      *y = *x;
      *fy = *fx;
      *dy = *dx;
    }
    /* Cases b and c */
    *x = *t;
    *fx = *ft;
    *dx = *dt;
  }

  /* Clip the new trial value in [tmin, tmax]. */
  if( tmax < newt )
    newt = tmax;
  if( newt < tmin )
    newt = tmin;

  /*
   Redefine the new trial value if it is close to the upper bound
   of the interval.
   */
  if( *brackt && bound ) {
    mq = *x + 0.66 * (*y - *x);
    if( *x < *y ) {
      if( mq < newt )
        newt = mq;
    }
    else {
      if( newt < mq )
        newt = mq;
    }
  }

  /* Return the new trial value. */
  *t = newt;

#undef CUBIC_MINIMIZER
#undef CUBIC_MINIMIZER2
#undef QUAD_MINIMIZER
#undef QUAD_MINIMIZER2

  return 0;
}

