/*
 * bfgs.cc
 *
 *  Created on: Aug 12, 2023
 *      Author: amyznikov
 */
#include "bfgs.h"
#include <core/debug.h>

// #define ITMAX 200
// #define EPS 3.0e-8
// #define TOLX (4*EPS)
// #define STPMX 100.0

static inline double SQR(double x)
{
  return x * x;
}

static inline double dot(const std::vector<double> & x, const std::vector<double> & y)
{
  double s = 0;
  for( int i = 0, n = x.size(); i < n; ++i ) {
    s += x[i] * y[i];
  }
  return s;
}

c_bfgs::c_bfgs(int max_iterations) :
    max_iterations_(max_iterations)
{

}

c_bfgs::~c_bfgs()
{

}

bool c_bfgs::compute(const callback & cb, const std::vector<double> & p,
    double * f, std::vector<double> * g) const
{

  bool have_g = false;

  if( !cb.compute(p, f, g, &have_g) ) {
    CF_ERROR("cb.compute() fails");
    return false;
  }

  if ( g ) {

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

/*
 * Given a starting point p[1..n] that is a vector of length n, the Broyden-Fletcher-Goldfarb-
 * Shanno variant of Davidon-Fletcher-Powell minimization is performed on a function func, using
 * its gradient as calculated by a routine dfunc. The convergence requirement on zeroing the
 * gradient is input as gtol. Returned quantities are p[1..n] (the location of the minimum),
 * iter (the number of iterations that were performed), and fret (the minimum value of the
 * function). The routine lnsrch is called to perform approximate line minimizations.
 */
int c_bfgs::run(const callback & cb, std::vector<double> & p)
{
  const int n = p.size();

  double fp, fret, den, fac, fad, fae, stpmax, sum = 0.0, sumdg, sumxi, temp, test;

  int check, its, i, j;

  // g = vector(1,n);
  std::vector<double> g(n);

  // dg = vector(1,n);
  std::vector<double> dg(n);

  // hdg = vector(1,n);
  std::vector<double> hdg(n);

  // hessin = matrix(1,n,1,n);
  cv::Mat1d hessin =
      cv::Mat1d::eye(n, n);

  // pnew = vector(1,n);
  std::vector<double> pnew(n);

  // xi=vector(1,n);
  std::vector<double> xi(n);

  if( !compute(cb, p, &fp, &g) ) {
    CF_ERROR("compute() fails");
    return -1;
  }

  for( i = 0; i < n; ++i ) {
    xi[i] = -g[i];
    sum += p[i] * p[i];
  }

  stpmax = stpmx_ * (std::max)(sqrt(sum), (double) n);

  // Main loop over the iterations.
  for( its = 1; its <= max_iterations_; ++its ) {

    // *iter = its;
    lnsrch(n, p, fp, g, xi, pnew, &fret, stpmax, &check, cb);

    // The new function evaluation occurs in lnsrch; save the function value in fp
    // for the next line search. It is usually safe to ignore the value of check.

    fp = fret;
    for( i = 0; i < n; ++i ) {
      xi[i] = pnew[i] - p[i]; // Update the line direction,
      p[i] = pnew[i]; // and the current point.
    }

    test = 0.0; // Test for convergence on ∆x.

    for( i = 0; i < n; ++i ) {
      temp = std::abs(xi[i]) / (std::max)(std::abs(p[i]), 1.0);
      if( temp > test ) {
        test = temp;
      }
    }

    if( test < tolx_ ) {
      return its;
    }

    for( i = 0; i < n; ++i ) { // Save the old gradient
      dg[i] = g[i];
    }

    if( !compute(cb, p, nullptr, &g) ) { // and get the new gradient
      CF_ERROR("compute() fails");
      return -1;
    }

    test = 0.0; // Test for convergence on zero gradient.

    den = (std::max)(fret, 1.0);

    for( i = 0; i < n; ++i ) {
      temp = std::abs(g[i]) * (std::max)(std::abs(p[i]), 1.0) / den;
      if( temp > test ) {
        test = temp;
      }
    }

    if( test < gtol_ ) {
      return its;
    }

    for( i = 0; i < n; ++i ) { // Compute difference of gradients
      dg[i] = g[i] - dg[i];
    }

    for( i = 0; i < n; ++i ) { // and difference times current matrix.
      hdg[i] = 0.0;
      for( j = 0; j < n; ++j ) {
        hdg[i] += hessin[i][j] * dg[j];
      }
    }

    // Calculate dot products  for the denominators
    fac = fae = sumdg = sumxi = 0.0;

    for( i = 0; i < n; ++i ) {
      fac += dg[i] * xi[i];
      fae += dg[i] * hdg[i];
      sumdg += SQR(dg[i]);
      sumxi += SQR(xi[i]);
    }

    if( fac > sqrt(eps_ * sumdg * sumxi) ) { // Skip update if fac not sufficiently positive.
      fac = 1.0 / fac;
      fad = 1.0 / fae;

      // The vector that makes BFGS different from DFP:
      for( i = 0; i < n; ++i ) {
        dg[i] = fac * xi[i] - fad * hdg[i];
      }

      for( i = 0; i < n; ++i ) { // The BFGS updating formula:
        for( j = i; j < n; ++j ) {
          hessin[i][j] += fac * xi[i] * xi[j] - fad * hdg[i] * hdg[j] + fae * dg[i] * dg[j];
          hessin[j][i] = hessin[i][j];
        }
      }
    }

    for( i = 0; i < n; ++i ) { // Now calculate the next direction to go,
      xi[i] = 0.0;
      for( j = 0; j < n; ++j ) {
        xi[i] -= hessin[i][j] * g[j];
      }
    }
  } // and go back for another iteration.

  CF_DEBUG("too many iterations %d", its);
  return its;
}

/*
 * Given an n-dimensional point xold[1..n], the value of the function and gradient there, fold
 * and g[1..n] , and a direction p[1..n] , finds a new point x[1..n] along the direction p from
 * xold where the function func has decreased “sufficiently.” The new function value is returned
 * in f. stpmax is an input quantity that limits the length of the steps so that you do not try to
 * evaluate the function in regions where it is undefined or subject to overflow. p is usually the
 * Newton direction. The output quantity check is false (0) on a normal exit. It is true (1) when
 * x is too close to xold. In a minimization algorithm, this usually signals convergence and can
 * be ignored. However, in a zero-finding algorithm the calling program should check whether the
 * convergence is spurious. Some “difficult” problems may require double precision in this routine.
 */
void c_bfgs::lnsrch(int n, const std::vector<double> & xold, double fold,
    const std::vector<double> & g, const std::vector<double> & p, std::vector<double> & x,
    double * f, double stpmax, int * check, const callback & cb)
{
  constexpr double ALF = 1.0e-4;
  constexpr double TOLX = 1.0e-7;

  double a, alam, alam2, alamin, b, disc, f2, rhs1, rhs2, slope, temp, test, tmplam;
  int i;

  *check = 0;

  ////////
  if ( true ) {
    //  double sum;
    //  for( sum = 0.0, i = 0; i < n; ++i ) {
    //    sum += p[i] * p[i];
    //  }
    //  sum = sqrt(sum);
    const double sum = sqrt(dot(p, p));
    if( sum > stpmax ) {
      CF_DEBUG("WARNING: sum=%g > stpmax=%g", sum, stpmax);
      //  for( i = 0; i < n; ++i ) { // Scale if attempted step is too big.
      //    p[i] *= stpmax / sum;
      //  }
    }
  }
  ////////

  //  for( slope = 0.0, i = 0; i < n; ++i ) {
  //    slope += g[i] * p[i];
  //  }
  slope = dot(p, g);
  if( slope >= 0 ) {
    CF_ERROR("Roundoff problem in lnsrch: slope=%g", slope);
  }

  test = 0.0; // Compute λmin .
  for( i = 0; i < n; ++i ) {
    if( (temp = fabs(p[i]) / (std::max)(fabs(xold[i]), 1.0)) > test ) {
      test = temp;
    }
  }

  alamin = TOLX / test;
  alam = 1.0; // Always try full Newton step first.

  for( ;; ) { // Start of iteration loop.

    for( i = 0; i < n; ++i ) {
      x[i] = xold[i] + alam * p[i];
    }

    //*f = (*func)(x);
    if ( !compute(cb, x, f, nullptr) ) {
      CF_ERROR("compute() fails");
    }


    if( alam < alamin ) { // Convergence on ∆x.
      // For zero finding, the calling program should verify the convergence.
      // for( i = 0; i < n; ++i ) {
      //  x[i] = xold[i];
      //  }
      x = xold;
      *check = 1;
      return;
    }

    if( *f <= fold + ALF * alam * slope ) { // Sufficient function decrease.
      return;
    }

    // Backtrack.
    if( alam == 1 ) { // First time
      tmplam = -slope / (2.0 * (*f - fold - slope));
    }
    else { // Subsequent backtracks.

      rhs1 = *f - fold - alam * slope;
      rhs2 = f2 - fold - alam2 * slope;
      a = (rhs1 / (alam * alam) - rhs2 / (alam2 * alam2)) / (alam - alam2);
      b = (-alam2 * rhs1 / (alam * alam) + alam * rhs2 / (alam2 * alam2)) / (alam - alam2);

      if( a == 0 ) {
        tmplam = -slope / (2.0 * b);
      }
      else {

        disc = b * b - 3.0 * a * slope;

        if( disc < 0.0 ) {
          tmplam = 0.5 * alam;
        }
        else if( b <= 0.0 ) {
          tmplam = (-b + sqrt(disc)) / (3.0 * a);
        }
        else {
          tmplam = -slope / (b + sqrt(disc));
        }
      }

      if( tmplam > 0.5 * alam ) {
        tmplam = 0.5 * alam; // λ ≤ 0.5λ1.
      }
    }

    alam2 = alam;
    f2 = *f;
    alam = (std::max)(tmplam, 0.1 * alam); // λ ≥ 0.1λ1.

  } //     Try again.
}



