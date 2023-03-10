/*
 * levmar.h
 *
 *  This file is derived from levmar 2.6 source code
 *  <http://users.ics.forth.gr/~lourakis/levmar>.
 *
 *  The reason is to reduce the external library dependencies for
 *  glddm project cross-compilation for variety of target platforms.
 *
 *  Created on: Dec 18, 2021
 *      Author: amyznikov
 */

#pragma once
#ifndef __levmar_h__
#define __levmar_h__

#define LM_OPTS_SZ        5 /* max(4, 5) */
#define LM_INFO_SZ        10
#define LM_ERROR          -1
#define LM_INIT_MU        1E-03
#define LM_STOP_THRESH    1E-17
#define LM_DIFF_DELTA     1E-06


/* work arrays size for ?levmar_der and ?levmar_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_DER_WORKSZ(npar, nmeas) (2*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))
#define LM_DIF_WORKSZ(npar, nmeas) (4*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))

/* levmar input options */
struct c_levmar_opts {

  /* Damping constant */
  double mu = LM_INIT_MU;

  /* Stopping threshold for ||J^T e||_inf */
  double eps1 = LM_STOP_THRESH;

  /* Stopping threshold for ||Dp||_2 */
  double eps2 = LM_STOP_THRESH;

  /* Stopping threshold for ||e||_2 */
  double eps3 = LM_STOP_THRESH;

  /* Relevant only if the Jacobian is approximated using finite differences;
   * The step used in difference approximation to the Jacobian.
   * If delta<0, the Jacobian is approximated with central differences
   * which are more accurate but slower compared to the forward
   * differences employed by default */
  double delta = LM_DIFF_DELTA;
};


enum levmar_stopping_reason {
  levmar_stopping_reason_unknow = 0,
  levmar_stopping_reason_eps1 = 1,  // stopped by small gradient J^T e
  levmar_stopping_reason_eps2 = 2, // stopped by small Dp
  levmar_stopping_reason_itmax = 3,  // stopped by itmax
  levmar_stopping_reason_singular_matrix = 4,  // singular matrix. Restart from current p with increased mu
  levmar_stopping_reason_mu = 5,  // no further error reduction is possible. Restart with increased mu
  levmar_stopping_reason_eps3 = 6,  // stopped by small ||e||_2
  levmar_stopping_reason_nan = 7,  // stopped by invalid (i.e. NaN or Inf) "func" values. This is a user error
} ;

const char * comment(enum levmar_stopping_reason v);



/* levmar output information regarding the minimization */
struct c_levmar_status {

  double eps3_initial = 0; // ||e||_2 at initial p.
  double eps3 = 0;  // ||e||_2 at estimated p
  double eps1 = 0;  // ||J^T e||_inf at estimated p
  double eps2 = 0;  // ||Dp||_2 at estimated p
  double mu = 0;    // mu/max[J^T J]_ii ], all computed at estimated p.

  enum levmar_stopping_reason reason =
      levmar_stopping_reason_unknow;

  int iters = 0;  // info[5]= # iterations,
  int nfev = 0;   // info[7]= # function evaluations
  int njev = 0;   // info[8]= # Jacobian evaluations
  int nlss = 0;   // info[9]= # linear systems solved, i.e. # attempts for reducing error
};

int levmar_der(
    void (*func)(const double p[], double hx[], int m, int n, void *adata),
    void (*jacf)(const double p[], double j[], int m, int n, void *adata),
    double *p,
    double *x,
    int m,
    int n,
    int itmax,
    const c_levmar_opts & opts,
    //double info[LM_INFO_SZ],
    c_levmar_status & info,
    double *work,
    double *covar,
    void *adata);

int levmar_der(
    void (*func)(const float p[], float hx[], int m, int n, void *adata),
    void (*jacf)(const float p[], float j[], int m, int n, void *adata),
    float p[],
    float x[],
    int m,
    int n,
    int itmax,
    const c_levmar_opts & opts,
    //float info[LM_INFO_SZ],
    c_levmar_status & info,
    float *work,
    float *covar,
    void *adata);

int levmar_dif(
    void (*func)(const double p[], double hx[], int m, int n, void *adata),
    double p[],
    double x[],
    int m,
    int n,
    int itmax,
    const c_levmar_opts & opts,
    //double info[LM_INFO_SZ],
    c_levmar_status & info,
    double work[],
    double covar[],
    void * adata);

int levmar_dif(
    void (*func)(const float p[], float hx[], int m, int n, void *adata),
    float p[],
    float x[],
    int m,
    int n,
    int itmax,
    const c_levmar_opts & opts,
    //float info[LM_INFO_SZ],
    c_levmar_status & info,
    float work[],
    float covar[],
    void * adata);

#endif /* __levmar_h__ */
