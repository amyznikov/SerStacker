/*
 * levmar.cc
 *
 *  This file is derived from levmar 2.6 source code
 *  <http://users.ics.forth.gr/~lourakis/levmar>.
 *
 *  The reason is to reduce the external library dependencies
 *  for glddm project cross-compilation for variety of target platforms.
 *
 *  Created on: Dec 18, 2021
 *      Author: amyznikov
 */
#define HAVE_LAPACK 1

#include "levmar.h"
//#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <limits>
#include <vector>
#include <cmath>
#include <core/debug.h>

static constexpr int __BLOCKSZ__ = 32;
static constexpr int __BLOCKSZ__SQ = (__BLOCKSZ__) * (__BLOCKSZ__);

#if HAVE_LAPACK
extern "C" {

/* C := alpha*op( A )*op( B ) + beta*C */
void dgemm_(const char *transa, const char *transb,
    int *m, int *n, int *k,
    double *alpha,
    double *a, int *lda,
    double *b, int *ldb,
    double *beta,
    double *c,
    int *ldc);

void sgemm_(const char *transa, const char *transb,
    int *m, int *n, int *k,
    float *alpha,
    float *a, int *lda,
    float *b, int *ldb,
    float *beta,
    float *c,
    int *ldc);

int dgesvd_(const char * jobu, const char * jobvt,
    int *m, int *n,
    double *a, int *lda,
    double *s,
    double *u, int *ldu,
    double *vt, int *ldvt,
    double *work, int *lwork,
    int *info);

int sgesvd_(const char * jobu, const char * jobvt,
    int *m, int *n,
    float *a, int *lda,
    float *s,
    float *u, int *ldu,
    float *vt, int *ldvt,
    float *work, int *lwork,
    int *info);

/* lapack 3.0 new SVD routine, faster than xgesvd() */
int dgesdd_(const char *jobz,
    int *m, int *n,
    double *a, int *lda,
    double *s,
    double *u, int *ldu,
    double *vt, int *ldvt,
    double *work, int *lwork, int *iwork,
    int *info);

int sgesdd_(const char *jobz,
    int *m, int *n,
    float *a, int *lda,
    float *s,
    float *u, int *ldu,
    float *vt, int *ldvt,
    float *work, int *lwork, int *iwork,
    int *info);

/* Cholesky decomposition */
int dpotf2_(const char *uplo,
    int *n,
    double *a, int *lda,
    int *info);

int spotf2_(const char *uplo,
    int *n,
    float *a, int *lda,
    int *info);

/* LDLt/UDUt factorization and systems solution */
int dsytrf_(const char *uplo,
    int *n,
    double *a, int *lda,
    int *ipiv,
    double *work, int *lwork,
    int *info);

int ssytrf_(const char *uplo,
    int *n,
    float *a, int *lda,
    int *ipiv,
    float *work, int *lwork,
    int *info);

int dsytrs_(const char *uplo,
    int *n,  int *nrhs,
    double *a, int *lda,
    int *ipiv,
    double *b, int *ldb,
    int *info);

int ssytrs_(const char *uplo,
    int *n,  int *nrhs,
    float *a, int *lda,
    int *ipiv,
    float *b, int *ldb,
    int *info);
}
#endif // HAVE_LAPACK

static inline void GEMM(const char *transa, const char *transb,
    int *m, int *n, int *k,
    double *alpha,
    double *a, int *lda,
    double *b, int *ldb,
    double *beta,
    double *c,
    int *ldc)
{
  return dgemm_(transa, transb,
      m, n, k,
      alpha,
      a, lda,
      b, ldb,
      beta,
      c,
      ldc);
}

static inline void GEMM(const char *transa, const char *transb,
    int *m, int *n, int *k,
    float *alpha,
    float *a, int *lda,
    float *b, int *ldb,
    float *beta,
    float *c,
    int *ldc)
{
  return sgemm_(transa, transb,
      m, n, k,
      alpha,
      a, lda,
      b, ldb,
      beta,
      c,
      ldc);
}

static int GESVD(const char * jobu, const char * jobvt,
    int *m, int *n,
    double *a, int *lda,
    double *s,
    double *u, int *ldu,
    double *vt, int *ldvt,
    double *work, int *lwork,
    int *info)
{
  return dgesvd_(jobu, jobvt,
      m, n,
      a, lda,
      s,
      u, ldu,
      vt, ldvt,
      work, lwork,
      info);
}

static int GESVD(const char * jobu, const char * jobvt,
    int *m, int *n,
    float *a, int *lda,
    float *s,
    float *u, int *ldu,
    float *vt, int *ldvt,
    float *work, int *lwork,
    int *info)
{
  return sgesvd_(jobu, jobvt,
      m, n,
      a, lda,
      s,
      u, ldu,
      vt, ldvt,
      work, lwork,
      info);
}


static inline int SYTRF(const char *uplo, int *n, double *a, int *lda,
    int *ipiv, double *work, int *lwork, int *info)
{
  return dsytrf_(uplo, n, a, lda, ipiv, work, lwork, info);
}

static inline int SYTRF(const char *uplo, int *n, float *a, int *lda,
    int *ipiv, float *work, int *lwork, int *info)
{
  return ssytrf_(uplo, n, a, lda, ipiv, work, lwork, info);
}

static inline int SYTRS(const char *uplo, int *n, int *nrhs, double *a, int *lda,
    int *ipiv, double *b, int *ldb, int *info)
{
  return dsytrs_(uplo, n, nrhs, a, lda, ipiv, b, ldb, info);
}

static inline int SYTRS(const char *uplo, int *n, int *nrhs, float *a, int *lda,
    int *ipiv, float *b, int *ldb, int *info)
{
  return ssytrs_(uplo, n, nrhs, a, lda, ipiv, b, ldb, info);
}


/* work arrays size for ?levmar_bc_der and ?levmar_bc_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BC_DER_WORKSZ(npar, nmeas) (2*(nmeas) + 4*(npar) + (nmeas)*(npar) + (npar)*(npar))
#define LM_BC_DIF_WORKSZ(npar, nmeas) LM_BC_DER_WORKSZ((npar), (nmeas)) /* LEVMAR_BC_DIF currently implemented using LEVMAR_BC_DER()! */

/* work arrays size for ?levmar_lec_der and ?levmar_lec_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_LEC_DER_WORKSZ(npar, nmeas, nconstr) LM_DER_WORKSZ((npar)-(nconstr), (nmeas))
#define LM_LEC_DIF_WORKSZ(npar, nmeas, nconstr) LM_DIF_WORKSZ((npar)-(nconstr), (nmeas))

/* work arrays size for ?levmar_blec_der and ?levmar_blec_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BLEC_DER_WORKSZ(npar, nmeas, nconstr) LM_LEC_DER_WORKSZ((npar), (nmeas)+(npar), (nconstr))
#define LM_BLEC_DIF_WORKSZ(npar, nmeas, nconstr) LM_LEC_DIF_WORKSZ((npar), (nmeas)+(npar), (nconstr))

/* work arrays size for ?levmar_bleic_der and ?levmar_bleic_dif functions.
 * should be multiplied by sizeof(double) or sizeof(float) to be converted to bytes
 */
#define LM_BLEIC_DER_WORKSZ(npar, nmeas, nconstr1, nconstr2) LM_BLEC_DER_WORKSZ((npar)+(nconstr2), (nmeas)+(nconstr2), (nconstr1)+(nconstr2))
#define LM_BLEIC_DIF_WORKSZ(npar, nmeas, nconstr1, nconstr2) LM_BLEC_DIF_WORKSZ((npar)+(nconstr2), (nmeas)+(nconstr2), (nconstr1)+(nconstr2))


#define EPSILON           1E-12
#define ONE_THIRD         0.3333333334 /* 1.0/3.0 */

#define LM_FINITE(x)     std::isfinite(x)

#if HAVE_LAPACK
#define AX_EQ_B_LU Ax_eq_b_LU
#define AX_EQ_B_CHOL Ax_eq_b_Chol
#define AX_EQ_B_QR Ax_eq_b_QR
#define AX_EQ_B_QRLS Ax_eq_b_QRLS
#define AX_EQ_B_SVD Ax_eq_b_SVD
//#define AX_EQ_B_BK Ax_eq_b_BK
#else
#define AX_EQ_B_LU LM_ADD_PREFIX(Ax_eq_b_LU_noLapack)
#endif /* HAVE_LAPACK */

#ifdef HAVE_PLASMA
#define AX_EQ_B_PLASMA_CHOL LM_ADD_PREFIX(Ax_eq_b_PLASMA_Chol)
#endif



#if !HAVE_LAPACK

/* blocked multiplication of the transpose of the nxm matrix a with itself (i.e. a^T a)
 * using a block size of bsize. The product is returned in b.
 * Since a^T a is symmetric, its computation can be sped up by computing only its
 * upper triangular part and copying it to the lower part.
 *
 * More details on blocking can be found at
 * http://www-2.cs.cmu.edu/afs/cs/academic/class/15213-f02/www/R07/section_a/Recitation07-SectionA.pdf
 */

template<class LM_REAL>
static void LEVMAR_TRANS_MAT_MAT_MULT_NOLAPACK(LM_REAL *a, LM_REAL *b, int n, int m)
{
  /* block size for cache-friendly matrix-matrix multiply. It should be
   * such that __BLOCKSZ__^2*sizeof(LM_REAL) is smaller than the CPU (L1)
   * data cache size. Notice that a value of 32 when LM_REAL=double assumes
   * an 8Kb L1 data cache (32*32*8=8K). This is a concervative choice since
   * newer Pentium 4s have a L1 data cache of size 16K, capable of holding
   * up to 45x45 double blocks.
   */

  int i, j, k, jj, kk;
  LM_REAL sum, *bim, *akm;
  const int bsize = __BLOCKSZ__;

#define __MIN__(x, y) (((x)<=(y))? (x) : (y))
#define __MAX__(x, y) (((x)>=(y))? (x) : (y))

  /* compute upper triangular part using blocking */
  for ( jj = 0; jj < m; jj += bsize ) {
    for ( i = 0; i < m; ++i ) {
      bim = b + i * m;
      for ( j = __MAX__(jj, i); j < __MIN__(jj + bsize, m); ++j )
        bim[j] = 0.0;  //b[i*m+j]=0.0;
    }

    for ( kk = 0; kk < n; kk += bsize ) {
      for ( i = 0; i < m; ++i ) {
        bim = b + i * m;
        for ( j = __MAX__(jj, i); j < __MIN__(jj + bsize, m); ++j ) {
          sum = 0.0;
          for ( k = kk; k < __MIN__(kk + bsize, n); ++k ) {
            akm = a + k * m;
            sum += akm[i] * akm[j];  //a[k*m+i]*a[k*m+j];
          }
          bim[j] += sum;  //b[i*m+j]+=sum;
        }
      }
    }
  }

  /* copy upper triangular part to the lower one */
  for ( i = 0; i < m; ++i )
    for ( j = 0; j < i; ++j )
      b[i * m + j] = b[j * m + i];

#undef __MIN__
#undef __MAX__
}

#endif // !HAVE_LAPACK

static void LEVMAR_TRANS_MAT_MAT_MULT(double *a, double *b, int n, int m)
{
#if HAVE_LAPACK /* use BLAS matrix multiply */

  /* Fool BLAS to compute a^T*a avoiding transposing a: a is equivalent to a^T in column major,
   * therefore BLAS computes a*a^T with a and a*a^T in column major, which is equivalent to
   * computing a^T*a in row major!
   */
  double alpha = 1.0, beta = 0.0;
  GEMM("N", "T", &m, &m, &n, &alpha, a, &m, a, &m, &beta, b, &m);

#else /* no LAPACK, use blocking-based multiply */

  LEVMAR_TRANS_MAT_MAT_MULT_NOLAPACK(a, b, n, m);

#endif /* HAVE_LAPACK */
}

static void LEVMAR_TRANS_MAT_MAT_MULT(float * a, float * b, int n, int m)
{
#if HAVE_LAPACK /* use BLAS matrix multiply */

  /* Fool BLAS to compute a^T*a avoiding transposing a: a is equivalent to a^T in column major,
   * therefore BLAS computes a*a^T with a and a*a^T in column major, which is equivalent to
   * computing a^T*a in row major!
   */
  float alpha = 1.0f, beta = 0.0f;
  GEMM("N", "T", &m, &m, &n, &alpha, a, &m, a, &m, &beta, b, &m);

#else /* no LAPACK, use blocking-based multiply */

  LEVMAR_TRANS_MAT_MAT_MULT_NOLAPACK(a, b, n, m);

#endif /* HAVE_LAPACK */
}

/* Compute e=x-y for two n-vectors x and y and return the squared L2 norm of e.
 * e can coincide with either x or y; x can be NULL, in which case it is assumed
 * to be equal to the zero vector.
 * Uses loop unrolling and blocking to reduce bookkeeping overhead & pipeline
 * stalls and increase instruction-level parallelism; see http://www.abarnett.demon.co.uk/tutorial.html
 */

template<typename LM_REAL>
static LM_REAL LEVMAR_L2NRMXMY(LM_REAL *e, LM_REAL *x, LM_REAL *y, int n)
{
  const int blocksize = 8, bpwr = 3; /* 8=2^3 */
  int i, j1, j2, j3, j4, j5, j6, j7;
  int blockn;
  LM_REAL sum0 = 0.0, sum1 = 0.0, sum2 = 0.0, sum3 = 0.0;

  /* n may not be divisible by blocksize,
   * go as near as we can first, then tidy up.
   */
  blockn = (n >> bpwr) << bpwr; /* (n / blocksize) * blocksize; */

  /* unroll the loop in blocks of `blocksize'; looping downwards gains some more speed */
  if ( x ) {
    for ( i = blockn - 1; i > 0; i -= blocksize ) {
      e[i] = x[i] - y[i];
      sum0 += e[i] * e[i];
      j1 = i - 1;
      e[j1] = x[j1] - y[j1];
      sum1 += e[j1] * e[j1];
      j2 = i - 2;
      e[j2] = x[j2] - y[j2];
      sum2 += e[j2] * e[j2];
      j3 = i - 3;
      e[j3] = x[j3] - y[j3];
      sum3 += e[j3] * e[j3];
      j4 = i - 4;
      e[j4] = x[j4] - y[j4];
      sum0 += e[j4] * e[j4];
      j5 = i - 5;
      e[j5] = x[j5] - y[j5];
      sum1 += e[j5] * e[j5];
      j6 = i - 6;
      e[j6] = x[j6] - y[j6];
      sum2 += e[j6] * e[j6];
      j7 = i - 7;
      e[j7] = x[j7] - y[j7];
      sum3 += e[j7] * e[j7];
    }

    /*
     * There may be some left to do.
     * This could be done as a simple for() loop,
     * but a switch is faster (and more interesting)
     */

    i = blockn;
    if ( i < n ) {
      /* Jump into the case at the place that will allow
       * us to finish off the appropriate number of items.
       */

      switch ( n - i ) {
      case 7 :
        e[i] = x[i] - y[i];
        sum0 += e[i] * e[i];
        ++i;
      case 6 :
        e[i] = x[i] - y[i];
        sum1 += e[i] * e[i];
        ++i;
      case 5 :
        e[i] = x[i] - y[i];
        sum2 += e[i] * e[i];
        ++i;
      case 4 :
        e[i] = x[i] - y[i];
        sum3 += e[i] * e[i];
        ++i;
      case 3 :
        e[i] = x[i] - y[i];
        sum0 += e[i] * e[i];
        ++i;
      case 2 :
        e[i] = x[i] - y[i];
        sum1 += e[i] * e[i];
        ++i;
      case 1 :
        e[i] = x[i] - y[i];
        sum2 += e[i] * e[i];  //++i;
      }
    }
  }
  else { /* x==0 */
    for ( i = blockn - 1; i > 0; i -= blocksize ) {
      e[i] = -y[i];
      sum0 += e[i] * e[i];
      j1 = i - 1;
      e[j1] = -y[j1];
      sum1 += e[j1] * e[j1];
      j2 = i - 2;
      e[j2] = -y[j2];
      sum2 += e[j2] * e[j2];
      j3 = i - 3;
      e[j3] = -y[j3];
      sum3 += e[j3] * e[j3];
      j4 = i - 4;
      e[j4] = -y[j4];
      sum0 += e[j4] * e[j4];
      j5 = i - 5;
      e[j5] = -y[j5];
      sum1 += e[j5] * e[j5];
      j6 = i - 6;
      e[j6] = -y[j6];
      sum2 += e[j6] * e[j6];
      j7 = i - 7;
      e[j7] = -y[j7];
      sum3 += e[j7] * e[j7];
    }

    /*
     * There may be some left to do.
     * This could be done as a simple for() loop,
     * but a switch is faster (and more interesting)
     */

    i = blockn;
    if ( i < n ) {
      /* Jump into the case at the place that will allow
       * us to finish off the appropriate number of items.
       */

      switch ( n - i ) {
      case 7 :
        e[i] = -y[i];
        sum0 += e[i] * e[i];
        ++i;
      case 6 :
        e[i] = -y[i];
        sum1 += e[i] * e[i];
        ++i;
      case 5 :
        e[i] = -y[i];
        sum2 += e[i] * e[i];
        ++i;
      case 4 :
        e[i] = -y[i];
        sum3 += e[i] * e[i];
        ++i;
      case 3 :
        e[i] = -y[i];
        sum0 += e[i] * e[i];
        ++i;
      case 2 :
        e[i] = -y[i];
        sum1 += e[i] * e[i];
        ++i;
      case 1 :
        e[i] = -y[i];
        sum2 += e[i] * e[i];  //++i;
      }
    }
  }

  return sum0 + sum1 + sum2 + sum3;
}

/*
 * This function returns the solution of Ax = b for a real symmetric matrix A
 *
 * The function is based on LDLT factorization with the pivoting
 * strategy of Bunch and Kaufman:
 * A is factored as L*D*L^T where L is lower triangular and
 * D symmetric and block diagonal (aka spectral decomposition,
 * Banachiewicz factorization, modified Cholesky factorization)
 *
 * A is mxm, b is mx1.
 *
 * The function returns 0 in case of error, 1 if successfull
 *
 * This function is often called repetitively to solve problems of identical
 * dimensions. To avoid repetitive malloc's and free's, allocated memory is
 * retained between calls and free'd-malloc'ed when not of the appropriate size.
 * A call with NULL as the first argument forces this memory to be released.
 */
template<typename LM_REAL>
static bool AX_EQ_B_BK(LM_REAL *A, LM_REAL *B, LM_REAL *x, int m)
{
  std::vector<LM_REAL> buf;
  int nb = 0;

  LM_REAL *a, *work;
  int a_sz, ipiv_sz, work_sz, tot_sz;
  int info, *ipiv, nrhs = 1;

  if ( !A ) {
    return true;
  }

  /* calculate required memory size */
  ipiv_sz = m;
  a_sz = m * m;
  if ( !nb ) {
    LM_REAL tmp;

    work_sz = -1;  // workspace query; optimal size is returned in tmp
    SYTRF("L", &m, NULL, &m, NULL, &tmp, &work_sz, &info);
    nb = ((int) tmp) / m;  // optimal worksize is m*nb
  }
  work_sz = (nb != -1) ? nb * m : 1;
  tot_sz = (a_sz + work_sz) * sizeof(LM_REAL) + ipiv_sz * sizeof(int); /* should be arranged in that order for proper doubles alignment */

  buf.resize(tot_sz);
  a = buf.data();
  work = a + a_sz;
  ipiv = (int *) (work + work_sz);

  /* store A into a and B into x; A is assumed to be symmetric, hence
   * the column and row major order representations are the same
   */
  memcpy(a, A, a_sz * sizeof(LM_REAL));
  memcpy(x, B, m * sizeof(LM_REAL));

  /* LDLt factorization for A */
  SYTRF("L", (int *) &m, a, (int *) &m, ipiv, work, (int *) &work_sz, (int *) &info);
  if ( info != 0 ) {
    if ( info < 0 ) {
      CF_ERROR("LAPACK error: illegal value for argument %d of SYTRF", -info);
      return false;
    }
    else {
      CF_ERROR("LAPACK error: singular block diagonal matrix D for SYTRF() [D(%d, %d) is zero]", info, info);
      return false;
    }
  }

  /* solve the system with the computed factorization */
  SYTRS("L", (int *) &m, (int *) &nrhs, a, (int *) &m, ipiv, x, (int *) &m, (int *) &info);
  if ( info < 0 ) {
    CF_ERROR("LAPACK error: illegal value for argument %d of SYTRS()", -info);
    return false;
  }

  return true;
}



#if HAVE_LAPACK
/*
 * This function computes the pseudoinverse of a square matrix A
 * into B using SVD. A and B can coincide
 *
 * The function returns 0 in case of error (e.g. A is singular),
 * the rank of A if successful
 *
 * A, B are mxm
 *
 */
template<class LM_REAL>
static int LEVMAR_PSEUDOINVERSE(LM_REAL *A, LM_REAL *B, int m)
{
  std::vector<LM_REAL> buf;
  //int buf_sz = 0;
  static LM_REAL eps = -1.0;

  int i, j;
  LM_REAL *a, *u, *s, *vt, *work;
  int a_sz, u_sz, s_sz, vt_sz, tot_sz;
  LM_REAL thresh, one_over_denom;
  int info, rank, worksz, *iwork, iworksz;

  /* calculate required memory size */
  worksz = 5 * m;  // min worksize for GESVD
  //worksz=m*(7*m+4); // min worksize for GESDD
  iworksz = 8 * m;
  a_sz = m * m;
  u_sz = m * m;
  s_sz = m;
  vt_sz = m * m;

  /* should be arranged in that order for proper doubles alignment */
  tot_sz = (a_sz + u_sz + s_sz + vt_sz + worksz) * sizeof(LM_REAL) + iworksz * sizeof(int);

  buf.resize(tot_sz);
  a = buf.data();
  u = a + a_sz;
  s = u + u_sz;
  vt = s + s_sz;
  work = vt + vt_sz;
  iwork = (int *) (work + worksz);

  /* store A (column major!) into a */
  for ( i = 0; i < m; i++ )
    for ( j = 0; j < m; j++ )
      a[i + j * m] = A[i * m + j];

  /* SVD decomposition of A */
  GESVD("A", "A", &m,  &m, a, &m, s, u, &m, vt, &m, work, &worksz, &info);
  //GESDD("A", (int *)&m, (int *)&m, a, (int *)&m, s, u, (int *)&m, vt, (int *)&m, work, (int *)&worksz, iwork, &info);

  /* error treatment */
  if ( info != 0 ) {
    if ( info < 0 ) {
      CF_ERROR("LAPACK error: illegal value for argument %d of GESVD / GESDD)", -info);
    }
    else {
      CF_ERROR("LAPACK error: dgesdd (dbdsdc)/dgesvd (dbdsqr) failed to converge in [info=%d]", info);
    }
    return 0;
  }

  if ( eps < 0.0 ) {
    LM_REAL aux;

    /* compute machine epsilon */
    for ( eps = 1.0; aux = eps + 1.0, aux - 1.0 > 0.0; eps *= 0.5 )
      ;
    eps *= 2.0;
  }

  /* compute the pseudoinverse in B */
  for ( i = 0; i < a_sz; i++ )
    B[i] = 0.0; /* initialize to zero */

  for ( rank = 0, thresh = eps * s[0]; rank < m && s[rank] > thresh; rank++ ) {
    one_over_denom = 1.0 / s[rank];

    for ( j = 0; j < m; j++ )
      for ( i = 0; i < m; i++ )
        B[i * m + j] += vt[rank + i * m] * u[j + rank * m] * one_over_denom;
  }


  return rank;
}
#else // no LAPACK

/*
 * This function computes the inverse of A in B. A and B can coincide
 *
 * The function employs LAPACK-free LU decomposition of A to solve m linear
 * systems A*B_i=I_i, where B_i and I_i are the i-th columns of B and I.
 *
 * A and B are mxm
 *
 * The function returns 0 in case of error, 1 if successful
 *
 */
template<class LM_REAL>
static int LEVMAR_LUINVERSE(LM_REAL *A, LM_REAL *B, int m)
{
  void *buf=NULL;
  int buf_sz=0;

  register int i, j, k, l;
  int *idx, maxi=-1, idx_sz, a_sz, x_sz, work_sz, tot_sz;
  LM_REAL *a, *x, *work, max, sum, tmp;

  /* calculate required memory size */
  idx_sz=m;
  a_sz=m*m;
  x_sz=m;
  work_sz=m;
  tot_sz=(a_sz + x_sz + work_sz)*sizeof(LM_REAL) + idx_sz*sizeof(int); /* should be arranged in that order for proper doubles alignment */

  buf_sz=tot_sz;
  buf=(void *)malloc(tot_sz);
  if(!buf) {
    CF_ERROR("memory allocation failed");
    return 0; /* error */
  }

  a=buf;
  x=a+a_sz;
  work=x+x_sz;
  idx=(int *)(work+work_sz);

  /* avoid destroying A by copying it to a */
  for(i=0; i<a_sz; ++i) a[i]=A[i];

  /* compute the LU decomposition of a row permutation of matrix a; the permutation itself is saved in idx[] */
  for(i=0; i<m; ++i) {
    max=0.0;
    for(j=0; j<m; ++j)
    if((tmp=FABS(a[i*m+j]))>max)
    max=tmp;
    if(max==0.0) {
      CF_ERROR("Singular matrix A");
      free(buf);

      return 0;
    }
    work[i]=LM_CNST(1.0)/max;
  }

  for(j=0; j<m; ++j) {
    for(i=0; i<j; ++i) {
      sum=a[i*m+j];
      for(k=0; k<i; ++k)
      sum-=a[i*m+k]*a[k*m+j];
      a[i*m+j]=sum;
    }
    max=0.0;
    for(i=j; i<m; ++i) {
      sum=a[i*m+j];
      for(k=0; k<j; ++k)
      sum-=a[i*m+k]*a[k*m+j];
      a[i*m+j]=sum;
      if((tmp=work[i]*FABS(sum))>=max) {
        max=tmp;
        maxi=i;
      }
    }
    if(j!=maxi) {
      for(k=0; k<m; ++k) {
        tmp=a[maxi*m+k];
        a[maxi*m+k]=a[j*m+k];
        a[j*m+k]=tmp;
      }
      work[maxi]=work[j];
    }
    idx[j]=maxi;
    if(a[j*m+j]==0.0)
    a[j*m+j]=LM_REAL_EPSILON;
    if(j!=m-1) {
      tmp=LM_CNST(1.0)/(a[j*m+j]);
      for(i=j+1; i<m; ++i)
      a[i*m+j]*=tmp;
    }
  }

  /* The decomposition has now replaced a. Solve the m linear systems using
   * forward and back substitution
   */
  for(l=0; l<m; ++l) {
    for(i=0; i<m; ++i) x[i]=0.0;
    x[l]=LM_CNST(1.0);

    for(i=k=0; i<m; ++i) {
      j=idx[i];
      sum=x[j];
      x[j]=x[i];
      if(k!=0)
      for(j=k-1; j<i; ++j)
      sum-=a[i*m+j]*x[j];
      else
      if(sum!=0.0)
      k=i+1;
      x[i]=sum;
    }

    for(i=m-1; i>=0; --i) {
      sum=x[i];
      for(j=i+1; j<m; ++j)
      sum-=a[i*m+j]*x[j];
      x[i]=sum/a[i*m+i];
    }

    for(i=0; i<m; ++i)
    B[i*m+l]=x[i];
  }

  free(buf);

  return 1;
}
#endif /* HAVE_LAPACK */

/*
 * This function computes in C the covariance matrix corresponding to a least
 * squares fit. JtJ is the approximate Hessian at the solution (i.e. J^T*J, where
 * J is the Jacobian at the solution), sumsq is the sum of squared residuals
 * (i.e. goodnes of fit) at the solution, m is the number of parameters (variables)
 * and n the number of observations. JtJ can coincide with C.
 *
 * if JtJ is of full rank, C is computed as sumsq/(n-m)*(JtJ)^-1
 * otherwise and if LAPACK is available, C=sumsq/(n-r)*(JtJ)^+
 * where r is JtJ's rank and ^+ denotes the pseudoinverse
 * The diagonal of C is made up from the estimates of the variances
 * of the estimated regression coefficients.
 * See the documentation of routine E04YCF from the NAG fortran lib
 *
 * The function returns the rank of JtJ if successful, 0 on error
 *
 * A and C are mxm
 *
 */
template<class LM_REAL>
static int LEVMAR_COVAR(LM_REAL *JtJ, LM_REAL *C, LM_REAL sumsq, int m, int n)
{
  int i;
  int rnk;
  LM_REAL fact;

#if HAVE_LAPACK
  rnk = LEVMAR_PSEUDOINVERSE(JtJ, C, m);
  if ( !rnk )
    return 0;
#else
#ifdef _MSC_VER
#pragma message("LAPACK not available, LU will be used for matrix inversion when computing the covariance; this might be unstable at times")
#else
#warning LAPACK not available, LU will be used for matrix inversion when computing the covariance; this might be unstable at times
#endif // _MSC_VER

  rnk=LEVMAR_LUINVERSE(JtJ, C, m);
  if(!rnk) return 0;

  rnk=m; /* assume full rank */
#endif /* HAVE_LAPACK */

  fact = sumsq / (LM_REAL) (n - rnk);
  for ( i = 0; i < m * m; ++i )
    C[i] *= fact;

  return rnk;
}

/* forward finite difference approximation to the Jacobian of func */
template<class LM_REAL>
static void LEVMAR_FDIF_FORW_JAC_APPROX(
    void (*func)(const LM_REAL p[], LM_REAL *hx, int m, int n, void *adata),
    /* function to differentiate */
    LM_REAL *p, /* I: current parameter estimate, mx1 */
    LM_REAL *hx, /* I: func evaluated at p, i.e. hx=func(p), nx1 */
    LM_REAL *hxx, /* W/O: work array for evaluating func(p+delta), nx1 */
    LM_REAL delta, /* increment for computing the Jacobian */
    LM_REAL *jac, /* O: array for storing approximated Jacobian, nxm */
    int m,
    int n,
    void *adata)
{
  int i, j;
  LM_REAL tmp;
  LM_REAL d;

  for ( j = 0; j < m; ++j ) {
    /* determine d=max(1E-04*|p[j]|, delta), see HZ */
    d = 1E-04 * p[j];  // force evaluation
    d = abs(d);
    if ( d < delta )
      d = delta;

    tmp = p[j];
    p[j] += d;

    (*func)(p, hxx, m, n, adata);

    p[j] = tmp; /* restore */

    d = 1.0 / d; /* invert so that divisions can be carried out faster as multiplications */
    for ( i = 0; i < n; ++i ) {
      jac[i * m + j] = (hxx[i] - hx[i]) * d;
    }
  }
}

/* central finite difference approximation to the Jacobian of func */
template<class LM_REAL>
static void LEVMAR_FDIF_CENT_JAC_APPROX(
    void (*func)(const LM_REAL p[], LM_REAL *hx, int m, int n, void *adata),
    /* function to differentiate */
    LM_REAL *p, /* I: current parameter estimate, mx1 */
    LM_REAL *hxm, /* W/O: work array for evaluating func(p-delta), nx1 */
    LM_REAL *hxp, /* W/O: work array for evaluating func(p+delta), nx1 */
    LM_REAL delta, /* increment for computing the Jacobian */
    LM_REAL *jac, /* O: array for storing approximated Jacobian, nxm */
    int m,
    int n,
    void *adata)
{
  int i, j;
  LM_REAL tmp;
  LM_REAL d;

  for ( j = 0; j < m; ++j ) {
    /* determine d=max(1E-04*|p[j]|, delta), see HZ */
    d = 1E-04 * p[j];  // force evaluation
    d = abs(d);
    if ( d < delta )
      d = delta;

    tmp = p[j];
    p[j] -= d;
    (*func)(p, hxm, m, n, adata);

    p[j] = tmp + d;
    (*func)(p, hxp, m, n, adata);
    p[j] = tmp; /* restore */

    d = 0.5 / d; /* invert so that divisions can be carried out faster as multiplications */
    for ( i = 0; i < n; ++i ) {
      jac[i * m + j] = (hxp[i] - hxm[i]) * d;
    }
  }
}


/*
 * This function seeks the parameter vector p that best describes the measurements vector x.
 * More precisely, given a vector function  func : R^m --> R^n with n>=m,
 * it finds p s.t. func(p) ~= x, i.e. the squared second order (i.e. L2) norm of
 * e=x-func(p) is minimized.
 *
 * This function requires an analytic Jacobian. In case the latter is unavailable,
 * use LEVMAR_DIF() bellow
 *
 * Returns the number of iterations (>=0) if successful, LM_ERROR if failed
 *
 * For more details, see K. Madsen, H.B. Nielsen and O. Tingleff's lecture notes on
 * non-linear least squares at http://www.imm.dtu.dk/pubdb/views/edoc_download.php/3215/pdf/imm3215.pdf
 */

template<typename LM_REAL>
static int levmar_der_(
  void (*func)(const LM_REAL *p, LM_REAL *hx, int m, int n, void *adata), /* functional relation describing measurements. A p \in R^m yields a \hat{x} \in  R^n */
  void (*jacf)(const LM_REAL *p, LM_REAL *j, int m, int n, void *adata),  /* function to evaluate the Jacobian \part x / \part p */
  LM_REAL *p,         /* I/O: initial parameter estimates. On output has the estimated solution */
  LM_REAL *x,         /* I: measurement vector. NULL implies a zero vector */
  int m,              /* I: parameter vector dimension (i.e. #unknowns) */
  int n,              /* I: measurement vector dimension */
  int itmax,          /* I: maximum number of iterations */
  const c_levmar_opts & opts,
  c_levmar_status & info,
  LM_REAL *work,     /* working memory at least LM_DER_WORKSZ() reals large, allocated if NULL */
  LM_REAL *covar,    /* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
  void *adata)       /* pointer to possibly additional data, passed uninterpreted to func & jacf.
                      * Set to NULL if not needed
                      */
{
  constexpr LM_REAL LM_REAL_MAX =
      std::numeric_limits<LM_REAL>::max();

  constexpr LM_REAL LM_REAL_MIN =
      -LM_REAL_MAX;

  int i, j, k, l;
  int worksz, freework = 0, issolved;

  /* temp work arrays */
  LM_REAL *e, /* nx1 */
    *hx, /* \hat{x}_i, nx1 */
  *jacTe, /* J^T e_i mx1 */
  *jac, /* nxm */
  *jacTjac, /* mxm */
  *Dp, /* mx1 */
  *diag_jacTjac, /* diagonal of J^T J, mx1 */
  *pDp; /* p + Dp, mx1 */

  LM_REAL mu, /* damping constant */
    tmp; /* mainly used in matrix & vector multiplications */
  LM_REAL p_eL2, jacTe_inf, pDp_eL2; /* ||e(p)||_2, ||J^T e||_inf, ||e(p+Dp)||_2 */
  LM_REAL p_L2, Dp_L2 = LM_REAL_MAX, dF, dL;
  LM_REAL tau, eps1, eps2, eps2_sq, eps3;
  LM_REAL init_p_eL2;
  int nu = 2, nu2, stop = 0, nfev, njev = 0, nlss = 0;
  const int nm = n * m;
  bool (*linsolver)(LM_REAL *A, LM_REAL *B, LM_REAL *x, int m)=NULL;

  mu = jacTe_inf = 0.0; /* -Wall */

  if ( n < m ) {
    CF_ERROR("cannot solve a problem with fewer measurements [%d] than unknowns [%d]", n, m);
    return LM_ERROR;
  }

  if ( !jacf ) {
    CF_ERROR("No function specified for computing the Jacobian.\n"
        "If no such function is available, use  levmar_dif  rather than levmar_der()");
    return LM_ERROR;
  }

  tau = opts.mu;
  eps1 = opts.eps1;
  eps2 = opts.eps2;
  eps2_sq = opts.eps2 * opts.eps2;
  eps3 = opts.eps3;

  if ( !work ) {
    worksz = LM_DER_WORKSZ(m, n);  //2*n+4*m + n*m + m*m;
    work = (LM_REAL *) malloc(worksz * sizeof(LM_REAL)); /* allocate a big chunk in one step */
    if ( !work ) {
      CF_ERROR("memory allocation request failed");
      return LM_ERROR;
    }
    freework = 1;
  }

  /* set up work arrays */
  e = work;
  hx = e + n;
  jacTe = hx + n;
  jac = jacTe + m;
  jacTjac = jac + nm;
  Dp = jacTjac + m * m;
  diag_jacTjac = Dp + m;
  pDp = diag_jacTjac + m;

  /* compute e=x - f(p) and its L2 norm */
  (*func)(p, hx, m, n, adata);
  nfev = 1;
  /* ### e=x-hx, p_eL2=||e|| */
#if 1
  p_eL2 = LEVMAR_L2NRMXMY(e, x, hx, n);
#else
  for(i=0, p_eL2=0.0; i<n; ++i) {
    e[i]=tmp=x[i]-hx[i];
    p_eL2+=tmp*tmp;
  }
#endif
  init_p_eL2 = p_eL2;
  if ( !LM_FINITE(p_eL2) )
    stop = 7;

  for ( k = 0; k < itmax && !stop; ++k ) {
    /* Note that p and e have been updated at a previous iteration */

    if ( p_eL2 <= eps3 ) { /* error is small */
      stop = 6;
      break;
    }

    /* Compute the Jacobian J at p,  J^T J,  J^T e,  ||J^T e||_inf and ||p||^2.
     * Since J^T J is symmetric, its computation can be sped up by computing
     * only its upper triangular part and copying it to the lower part
     */

    (*jacf)(p, jac, m, n, adata);
    ++njev;

    /* J^T J, J^T e */
    if ( nm < __BLOCKSZ__SQ ) {  // this is a small problem
      /* J^T*J_ij = \sum_l J^T_il * J_lj = \sum_l J_li * J_lj.
       * Thus, the product J^T J can be computed using an outer loop for
       * l that adds J_li*J_lj to each element ij of the result. Note that
       * with this scheme, the accesses to J and JtJ are always along rows,
       * therefore induces less cache misses compared to the straightforward
       * algorithm for computing the product (i.e., l loop is innermost one).
       * A similar scheme applies to the computation of J^T e.
       * However, for large minimization problems (i.e., involving a large number
       * of unknowns and measurements) for which J/J^T J rows are too large to
       * fit in the L1 cache, even this scheme incures many cache misses. In
       * such cases, a cache-efficient blocking scheme is preferable.
       *
       * Thanks to John Nitao of Lawrence Livermore Lab for pointing out this
       * performance problem.
       *
       * Note that the non-blocking algorithm is faster on small
       * problems since in this case it avoids the overheads of blocking.
       */

      /* looping downwards saves a few computations */
      int l;
      LM_REAL alpha, *jaclm, *jacTjacim;

      for ( i = m * m; i-- > 0; )
        jacTjac[i] = 0.0;
      for ( i = m; i-- > 0; )
        jacTe[i] = 0.0;

      for ( l = n; l-- > 0; ) {
        jaclm = jac + l * m;
        for ( i = m; i-- > 0; ) {
          jacTjacim = jacTjac + i * m;
          alpha = jaclm[i];  //jac[l*m+i];
          for ( j = i + 1; j-- > 0; ) /* j<=i computes lower triangular part only */
            jacTjacim[j] += jaclm[j] * alpha;  //jacTjac[i*m+j]+=jac[l*m+j]*alpha

          /* J^T e */
          jacTe[i] += alpha * e[l];
        }
      }

      for ( i = m; i-- > 0; ) /* copy to upper part */
        for ( j = i + 1; j < m; ++j )
          jacTjac[i * m + j] = jacTjac[j * m + i];

    }
    else {  // this is a large problem
      /* Cache efficient computation of J^T J based on blocking
       */
      LEVMAR_TRANS_MAT_MAT_MULT(jac, jacTjac, n, m);

      /* cache efficient computation of J^T e */
      for ( i = 0; i < m; ++i )
        jacTe[i] = 0.0;

      for ( i = 0; i < n; ++i ) {
        LM_REAL *jacrow;

        for ( l = 0, jacrow = jac + i * m, tmp = e[i]; l < m; ++l )
          jacTe[l] += jacrow[l] * tmp;
      }
    }

    /* Compute ||J^T e||_inf and ||p||^2 */
    for ( i = 0, p_L2 = jacTe_inf = 0.0; i < m; ++i ) {
      if ( jacTe_inf < (tmp = abs(jacTe[i])) )
        jacTe_inf = tmp;

      diag_jacTjac[i] = jacTjac[i * m + i]; /* save diagonal entries so that augmentation can be later canceled */
      p_L2 += p[i] * p[i];
    }
    //p_L2=sqrt(p_L2);

#if 0
    if(!(k%100)) {
      printf("Current estimate: ");
      for(i=0; i<m; ++i)
      printf("%.9g ", p[i]);
      printf("-- errors %.9g %0.9g\n", jacTe_inf, p_eL2);
    }
#endif

    /* check for convergence */
    if ( (jacTe_inf <= eps1) ) {
      Dp_L2 = 0.0; /* no increment for p in this case */
      stop = 1;
      break;
    }

    /* compute initial damping factor */
    if ( k == 0 ) {
      for ( i = 0, tmp = LM_REAL_MIN; i < m; ++i )
        if ( diag_jacTjac[i] > tmp )
          tmp = diag_jacTjac[i]; /* find max diagonal element */
      mu = tau * tmp;
    }

    /* determine increment using adaptive damping */
    while ( 1 ) {
      /* augment normal equations */
      for ( i = 0; i < m; ++i )
        jacTjac[i * m + i] += mu;

      /* solve augmented equations */
#if HAVE_LAPACK
      /* 7 alternatives are available: LU, Cholesky + Cholesky with PLASMA, LDLt, 2 variants of QR decomposition and SVD.
       * For matrices with dimensions of at least a few hundreds, the PLASMA implementation of Cholesky is the fastest.
       * From the serial solvers, Cholesky is the fastest but might occasionally be inapplicable due to numerical round-off;
       * QR is slower but more robust; SVD is the slowest but most robust; LU is quite robust but
       * slower than LDLt; LDLt offers a good tradeoff between robustness and speed
       */

      issolved = AX_EQ_B_BK(jacTjac, jacTe, Dp, m);
      ++nlss;
      linsolver = &AX_EQ_B_BK<LM_REAL>;
      //issolved=AX_EQ_B_LU(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_LU;
      //issolved=AX_EQ_B_CHOL(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_CHOL;
#ifdef HAVE_PLASMA
      //issolved=AX_EQ_B_PLASMA_CHOL(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_PLASMA_CHOL;
#endif
      //issolved=AX_EQ_B_QR(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_QR;
      //issolved=AX_EQ_B_QRLS(jacTjac, jacTe, Dp, m, m); ++nlss; linsolver=(int (*)(LM_REAL *A, LM_REAL *B, LM_REAL *x, int m))AX_EQ_B_QRLS;
      //issolved=AX_EQ_B_SVD(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_SVD;

#else
      /* use the LU included with levmar */
      issolved=AX_EQ_B_LU(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_LU;
#endif /* HAVE_LAPACK */

      if ( issolved ) {
        /* compute p's new estimate and ||Dp||^2 */
        for ( i = 0, Dp_L2 = 0.0; i < m; ++i ) {
          pDp[i] = p[i] + (tmp = Dp[i]);
          Dp_L2 += tmp * tmp;
        }
        //Dp_L2=sqrt(Dp_L2);

        if ( Dp_L2 <= eps2_sq * p_L2 ) { /* relative change in p is small, stop */
          //if(Dp_L2<=eps2*(p_L2 + eps2)){ /* relative change in p is small, stop */
          stop = 2;
          break;
        }

        if ( Dp_L2 >= (p_L2 + eps2) / (EPSILON * EPSILON) ) { /* almost singular */
          //if(Dp_L2>=(p_L2+eps2)/LM_CNST(EPSILON)){ /* almost singular */
          stop = 4;
          break;
        }

        (*func)(pDp, hx, m, n, adata);
        ++nfev; /* evaluate function at p + Dp */
        /* compute ||e(pDp)||_2 */
        /* ### hx=x-hx, pDp_eL2=||hx|| */
#if 1
        pDp_eL2 = LEVMAR_L2NRMXMY(hx, x, hx, n);
#else
        for(i=0, pDp_eL2=0.0; i<n; ++i) {
          hx[i]=tmp=x[i]-hx[i];
          pDp_eL2+=tmp*tmp;
        }
#endif
        if ( !LM_FINITE(pDp_eL2) ) { /* sum of squares is not finite, most probably due to a user error.
         * This check makes sure that the inner loop does not run indefinitely.
         * Thanks to Steve Danauskas for reporting such cases
         */
          stop = 7;
          break;
        }

        for ( i = 0, dL = 0.0; i < m; ++i )
          dL += Dp[i] * (mu * Dp[i] + jacTe[i]);

        dF = p_eL2 - pDp_eL2;

        if ( dL > 0.0 && dF > 0.0 ) { /* reduction in error, increment is accepted */
          tmp = (2.0 * dF / dL - 1.0);
          tmp = 1.0 - tmp * tmp * tmp;
          mu = mu * ((tmp >= ONE_THIRD) ? tmp : ONE_THIRD);
          nu = 2;

          for ( i = 0; i < m; ++i ) /* update p's estimate */
            p[i] = pDp[i];

          for ( i = 0; i < n; ++i ) /* update e and ||e||_2 */
            e[i] = hx[i];
          p_eL2 = pDp_eL2;
          break;
        }
      }

      /* if this point is reached, either the linear system could not be solved or
       * the error did not reduce; in any case, the increment must be rejected
       */

      mu *= nu;
      nu2 = nu << 1;  // 2*nu;
      if ( nu2 <= nu ) { /* nu has wrapped around (overflown). Thanks to Frank Jordan for spotting this case */
        stop = 5;
        break;
      }
      nu = nu2;

      for ( i = 0; i < m; ++i ) /* restore diagonal J^T J entries */
        jacTjac[i * m + i] = diag_jacTjac[i];
    } /* inner loop */
  }

  if ( k >= itmax )
    stop = 3;

  for ( i = 0; i < m; ++i ) /* restore diagonal J^T J entries */
    jacTjac[i * m + i] = diag_jacTjac[i];

  // fill info
  if ( true ) {
    info.eps3_initial = init_p_eL2; // info[0] = init_p_eL2;
    info.eps3 = p_eL2; //  info[1] = p_eL2;
    info.eps1 = jacTe_inf; // info[2] = jacTe_inf;
    info.eps2 = Dp_L2; // info[3] = Dp_L2;
    for ( i = 0, tmp = LM_REAL_MIN; i < m; ++i ) {
      if ( tmp < jacTjac[i * m + i] ) {
        tmp = jacTjac[i * m + i];
      }
    }
    info.mu = mu / tmp; // info[4] = mu / tmp;
    info.iters = k; // info[5] = (LM_REAL) k;
    info.reason = (enum levmar_stopping_reason)stop; // info[6] = (LM_REAL) stop;
    info.nfev = nfev;// info[7] = (LM_REAL) nfev;
    info.njev = njev;// info[8] = (LM_REAL) njev;
    info.nlss = nlss;// info[9] = (LM_REAL) nlss;
  }

  /* covariance matrix */
  if ( covar ) {
    LEVMAR_COVAR(jacTjac, covar, p_eL2, m, n);
  }

  if ( freework )
    free(work);

  return (stop != 4 && stop != 7) ? k : LM_ERROR;
}


/* Secant version of the LEVMAR_DER() function above: the Jacobian is approximated with
 * the aid of finite differences (forward or central, see the comment for the opts argument)
 */
template<typename LM_REAL>
static int levmar_dif_(
  void (*func)(const LM_REAL *p, LM_REAL *hx, int m, int n, void *adata), /* functional relation describing measurements. A p \in R^m yields a \hat{x} \in  R^n */
  LM_REAL *p,         /* I/O: initial parameter estimates. On output has the estimated solution */
  LM_REAL *x,         /* I: measurement vector. NULL implies a zero vector */
  int m,              /* I: parameter vector dimension (i.e. #unknowns) */
  int n,              /* I: measurement vector dimension */
  int itmax,          /* I: maximum number of iterations */
  const c_levmar_opts & opts,
  c_levmar_status & info,
  LM_REAL *work,     /* working memory at least LM_DIF_WORKSZ() reals large, allocated if NULL */
  LM_REAL *covar,    /* O: Covariance matrix corresponding to LS solution; mxm. Set to NULL if not needed. */
  void *adata)       /* pointer to possibly additional data, passed uninterpreted to func.
                      * Set to NULL if not needed
                      */
{
  INSTRUMENT_REGION("");

  constexpr LM_REAL LM_REAL_MAX =
      std::numeric_limits<LM_REAL>::max();

  constexpr LM_REAL LM_REAL_MIN =
      -LM_REAL_MAX;

  int i, j, k, l;
  int worksz, freework = 0, issolved;
  /* temp work arrays */
  LM_REAL *e, /* nx1 */
  *hx, /* \hat{x}_i, nx1 */
  *jacTe, /* J^T e_i mx1 */
  *jac, /* nxm */
  *jacTjac, /* mxm */
  *Dp, /* mx1 */
  *diag_jacTjac, /* diagonal of J^T J, mx1 */
  *pDp, /* p + Dp, mx1 */
  *wrk, /* nx1 */
  *wrk2; /* nx1, used only for holding a temporary e vector and when differentiating with central differences */

  int using_ffdif = 1;

  LM_REAL mu, /* damping constant */
    tmp; /* mainly used in matrix & vector multiplications */
  LM_REAL p_eL2, jacTe_inf, pDp_eL2; /* ||e(p)||_2, ||J^T e||_inf, ||e(p+Dp)||_2 */
  LM_REAL p_L2, Dp_L2 = LM_REAL_MAX, dF, dL;
  LM_REAL tau, eps1, eps2, eps2_sq, eps3, delta;
  LM_REAL init_p_eL2;
  int nu, nu2, stop = 0, nfev, njap = 0, nlss = 0, K = (m >= 10) ? m : 10, updjac, updp = 1, newjac;
  const int nm = n * m;
  bool (*linsolver)(LM_REAL *A, LM_REAL *B, LM_REAL *x, int m)=NULL;

  mu = jacTe_inf = p_L2 = 0.0; /* -Wall */
  updjac = newjac = 0; /* -Wall */

  if ( n < m ) {
    CF_ERROR("cannot solve a problem with fewer measurements [%d] than unknowns [%d]", n, m);
    return LM_ERROR;
  }

  tau = opts.mu;
  eps1 = opts.eps1;
  eps2 = opts.eps2;
  eps2_sq = opts.eps2 * opts.eps2;
  eps3 = opts.eps3;
  delta = opts.delta;
  if ( delta < 0 ) {
    delta = -delta; /* make positive */
    using_ffdif = 0; /* use central differencing */
  }
  else if ( delta == 0 ) {
    delta = LM_DIFF_DELTA;
  }

  if ( !work ) {
    worksz = LM_DIF_WORKSZ(m, n);  //4*n+4*m + n*m + m*m;
    work = (LM_REAL *) malloc(worksz * sizeof(LM_REAL)); /* allocate a big chunk in one step */
    if ( !work ) {
      CF_ERROR("memory allocation request failed");
      return LM_ERROR;
    }
    freework = 1;
  }

  /* set up work arrays */
  e = work;
  hx = e + n;
  jacTe = hx + n;
  jac = jacTe + m;
  jacTjac = jac + nm;
  Dp = jacTjac + m * m;
  diag_jacTjac = Dp + m;
  pDp = diag_jacTjac + m;
  wrk = pDp + m;
  wrk2 = wrk + n;

  /* compute e=x - f(p) and its L2 norm */
  (*func)(p, hx, m, n, adata);
  nfev = 1;
  /* ### e=x-hx, p_eL2=||e|| */
#if 1
  p_eL2 = LEVMAR_L2NRMXMY(e, x, hx, n);
#else
  for(i=0, p_eL2=0.0; i<n; ++i) {
    e[i]=tmp=x[i]-hx[i];
    p_eL2+=tmp*tmp;
  }
#endif
  init_p_eL2 = p_eL2;
  if ( !LM_FINITE(p_eL2) )
    stop = 7;

  nu = 20; /* force computation of J */

  for ( k = 0; k < itmax && !stop; ++k ) {
    /* Note that p and e have been updated at a previous iteration */

    if ( p_eL2 <= eps3 ) { /* error is small */
      stop = 6;
      break;
    }

    /* Compute the Jacobian J at p,  J^T J,  J^T e,  ||J^T e||_inf and ||p||^2.
     * The symmetry of J^T J is again exploited for speed
     */

    if ( (updp && nu > 16) || updjac == K ) { /* compute difference approximation to J */
      if ( using_ffdif ) { /* use forward differences */
        LEVMAR_FDIF_FORW_JAC_APPROX(func, p, hx, wrk, delta, jac, m, n, adata);
        ++njap;
        nfev += m;
      }
      else { /* use central differences */
        LEVMAR_FDIF_CENT_JAC_APPROX(func, p, wrk, wrk2, delta, jac, m, n, adata);
        ++njap;
        nfev += 2 * m;
      }
      nu = 2;
      updjac = 0;
      updp = 0;
      newjac = 1;
    }

    if ( newjac ) { /* Jacobian has changed, recompute J^T J, J^t e, etc */
      newjac = 0;

      /* J^T J, J^T e */
      if ( nm <= __BLOCKSZ__SQ ) {  // this is a small problem
        /* J^T*J_ij = \sum_l J^T_il * J_lj = \sum_l J_li * J_lj.
         * Thus, the product J^T J can be computed using an outer loop for
         * l that adds J_li*J_lj to each element ij of the result. Note that
         * with this scheme, the accesses to J and JtJ are always along rows,
         * therefore induces less cache misses compared to the straightforward
         * algorithm for computing the product (i.e., l loop is innermost one).
         * A similar scheme applies to the computation of J^T e.
         * However, for large minimization problems (i.e., involving a large number
         * of unknowns and measurements) for which J/J^T J rows are too large to
         * fit in the L1 cache, even this scheme incures many cache misses. In
         * such cases, a cache-efficient blocking scheme is preferable.
         *
         * Thanks to John Nitao of Lawrence Livermore Lab for pointing out this
         * performance problem.
         *
         * Note that the non-blocking algorithm is faster on small
         * problems since in this case it avoids the overheads of blocking.
         */
        int l;
        LM_REAL alpha, *jaclm, *jacTjacim;

        /* looping downwards saves a few computations */
        for ( i = m * m; i-- > 0; )
          jacTjac[i] = 0.0;
        for ( i = m; i-- > 0; )
          jacTe[i] = 0.0;

        for ( l = n; l-- > 0; ) {
          jaclm = jac + l * m;
          for ( i = m; i-- > 0; ) {
            jacTjacim = jacTjac + i * m;
            alpha = jaclm[i];  //jac[l*m+i];
            for ( j = i + 1; j-- > 0; ) /* j<=i computes lower triangular part only */
              jacTjacim[j] += jaclm[j] * alpha;  //jacTjac[i*m+j]+=jac[l*m+j]*alpha

            /* J^T e */
            jacTe[i] += alpha * e[l];
          }
        }

        for ( i = m; i-- > 0; ) /* copy to upper part */
          for ( j = i + 1; j < m; ++j )
            jacTjac[i * m + j] = jacTjac[j * m + i];
      }
      else {  // this is a large problem
        /* Cache efficient computation of J^T J based on blocking
         */
        LEVMAR_TRANS_MAT_MAT_MULT(jac, jacTjac, n, m);

        /* cache efficient computation of J^T e */
        for ( i = 0; i < m; ++i )
          jacTe[i] = 0.0;

        for ( i = 0; i < n; ++i ) {
          LM_REAL *jacrow;

          for ( l = 0, jacrow = jac + i * m, tmp = e[i]; l < m; ++l )
            jacTe[l] += jacrow[l] * tmp;
        }
      }

      /* Compute ||J^T e||_inf and ||p||^2 */
      for ( i = 0, p_L2 = jacTe_inf = 0.0; i < m; ++i ) {
        if ( jacTe_inf < (tmp = abs(jacTe[i])) )
          jacTe_inf = tmp;

        diag_jacTjac[i] = jacTjac[i * m + i]; /* save diagonal entries so that augmentation can be later canceled */
        p_L2 += p[i] * p[i];
      }
      //p_L2=sqrt(p_L2);
    }

#if 0
    if(!(k%100)) {
      printf("Current estimate: ");
      for(i=0; i<m; ++i)
      printf("%.9g ", p[i]);
      printf("-- errors %.9g %0.9g\n", jacTe_inf, p_eL2);
    }
#endif

    /* check for convergence */
    if ( (jacTe_inf <= eps1) ) {
      Dp_L2 = 0.0; /* no increment for p in this case */
      stop = 1;
      break;
    }

    /* compute initial damping factor */
    if ( k == 0 ) {
      for ( i = 0, tmp = LM_REAL_MIN; i < m; ++i )
        if ( diag_jacTjac[i] > tmp )
          tmp = diag_jacTjac[i]; /* find max diagonal element */
      mu = tau * tmp;
    }

    /* determine increment using adaptive damping */

    /* augment normal equations */
    for ( i = 0; i < m; ++i )
      jacTjac[i * m + i] += mu;

    /* solve augmented equations */
#if HAVE_LAPACK
    /* 7 alternatives are available: LU, Cholesky + Cholesky with PLASMA, LDLt, 2 variants of QR decomposition and SVD.
     * For matrices with dimensions of at least a few hundreds, the PLASMA implementation of Cholesky is the fastest.
     * From the serial solvers, Cholesky is the fastest but might occasionally be inapplicable due to numerical round-off;
     * QR is slower but more robust; SVD is the slowest but most robust; LU is quite robust but
     * slower than LDLt; LDLt offers a good tradeoff between robustness and speed
     */

    issolved = AX_EQ_B_BK(jacTjac, jacTe, Dp, m);
    ++nlss;
    linsolver = &AX_EQ_B_BK<LM_REAL>;
    //issolved=AX_EQ_B_LU(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_LU;
    //issolved=AX_EQ_B_CHOL(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_CHOL;
#ifdef HAVE_PLASMA
    //issolved=AX_EQ_B_PLASMA_CHOL(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_PLASMA_CHOL;
#endif
    //issolved=AX_EQ_B_QR(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_QR;
    //issolved=AX_EQ_B_QRLS(jacTjac, jacTe, Dp, m, m); ++nlss; linsolver=(int (*)(LM_REAL *A, LM_REAL *B, LM_REAL *x, int m))AX_EQ_B_QRLS;
    //issolved=AX_EQ_B_SVD(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_SVD;
#else
    /* use the LU included with levmar */
    issolved=AX_EQ_B_LU(jacTjac, jacTe, Dp, m); ++nlss; linsolver=AX_EQ_B_LU;
#endif /* HAVE_LAPACK */

    if ( issolved ) {
      /* compute p's new estimate and ||Dp||^2 */
      for ( i = 0, Dp_L2 = 0.0; i < m; ++i ) {
        pDp[i] = p[i] + (tmp = Dp[i]);
        Dp_L2 += tmp * tmp;
      }
      //Dp_L2=sqrt(Dp_L2);

      if ( Dp_L2 <= eps2_sq * p_L2 ) { /* relative change in p is small, stop */
        //if(Dp_L2<=eps2*(p_L2 + eps2)){ /* relative change in p is small, stop */
        stop = 2;
        break;
      }

      if ( Dp_L2 >= (p_L2 + eps2) / (EPSILON * EPSILON) ) { /* almost singular */
        //if(Dp_L2>=(p_L2+eps2)/LM_CNST(EPSILON)){ /* almost singular */
        stop = 4;
        break;
      }

      (*func)(pDp, wrk, m, n, adata);
      ++nfev; /* evaluate function at p + Dp */
      /* compute ||e(pDp)||_2 */
      /* ### wrk2=x-wrk, pDp_eL2=||wrk2|| */
#if 1
      pDp_eL2 = LEVMAR_L2NRMXMY(wrk2, x, wrk, n);
#else
      for(i=0, pDp_eL2=0.0; i<n; ++i) {
        wrk2[i]=tmp=x[i]-wrk[i];
        pDp_eL2+=tmp*tmp;
      }
#endif
      if ( !LM_FINITE(pDp_eL2) ) { /* sum of squares is not finite, most probably due to a user error.
       * This check makes sure that the loop terminates early in the case
       * of invalid input. Thanks to Steve Danauskas for suggesting it
       */

        stop = 7;
        break;
      }

      dF = p_eL2 - pDp_eL2;
      if ( updp || dF > 0 ) { /* update jac */
        for ( i = 0; i < n; ++i ) {
          for ( l = 0, tmp = 0.0; l < m; ++l )
            tmp += jac[i * m + l] * Dp[l]; /* (J * Dp)[i] */
          tmp = (wrk[i] - hx[i] - tmp) / Dp_L2; /* (f(p+dp)[i] - f(p)[i] - (J * Dp)[i])/(dp^T*dp) */
          for ( j = 0; j < m; ++j )
            jac[i * m + j] += tmp * Dp[j];
        }
        ++updjac;
        newjac = 1;
      }

      for ( i = 0, dL = 0.0; i < m; ++i )
        dL += Dp[i] * (mu * Dp[i] + jacTe[i]);

      if ( dL > 0.0 && dF > 0.0 ) { /* reduction in error, increment is accepted */
        tmp = (2.0 * dF / dL - 1.0);
        tmp = 1.0 - tmp * tmp * tmp;
        mu = mu * ((tmp >= ONE_THIRD) ? tmp : ONE_THIRD);
        nu = 2;

        for ( i = 0; i < m; ++i ) /* update p's estimate */
          p[i] = pDp[i];

        for ( i = 0; i < n; ++i ) { /* update e, hx and ||e||_2 */
          e[i] = wrk2[i];  //x[i]-wrk[i];
          hx[i] = wrk[i];
        }
        p_eL2 = pDp_eL2;
        updp = 1;
        continue;
      }
    }

    /* if this point is reached, either the linear system could not be solved or
     * the error did not reduce; in any case, the increment must be rejected
     */

    mu *= nu;
    nu2 = nu << 1;  // 2*nu;
    if ( nu2 <= nu ) { /* nu has wrapped around (overflown). Thanks to Frank Jordan for spotting this case */
      stop = 5;
      break;
    }
    nu = nu2;

    for ( i = 0; i < m; ++i ) /* restore diagonal J^T J entries */
      jacTjac[i * m + i] = diag_jacTjac[i];
  }

  if ( k >= itmax )
    stop = 3;

  for ( i = 0; i < m; ++i ) /* restore diagonal J^T J entries */
    jacTjac[i * m + i] = diag_jacTjac[i];

  if ( true ) {
    info.eps3_initial = init_p_eL2;// info[0] = init_p_eL2;
    info.eps3 = p_eL2; // info[1] = p_eL2;
    info.eps1 = jacTe_inf; // info[2] = jacTe_inf;
    info.eps2 = Dp_L2; // info[3] = Dp_L2;
    for ( i = 0, tmp = LM_REAL_MIN; i < m; ++i ) {
      if ( tmp < jacTjac[i * m + i] ) {
        tmp = jacTjac[i * m + i];
      }
    }
    info.mu = mu / tmp; //  info[4] = mu / tmp;
    info.iters = k;// info[5] = (LM_REAL) k;
    info.reason = (enum levmar_stopping_reason)stop; // info[6] = (LM_REAL) stop;
    info.nfev = nfev;// info[7] = (LM_REAL) nfev;
    info.njev = njap; //info[8] = (LM_REAL) njap;
    info.nlss = nlss;// info[9] = (LM_REAL) nlss;
  }

  /* covariance matrix */
  if ( covar ) {
    LEVMAR_COVAR(jacTjac, covar, p_eL2, m, n);
  }

  if ( freework )
    free(work);

  return (stop != 4 && stop != 7) ? k : LM_ERROR;
}


int levmar_der(
    void (*func)(const double p[], double hx[], int m, int n, void *adata),
    void (*jacf)(const double p[], double j[], int m, int n, void *adata),
    double p[],
    double x[],
    int m,
    int n,
    int itmax,
    const c_levmar_opts & opts,
    c_levmar_status & info,
    double *work,
    double *covar,
    void *adata)
{
  return levmar_der_(
      func,
      jacf,
      p,
      x,
      m,
      n,
      itmax,
      opts,
      info,
      work,
      covar,
      adata);
}

int levmar_der(
    void (*func)(const float p[], float hx[], int m, int n, void *adata),
    void (*jacf)(const float p[], float j[], int m, int n, void *adata),
    float p[],
    float x[],
    int m,
    int n,
    int itmax,
    const c_levmar_opts & opts,
    c_levmar_status & info,
    float *work,
    float *covar,
    void *adata)
{
  return levmar_der_(
      func,
      jacf,
      p,
      x,
      m,
      n,
      itmax,
      opts,
      info,
      work,
      covar,
      adata);
}



int levmar_dif(
  void (*func)(const double p[], double hx[], int m, int n, void *adata),
  double p[],
  double x[],
  int m,
  int n,
  int itmax,
  const c_levmar_opts & opts,
  c_levmar_status & info,
  double work[],
  double covar[],
  void * adata)
{
  return levmar_dif_(
      func,
      p,
      x,
      m,
      n,
      itmax,
      opts,
      info,
      work,
      covar,
      adata);
}


int levmar_dif(
  void (*func)(const float p[], float hx[], int m, int n, void *adata),
  float p[],
  float x[],
  int m,
  int n,
  int itmax,
  const c_levmar_opts & opts,
  c_levmar_status & info,
  float work[],
  float covar[],
  void * adata)
{
  return levmar_dif_(
      func,
      p,
      x,
      m,
      n,
      itmax,
      opts,
      info,
      work,
      covar,
      adata);
}

const char * comment(enum levmar_stopping_reason v)
{
  switch ( v ) {
  case levmar_stopping_reason_unknow :
    return "unknow reason";
  case levmar_stopping_reason_eps1 :
    return "stopped by small gradient J^T e";
  case levmar_stopping_reason_eps2 :
    return "stopped by small Dp";
  case levmar_stopping_reason_itmax :
    return "stopped by itmax";
  case levmar_stopping_reason_singular_matrix :
    return "singular matrix. Restart from current p with increased mu";
  case levmar_stopping_reason_mu :
    return "no further error reduction is possible. Restart with increased mu";
  case levmar_stopping_reason_eps3 :
    return "stopped by small ||e||_2";
  case levmar_stopping_reason_nan :
    return "stopped by invalid (i.e. NaN or Inf) func values. This is a user error";
  }
  return "Invalid value of reason tag";
}
