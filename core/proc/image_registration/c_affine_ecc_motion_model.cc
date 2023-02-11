/*
 * c_affine_ecc_motion_model.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "c_affine_ecc_motion_model.h"
#if HAVE_TBB && !defined(Q_MOC_RUN)
# include <tbb/tbb.h>
#endif
#include <core/debug.h>


namespace {

#if HAVE_TBB && !defined(Q_MOC_RUN)
typedef tbb::blocked_range<int> tbb_range;
constexpr int tbb_block_size = 512;
#endif

template<class T>
static inline T square(T x)
{
  return x * x;
}

} // namespace

int c_affine_ecc_motion_model::num_adustable_parameters() const
{
  return 6;
}

bool c_affine_ecc_motion_model::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  // SDI:
  //  [ gx * dWx/dp0 + gy * dWy/dp0]
  //  [ gx * dWx/dp1 + gy * dWy/dp1]
  //  ...
  //  [ gx * dWx/dpn + gy * dWy/dpn]

  // Map:
  // Wx =  a00 * x + a01 * y + a02
  // Wy =  a10 * x + a11 * y + a12

  // dWx / da00 = x
  // dWx / da01 = y
  // dWx / da02 = 1
  // dWx / da10 = 0
  // dWx / da11 = 0
  // dWx / da12 = 0

  // dWy / da00 = 0
  // dWy / da01 = 0
  // dWy / da02 = 0
  // dWy / da10 = x
  // dWy / da11 = y
  // dWy / da12 = 1

  const int w = gx.cols;
  const int h = gx.rows;

  dst.create(h * 6, w);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < h; ++y ) {
#endif
          for ( int x = 0; x < w; ++x ) {

            dst[y + 0 * h][x] = gx[y][x] * x;   // a00
            dst[y + 1 * h][x] = gx[y][x] * y;   // a01
            dst[y + 2 * h][x] = gx[y][x];       // a02

            dst[y + 3 * h][x] = gy[y][x] * x;   // a10
            dst[y + 4 * h][x] = gy[y][x] * y;   // a11
            dst[y + 5 * h][x] = gy[y][x];       // a12
          }
        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

bool c_affine_ecc_motion_model::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  a[0][0] += p(0, 0);
  a[0][1] += p(1, 0);
  a[0][2] += p(2, 0);

  a[1][0] += p(3, 0);
  a[1][1] += p(4, 0);
  a[1][2] += p(5, 0);

  if( e ) {
    *e = sqrt(square(p(2, 0)) + square(p(5, 0)) +
        square(size.width * p(0, 0)) + square(size.width * p(3, 0)) +
        square(size.height * p(1, 0)) + square(size.height * p(4, 0)));
  }

  return true;
}

bool c_affine_ecc_motion_model::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  const float a00 = p(0, 0);
  const float a01 = p(1, 0);
  const float a02 = p(2, 0);

  const float a10 = p(3, 0);
  const float a11 = p(4, 0);
  const float a12 = p(5, 0);

  cv::Matx33f P(
      a[0][0], a[0][1], a[0][2],
      a[1][0], a[1][1], a[1][2],
        0,     0,      1);

  cv::Matx33f dP(
      1 + a00, a01, a02,
      a10, 1 + a11, a12,
      0, 0, 1);

  bool isok = false;

  dP = dP.inv(cv::DECOMP_LU, &isok);
  if ( !isok ) {
    CF_ERROR("dP.inv() fails");
    return false;
  }

  P = P * dP;

  a[0][0] = P(0, 0);
  a[0][1] = P(0, 1);
  a[0][2] = P(0, 2);

  a[1][0] = P(1, 0);
  a[1][1] = P(1, 1);
  a[1][2] = P(1, 2);


  if( e ) {
    *e = sqrt(square(a02) + square(12) +
        square(size.width * a00) + square(size.width * a10) +
        square(size.height * a01) + square(size.height * a11));
  }

  return true;

}


