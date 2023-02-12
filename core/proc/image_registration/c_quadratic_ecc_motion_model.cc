/*
 * c_quadratic_ecc_motion_model.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "c_quadratic_ecc_motion_model.h"
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


int  c_quadratic_ecc_motion_model::num_adustable_parameters() const
{
  return 12;
}

bool c_quadratic_ecc_motion_model::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_quadratic_ecc_motion_model: "
        "pointer to c_quadratic_image_transform is NULL");
    return false;
  }

  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  const int w = gx.cols;
  const int h = gx.rows;

  dst.create(h * 12, w);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < h; ++y ) {
#endif
          for ( int x = 0; x < w; ++x ) {
            dst[y + 0 * h][x] = gx[y][x] * x;
            dst[y + 1 * h][x] = gy[y][x] * x;
            dst[y + 2 * h][x] = gx[y][x] * y;
            dst[y + 3 * h][x] = gy[y][x] * y;
            dst[y + 4 * h][x] = gx[y][x];
            dst[y + 5 * h][x] = gy[y][x];
            dst[y + 6 * h][x] = gx[y][x] * x * y;
            dst[y + 7 * h][x] = gy[y][x] * x * y;
            dst[y + 8 * h][x] = gx[y][x] * x * x;
            dst[y + 9 * h][x] = gy[y][x] * x * x;
            dst[y + 10 * h][x] = gx[y][x] * y * y;
            dst[y + 11 * h][x] = gy[y][x] * y * y;
          }
        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

bool c_quadratic_ecc_motion_model::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_quadratic_ecc_motion_model: "
        "pointer to c_quadratic_image_transform is NULL");
    return false;
  }

  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  cv::Matx26f a =
      transform_->matrix();

  a(0,0) += p(0, 0);
  a(0,1) += p(2, 0);
  a(0,2) += p(4, 0);
  a(0,3) += p(6, 0);
  a(0,4) += p(8, 0);
  a(0,5) += p(10, 0);

  a(1,0) += p(1, 0);
  a(1,1) += p(3, 0);
  a(1,2) += p(5, 0);
  a(1,3) += p(7, 0);
  a(1,4) += p(9, 0);
  a(1,5) += p(11, 0);

  transform_->set_matrix(a);

  if( e ) {
    *e = sqrt(square(p(4, 0)) + square(p(5, 0)) +
        square(size.width * p(0, 0)) + square(size.height * p(2, 0)) +
        square(size.width * p(1, 0)) + square(size.height * p(3, 0)) +
        square(size.width * size.height * p(6, 0)) + square(size.width * size.height * p(7, 0)) +
        square(size.width * size.width * p(8, 0)) + square(size.width * size.width * p(9, 0)) +
        square(size.height * size.height * p(10, 0)) + square(size.height * size.height * p(11, 0)));
  }

  return true;
}

bool c_quadratic_ecc_motion_model::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  return false;
}

