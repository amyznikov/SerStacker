/*
 * c_homography_ecc_motion_model.cc
 *
 *  Created on: Feb 12, 2023
 *      Author: amyznikov
 */

#include "c_homography_ecc_motion_model.h"

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


int c_homography_ecc_motion_model::num_adustable_parameters() const
{
  return 8;
}

bool c_homography_ecc_motion_model::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_homography_ecc_motion_model: "
        "pointer to c_homography_image_transform is NULL");
    return false;
  }

  // w  =  (x * a20 + y * a21 + a22)
  // x' =  (x * a00 + y * a01 + a02) / w
  // y' =  (x * a10 + y * a11 + a12) / w

  const int w = gx.cols;
  const int h = gx.rows;

  const cv::Matx33f a =
      transform_->homography_matrix();

  dst.create(h * 8, w);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [a, w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < h; ++y ) {
#endif
          for ( int x = 0; x < w; ++x ) {

            const float den = 1.f / (x * a(2,0) + y * a(2,1) + 1.f );
            const float hatX = -(x * a(0,0) + y * a(0,1) + a(0,2)) * den;
            const float hatY = -(x * a(1,0) + y * a(1,1) + a(1,2)) * den;

            const float ggx = gx[y][x] * den;
            const float ggy = gy[y][x] * den;
            const float gg = hatX * ggx + hatY * ggy;

            dst[y + 0 * h ][x] = ggx * x;
            dst[y + 1 * h ][x] = ggy * x;
            dst[y + 2 * h ][x] = gg * x;
            dst[y + 3 * h ][x] = ggx * y;
            dst[y + 4 * h ][x] = ggy * y;
            dst[y + 5 * h ][x] = gg * y;
            dst[y + 6 * h ][x] = ggx;
            dst[y + 7 * h ][x] = ggy;
          }
        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

bool c_homography_ecc_motion_model::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_homography_ecc_motion_model: "
        "pointer to c_homography_image_transform is NULL");
    return false;
  }

  // w  =  (x * a20 + y * a21 + a22)
  // x' =  (x * a00 + y * a01 + a02) / w
  // y' =  (x * a10 + y * a11 + a12) / w

  cv::Matx33f a =
      transform_->homography_matrix();

  a(0,0) += p(0, 0);
  a(0,1) += p(3, 0);
  a(0,2) += p(6, 0);
  a(1,0) += p(1, 0);
  a(1,1) += p(4, 0);
  a(1,2) += p(7, 0);
  a(2,0) += p(2, 0);
  a(2,1) += p(5, 0);

  transform_->set_homography_matrix(a);

  if( e ) {
    // FIXME?: this estimate does not account for w
    *e = sqrt(square(p(6, 0)) + square(p(7, 0)) +
        square(size.width * p(0, 0)) + square(size.height * p(3, 0)) +
        square(size.width * p(1, 0)) + square(size.height * p(4, 0)));
  }

  return true;
}

bool c_homography_ecc_motion_model::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_homography_ecc_motion_model: "
        "pointer to c_homography_image_transform is NULL");
    return false;
  }


  //  x' =  (x * a00 + y * a01 + a02) / w
  //  y' =  (x * a10 + y * a11 + a12) / w
  //  w  =  (x * a20 + y * a21 + a22)

  //  a(0,0) => p(0, 0);
  //  a(0,1) => p(3, 0);
  //  a(0,2) => p(6, 0);
  //  a(1,0) => p(1, 0);
  //  a(1,1) => p(4, 0);
  //  a(1,2) => p(7, 0);
  //  a(2,0) => p(2, 0);
  //  a(2,1) => p(5, 0);
  //  a(2,2) => 1;

  cv::Matx33f a =
      transform_->homography_matrix();

  cv::Matx33f P(
      a(0,0), a(0,1), a(0,2),
      a(1,0), a(1,1), a(1,2),
      a(2,0), a(2,1), 1.0);

  cv::Matx33f dP(
      1 + p(0, 0), p(3, 0), p(6, 0),
      p(1, 0), 1 + p(4, 0), p(7, 0),
      p(2, 0), p(5, 0), 1.);

  bool isok = false;

  dP = dP.inv(cv::DECOMP_LU, &isok);
  if( !isok ) {
    CF_ERROR("dP.inv() fails");
    return false;
  }

  P = P * dP;

  a(0,0) = P(0, 0);
  a(0,1) = P(0, 1);
  a(0,2) = P(0, 2);

  a(1,0) = P(1, 0);
  a(1,1) = P(1, 1);
  a(1,2) = P(1, 2);

  a(2,0) = P(2, 0);
  a(2,1) = P(2, 1);
  a(2,2) = 1;

  transform_->set_homography_matrix(a);

  if( e ) {
    *e = sqrt(square(p(6, 0)) + square(p(7, 0)) +
        square(size.width * p(0, 0)) + square(size.height * p(3, 0)) +
        square(size.width * p(1, 0)) + square(size.height * p(4, 0)));
  }

  return true;
}
