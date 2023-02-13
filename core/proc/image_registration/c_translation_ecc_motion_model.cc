/*
 * c_translation_ecc_motion_model.cc
 *
 *  Created on: Feb 13, 2023
 *      Author: amyznikov
 */

#include "c_translation_ecc_motion_model.h"
#if HAVE_TBB && !defined(Q_MOC_RUN)
# include <tbb/tbb.h>
#endif
#include <core/debug.h>


namespace {
#if HAVE_TBB && !defined(Q_MOC_RUN)
typedef tbb::blocked_range<int> tbb_range;
constexpr int tbb_block_size = 512;
#endif
} // namespace

template<class T>
static inline T square(T x)
{
  return x * x;
}

int c_translation_ecc_motion_model::num_adustable_parameters() const
{
  return 2;
}

bool c_translation_ecc_motion_model::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_translation_ecc_motion_model: "
        "pointer to c_translation_image_transform is NULL");
    return false;
  }

  // STEEPEST DESCENT IMAGES:
  //  [ gx * dWx / dp0 + gy * dWy / dp0]
  //  [ gx * dWx / dp1 + gy * dWy / dp1]
  //  ...
  //  [ gx * dWx / dpn + gy * dWy / dpn]

  // W([x,y], p):
  //  Wx =  x + tx
  //  Wy =  y + ty

  // dWx / dtx = 1
  // dWx / dty = 0

  // dWy / dtx = 0
  // dWy / dty = 1


  const int w = gx.cols;
  const int h = gx.rows;

  const cv::Vec2f T = transform_->translation();
  const float tx = T[0];
  const float ty = T[1];

  dst.create(h * 2, w);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [ w, h, tx, ty, &gx, &gy, &dst] (const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < h; ++y ) {
#endif // TBB

            for ( int x = 0; x < w; ++x ) {
              dst[y + 0 * h][x] = gx[y][x];
              dst[y + 1 * h][x] = gy[y][x];
            }

        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

bool c_translation_ecc_motion_model::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_translation_ecc_motion_model: "
        "pointer to c_euclidean_image_transform is NULL");
    return false;
  }

  if( p.rows != 2 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
    return false;
  }

  const float dx = p(0, 0);
  const float dy = p(1, 0);

  const cv::Vec2f T = transform_->translation();
  transform_->set_translation(cv::Vec2f(T[0] + dx, T[1] + dy));


  if( e ) {
    *e = sqrt(square(dx) + square(dy));
  }

  return true;
}

bool c_translation_ecc_motion_model::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  return false;
}
