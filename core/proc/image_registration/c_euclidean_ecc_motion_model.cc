/*
 * c_euclidean_ecc_motion_model.cc
 *
 *  Created on: Feb 11, 2023
 *      Author: amyznikov
 */

#include "c_euclidean_ecc_motion_model.h"
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



int c_euclidean_ecc_motion_model::num_adustable_parameters() const
{
  int n = 0;

  if ( !fix_translation_ ) {
    n += 2;
  }

  if ( !fix_rotation_ ) {
    n += 1;
  }

  if ( !fix_scale_ ) {
    n += 1;
  }

  return n;
}

bool c_euclidean_ecc_motion_model::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_euclidean_ecc_motion_model: "
        "pointer to c_euclidean_image_transform is NULL");
    return false;
  }

  // STEEPEST DESCENT IMAGES:
  //  [ gx * dWx / dp0 + gy * dWy / dp0]
  //  [ gx * dWx / dp1 + gy * dWy / dp1]
  //  ...
  //  [ gx * dWx / dpn + gy * dWy / dpn]

  // W([x,y], p):
  //  Wx =  scale * ( ca * (x - tx) - sa * (y - ty))
  //  Wy =  scale * ( sa * (x - tx) + ca * (y - ty))

  // dWx / dtx = scale * ( -ca )
  // dWx / dty = scale * ( +sa )
  // dWx / da = scale * ( -sa * (x - tx) - ca * (y - ty))
  // dWx / ds = (ca * (x - tx) - sa * (y - ty))

  // dWy / dtx = scale * ( -sa )
  // dWy / dty = scale * ( -ca )
  // dWy / da = scale * ( ca * (x - tx) - sa * (y - ty))
  // dWy / ds = (sa * (x - tx) + ca * (y - ty))


  const  int n =
      num_adustable_parameters();

  if ( n < 1 ) {
    CF_ERROR("All parameters are fixed, can not compute steepest_descent_images");
    return false;
  }

  const int w = gx.cols;
  const int h = gx.rows;

  const cv::Vec2f T = transform_->translation();
  const float tx = T[0];
  const float ty = T[1];
  const float angle = transform_->rotation();
  const float scale = transform_->scale();
  const float sa = std::sin(angle);
  const float ca = std::cos(angle);

  dst.create(h * n, w);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [ this, w, h, tx, ty, ca, sa, scale, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < h; ++y ) {
#endif // TBB
          int i = 0;

          const float yy = y - ty;

          if( !fix_translation_ ) {

            for ( int x = 0; x < w; ++x ) {
              const float xx = x - tx;
              dst[y + 0 * h][x] = -scale * ( gx[y][x] * ca - gy[y][x] * sa );
              dst[y + 1 * h][x] = -scale * ( gx[y][x] * sa + gy[y][x] * ca );
            }

            i += 2;
          }

          if( !fix_rotation_ ) {

            for ( int x = 0; x < w; ++x ) {
              const float xx = x - tx;
              dst[y + i * h][x] = scale * ( -gx[y][x] * ( sa * xx + ca * yy) + gy[y][x] * (ca * xx - sa * yy) );
            }

            i += 1;
          }

          if( !fix_scale_ ) {

            for ( int x = 0; x < w; ++x ) {
              const float xx = x - tx;
              dst[y + i * h][x] = gx[y][x] * (ca * xx - sa * yy) + gy[y][x] * (sa * xx + ca * yy);
            }
          }

        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

        bool c_euclidean_ecc_motion_model::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  INSTRUMENT_REGION("");

  if( !transform_ ) {
    CF_ERROR("c_euclidean_ecc_motion_model: "
        "pointer to c_euclidean_image_transform is NULL");
    return false;
  }

  const int n =
      num_adustable_parameters();

  if( p.rows != n || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, n);
    return false;
  }

  float dx = 0;
  float dy = 0;
  float da = 0;
  float ds = 0;
  int i = 0;

  if( !fix_translation_ ) {

    dx = p(i++, 0);
    dy = p(i++, 0);

    transform_->set_translation(transform_->translation() +
        cv::Vec2f(dx, dy));
  }

  if( !fix_rotation_ ) {
    da = p(i++, 0);
    transform_->set_rotation(transform_->rotation() + da);
  }

  if( !fix_scale_ ) {
    ds = p(i++, 0);
    transform_->set_scale(transform_->scale() + ds);
  }

  if( e ) {
    const float sa = sin(da);
    *e = sqrt(square(dx) + square(dy) +
        square(size.width * sa) + square(size.height * sa) +
        square(std::max(size.width, size.height) * ds));
  }

  return true;
}

bool c_euclidean_ecc_motion_model::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  return false;
}

