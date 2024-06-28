/*
 * c_image_transform.cc
 *
 *  Created on: Feb 11, 2023
 *      Author: amyznikov
 */

#include "c_image_transform.h"

#if HAVE_TBB && !defined(Q_MOC_RUN)
#include <tbb/tbb.h>
#endif

#include <core/debug.h>

namespace {
#if HAVE_TBB && !defined(Q_MOC_RUN)
typedef tbb::blocked_range<int> tbb_range;
constexpr int tbb_block_size = 256;
#endif
} // namespace


template<class T>
static inline T square(T x)
{
  return x * x;
}

///////////////////////////////////////////////////////////////////////////////////////////////////
bool c_image_transform::remap(cv::InputArray src, cv::InputArray src_mask, const cv::Size & size,
    cv::OutputArray dst, cv::OutputArray dst_mask,
    cv::InterpolationFlags interpolation,
    cv::BorderTypes borderMode,
    const cv::Scalar & borderValue) const
{
  cv::Mat2f rmap;

  if( !create_remap(size, rmap) ) {
    CF_ERROR("create_remap() fails");
    return false;
  }

  if( dst.needed() ) {

    if( src.empty() ) {
      dst.release();
    }
    else {

      cv::remap(src, dst, rmap, cv::noArray(), interpolation,
          borderMode, borderValue);
    }
  }

  if( dst_mask.needed() ) {

    if( src_mask.empty() ) {

      cv::remap(cv::Mat1b(size, (uint8_t) 255), dst_mask,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }
    else {

      cv::remap(src_mask, dst_mask,
          rmap, cv::noArray(),
          cv::INTER_LINEAR,
          cv::BORDER_CONSTANT);
    }

    cv::compare(dst_mask, 255, dst_mask, cv::CMP_GE);
  }

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_translation_image_transform::c_translation_image_transform(float Tx, float Ty)
{
  set_translation(cv::Vec2f(Tx, Ty));
}

c_translation_image_transform::c_translation_image_transform(const cv::Vec2f & T)
{
  set_translation(T);
}

void c_translation_image_transform::reset()
{
  set_translation(cv::Vec2f(0, 0));
}

void c_translation_image_transform::set_translation(const cv::Vec2f & T)
{
  if ( parameters_.rows != 2 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(2, 1);
  }

  parameters_(0, 0) = T(0);
  parameters_(1, 0) = T(1);
}

void c_translation_image_transform::set_translation(float x, float y)
{
  if ( parameters_.rows != 2 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(2, 1);
  }

  parameters_(0, 0) = x;
  parameters_(1, 0) = y;
}

cv::Vec2f c_translation_image_transform::translation() const
{
  return cv::Vec2f((const float*)parameters_.data);
}

bool c_translation_image_transform::set_parameters(const cv::Mat1f & p)
{
  if ( p.rows != 2 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(parameters_);

  return true;
}

void c_translation_image_transform::scale_transfrom(double factor)
{
  parameters_(0, 0) *= factor;
  parameters_(1, 0) *= factor;
}

double c_translation_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  return sqrt(dp(0, 0) * dp(0, 0) + dp(1, 0) * dp(0, 1));
}

bool c_translation_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 2 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
    return false;
  }
  return create_remap(cv::Vec2f((const float*) p.data), size, rmap);
}

bool c_translation_image_transform::create_remap(const cv::Vec2f & T, const cv::Size & size, cv::Mat2f & rmap) const
{
  INSTRUMENT_REGION("");

  //  x' =  x + tx
  //  y' =  y + ty

  // CF_DEBUG("T={%g %g} size=%dx%d", T(0), T(1), size.width, size.height);

  rmap.create(size);

#if HAVE_TBB
  tbb::parallel_for(tbb_range(0, rmap.rows, tbb_block_size),
      [&rmap, T](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for( int y = 0; y < rmap.rows; ++y ) {
#endif
          cv::Vec2f * mp = rmap[y];

          for( int x = 0; x < rmap.cols; ++x ) {
            mp[x][0] = x + T[0];
            mp[x][1] = y + T[1];
          }
        }
#if HAVE_TBB
  });
#endif

  return true;
}

bool c_translation_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/,
    const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
{
  //gx.copyTo(J[0]);
  //gy.copyTo(J[1]);

  J[0] = gx;
  J[1] = gy;

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_euclidean_image_transform::c_euclidean_image_transform(float Tx, float Ty, float angle, float scale)
{
  set_parameters(Tx, Ty, angle, scale, 0, 0);
}

c_euclidean_image_transform::c_euclidean_image_transform(const cv::Vec2f & T, float angle, float scale)
{
  set_parameters(T[0], T[0], angle, scale, 0, 0);
}

c_euclidean_image_transform::c_euclidean_image_transform(const cv::Vec2f & C, const cv::Vec2f & T, float angle, float scale)
{
  set_parameters(T[0], T[0], angle, scale, C[0], C[1]);
}

void c_euclidean_image_transform::reset()
{
  set_parameters(0, 0, 0, 0, 0, 0);
}

int c_euclidean_image_transform::num_adjustable_parameters() const
{
  int np = 0;

  if( !fix_translation_ ) {
    np += 2;
  }

  if( !fix_rotation_ ) {
    np += 1;
  }

  if( !fix_scale_ ) {
    np += 1;
  }

  return np;
}

void c_euclidean_image_transform::update_parameters()
{
  const int np =
      num_adjustable_parameters();

  if ( np < 1 ) {
    CF_ERROR("c_euclidean_image_transform: No adjusteble parameters");
    parameters_.release();
  }
  else {
    if ( parameters_.rows != np || parameters_.cols != 1 ) {
      parameters_.release();
      parameters_.create(np, 1);
    }

    int ip = 0;

    if( !fix_translation_ ) {
      parameters_(ip++, 0) = T_(0);
      parameters_(ip++, 0) = T_(1);
    }

    if( !fix_rotation_ ) {
      parameters_(ip++, 0) = angle_;
    }

    if( !fix_scale_ ) {
      parameters_(ip++, 0) = scale_;
    }
  }
}


void c_euclidean_image_transform::set_parameters(float Tx, float Ty, float angle, float scale, float Cx, float Cy)
{
  T_(0) = Tx;
  T_(1) = Ty;
  C_(0) = Cx;
  C_(1) = Cy;
  angle_ = angle;
  scale_ = scale;

  update_parameters() ;
}

bool c_euclidean_image_transform::set_parameters(const cv::Mat1f & p)
{
  const int np =
      num_adjustable_parameters();

  if( p.rows != np || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, np);
    return false;
  }

  int i = 0;

  if( !fix_translation_ ) {
    T_(0) = p(i++, 0);
    T_(1) = p(i++, 0);
  }

  if( !fix_rotation_ ) {
    angle_ = p(i++, 0);
  }

  if( !fix_scale_ ) {
    scale_ = p(i++, 0);
  }

  update_parameters();

  return true;
}

bool c_euclidean_image_transform::get_parameters(const cv::Mat1f & p, float * Tx, float * Ty, float * angle,
    float * scale, float * Cx, float * Cy) const
{
  const int np =
      num_adjustable_parameters();

  if( p.rows != np || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, np);
    return false;
  }

  *Cx = C_[0];
  *Cy = C_[1];

  int ip = 0;

  if( fix_translation_ ) {
    *Tx = T_(0);
    *Ty = T_(1);
  }
  else {
    *Tx = p(ip++, 0);
    *Ty = p(ip++, 0);
  }

  if( fix_rotation_ ) {
    *angle = angle_;
  }
  else {
    *angle = p(ip++, 0);
  }

  if( fix_scale_ ) {
    *scale = scale_;
  }
  else {
    *scale = p(ip++, 0);
  }

  return true;
}




void c_euclidean_image_transform::set_translation(const cv::Vec2f & v)
{
  T_ = v;
  update_parameters() ;
}

cv::Vec2f c_euclidean_image_transform::translation() const
{
  return T_;
}

void c_euclidean_image_transform::set_center(const cv::Vec2f & v)
{
  C_ = v;
}

cv::Vec2f c_euclidean_image_transform::center() const
{
  return C_;
}

void c_euclidean_image_transform::set_rotation(float angle)
{
  angle_ = angle;
  update_parameters();
}

float c_euclidean_image_transform::rotation() const
{
  return parameters_(2,0);
}

void c_euclidean_image_transform::set_scale(float scale)
{
  scale_ = scale;
  update_parameters();
}

float c_euclidean_image_transform::scale() const
{
  return scale_;
}

void c_euclidean_image_transform::set_fix_translation(bool v)
{
  fix_translation_ = v;
  update_parameters();
}

bool c_euclidean_image_transform::fix_translation() const
{
  return fix_translation_;
}

void c_euclidean_image_transform::set_fix_rotation(bool v)
{
  fix_rotation_ = v;
  update_parameters();
}

bool c_euclidean_image_transform::fix_rotation() const
{
  return fix_rotation_;
}

void c_euclidean_image_transform::set_fix_scale(bool v)
{
  fix_scale_ = v;
  update_parameters();
}

bool c_euclidean_image_transform::fix_scale() const
{
  return fix_scale_;
}


void c_euclidean_image_transform::scale_transfrom(double factor)
{
  T_ *= factor;
  C_ *= factor;
  update_parameters();
}

double c_euclidean_image_transform::eps(const cv::Mat1f & dp,
    const cv::Size & image_size)
{

  float dTx = 0, dTy = 0, da = 0, ds = 0, dCx = 0, dCy = 0;

  get_parameters(dp, &dTx, &dTy, &da, &ds, &dCx, &dCy);

  const float sa =
      std::sin(da);

  const float eps =
      std::sqrt(square(dTx) + square(dTy) + square(image_size.width * sa) + square(image_size.height * sa) +
          square(std::max(image_size.width, image_size.height) * ds));

  return eps;
}

bool c_euclidean_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  //  Wx =  s * ( ca * x  - sa * y ) + tx
  //  Wy =  s * ( sa * x  + ca * y ) + ty

  INSTRUMENT_REGION("");

  float Tx, Ty, angle, scale, Cx, Cy;

  if ( !get_parameters(p, &Tx, &Ty, &angle, &scale, &Cx, &Cy) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }

  const float sa =
      std::sin(angle);

  const float ca =
      std::cos(angle);

  rmap.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, rmap.rows, tbb_block_size),
      [Cx, Cy, Tx, Ty, ca, sa, scale, &rmap] (const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
#else
        for ( int y = 0; y < rmap.rows; ++y ) {
#endif // TBB

          cv::Vec2f * mp = rmap[y];

          const float yy = y - Cy;

          for ( int x = 0; x < rmap.cols; ++x ) {

            const float xx = x - Cx;

            mp[x][0] = scale * (ca * xx - sa * yy) + Tx;
            mp[x][1] = scale * (sa * xx + ca * yy) + Ty;
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
     });
#endif // TBB

  return true;
}


bool c_euclidean_image_transform::create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
{
  INSTRUMENT_REGION("");

  const int np =
      num_adjustable_parameters();

  if( p.rows != np || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, np);
    return false;
  }

  // STEEPEST DESCENT IMAGES:
  //  [ gx * dWx / dp0 + gy * dWy / dp0]
  //  [ gx * dWx / dp1 + gy * dWy / dp1]
  //  ...
  //  [ gx * dWx / dpn + gy * dWy / dpn]

  // W([x,y], p):
  //  Wx =  s * ( ca * (x-cx) - sa * (y-cy) ) + tx
  //  Wy =  s * ( sa * (x-cx) + ca * (y-cy) ) + ty

  // dWx / dtx = 1
  // dWx / dty = 0
  // dWx / da  = - s * (sa * (x-cx) + ca * (y-cy))
  // dWx / ds  = ( ca * (x-cx) - sa * (y-cy) )

  // dWy / dtx = 0
  // dWy / dty = 1
  // dWy / da  = s * (ca * (x-cx) - sa * (y-cy))
  // dWy / ds  = ( sa * (x-cx) + ca * (y-cy) )

  float Tx, Ty, angle, scale, Cx, Cy;

  if ( !get_parameters(p, &Tx, &Ty, &angle, &scale, &Cx, &Cy) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }

  const cv::Size size =
      gx.size();

  const bool fix_translation =
      fix_translation_;

  const bool fix_rotation =
      fix_rotation_;

  const bool fix_scale =
      fix_scale_;

  const float sa =
      std::sin(angle);

  const float ca =
      std::cos(angle);

  for ( int i = 0; i < np; ++i ) {
    J[i].create(size);
  }

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
      [ fix_translation, fix_rotation, fix_scale,
        size, Cx, Cy, Tx, Ty, ca, sa, scale, &gx, &gy, &J]
        (const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < size.height; ++y ) {
#endif // TBB

          int i = 0;

          if( !fix_translation ) {

            for ( int x = 0; x < size.width; ++x ) {
              J[0][y][x] = gx[y][x];
              J[1][y][x] = gy[y][x];
            }

            i += 2;
          }

          if( !fix_rotation ) {

            const float yy = y - Cy;

            for ( int x = 0; x < size.width; ++x ) {

              const float xx = x - Cx;

              J[i][y][x] = scale * ( -gx[y][x] * (sa * xx + ca * yy) + gy[y][x] * (ca * xx - sa * yy));
            }

            i += 1;
          }

          if( !fix_scale ) {

            const float yy = y - Cy;

            for ( int x = 0; x < size.width; ++x ) {

              const float xx = x - Cx;

              J[i][y][x] = gx[y][x] * (ca * xx - sa * yy) + gy[y][x] * (sa * xx + ca * yy);
            }
          }

        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_affine_image_transform::c_affine_image_transform()
{
  set_matrix(cv::Matx23f::eye());
}

c_affine_image_transform::c_affine_image_transform(const float data[2][3])
{
  set_matrix(cv::Matx23f((const float*) data));
}

c_affine_image_transform::c_affine_image_transform(const cv::Matx23f & a)
{
  set_matrix(a);
}

c_affine_image_transform::c_affine_image_transform(float a00, float a01, float a02, float a10, float a11, float a12)
{
  set_matrix(cv::Matx23f(
      a00, a01, a02,
      a10, a11, a12));
}

void c_affine_image_transform::reset()
{
  set_matrix(cv::Matx23f::eye());
}

void c_affine_image_transform::set_translation(const cv::Vec2f & v)
{
  parameters_(2, 0) = v(0);
  parameters_(5, 0) = v(1);
}

cv::Vec2f c_affine_image_transform::translation() const
{
  return cv::Vec2f(parameters_(2, 0), parameters_(5, 0));
}

void c_affine_image_transform::set_matrix(const cv::Matx23f & a)
{
  if ( parameters_.rows != 6 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(6, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = a(1, 0);
  parameters_(4, 0) = a(1, 1);
  parameters_(5, 0) = a(1, 2);
}

cv::Matx23f c_affine_image_transform::matrix(const cv::Mat1f & p) const
{
  return cv::Matx23f((const float*) p.data);
}

cv::Matx23f c_affine_image_transform::matrix() const
{
  return matrix(parameters_);
}

bool c_affine_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows != 6 || p.cols != 1 ) {
    CF_ERROR("c_affine_image_transform: Invalid parameters size: %dx%d. Must be 6x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(parameters_);
  return true;
}

void c_affine_image_transform::scale_transfrom(double scale)
{
  parameters_(2, 0) *= scale;
  parameters_(5, 0) *= scale;
}

double c_affine_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  const float eps =
      std::sqrt(square(image_size.width * dp(0, 0)) + square(image_size.height * dp(1, 0)) + square(dp(2, 0)) +
          square(image_size.width * dp(3, 0)) + square(image_size.height * dp(4, 0)) + square(dp(5, 0)));

  return eps;
}


bool c_affine_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 6 || p.cols != 1 ) {
    CF_ERROR("c_affine_image_transform: Invalid parameters size: %dx%d. Must be 6x1", p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}

bool c_affine_image_transform::create_remap(const cv::Matx23f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  INSTRUMENT_REGION("");

  //  x' =  a00 * x  + a01 * y + a02
  //  y' =  a10 * x  + a11 * y + a12

  rmap.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, rmap.rows, tbb_block_size),
      [a, &rmap](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for( int y = 0; y < rmap.rows; ++y ) {
#endif
          cv::Vec2f * mp = rmap[y];

          for( int x = 0; x < rmap.cols; ++x ) {
            mp[x][0] = a(0, 0) * x + a(0, 1) * y + a(0, 2);
            mp[x][1] = a(1, 0) * x + a(1, 1) * y + a(1, 2);
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  });
#endif

  return true;
}

bool c_affine_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
{
  const cv::Size size =
      gx.size();

#if 0
  if ( xx.size() != size || yy.size() != size ) {

    xx.create(size);
    yy.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
      [&](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for( int y = 0; y < size.height; ++y ) {
#endif
          for( int x = 0; x < size.width; ++x ) {
            xx[y][x] =  x;
            yy[y][x] =  y;
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
   });
#endif
  }

  tbb::parallel_invoke(
    [&]() {
      cv::multiply(gx, xx, J[0]); // a00
    },
    [&]() {
      cv::multiply(gx, yy, J[1]); // a01
    },
    [&]() {
      gx.copyTo(J[2]); // a02
    },

    [&]() {
      cv::multiply(gy, xx, J[3]); // a10
    },
    [&]() {
      cv::multiply(gy, yy, J[4]); // a11
    },
    [&]() {
      gy.copyTo(J[5]); // a12
    });

#else
  for ( int i = 0; i < 6; ++i ) {
    J[i].create(size);
  }

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
      [size, &gx, &gy, &J](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for( int y = 0; y < size.height; ++y ) {
#endif
          for( int x = 0; x < size.width; ++x ) {

            J[0][y][x] = gx[y][x] * x;   // a00
            J[1][y][x] = gx[y][x] * y;// a01
            J[2][y][x] = gx[y][x];     // a02

            J[3][y][x] = gy[y][x] * x;// a10
            J[4][y][x] = gy[y][x] * y;// a11
            J[5][y][x] = gy[y][x];     // a12
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
   });
#endif

#endif

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


c_homography_image_transform::c_homography_image_transform()
{
  set_matrix(cv::Matx33f::eye());
}

c_homography_image_transform::c_homography_image_transform(const float data[3][3])
{
  set_matrix(cv::Matx33f((const float*) data));
}

c_homography_image_transform::c_homography_image_transform(const cv::Matx33f & a)
{
  set_matrix(a);
}

c_homography_image_transform::c_homography_image_transform(float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22)
{
  set_matrix(cv::Matx33f(
      a00, a01, a02,
      a10, a11, a12,
      a20, a21, a22));

}

void c_homography_image_transform::reset()
{
  set_matrix(cv::Matx33f::eye());
}

void c_homography_image_transform::update_parameters()
{
  if ( parameters_.rows != 8 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(8, 1);
  }

  parameters_(0, 0) = matrix_(0, 0);
  parameters_(1, 0) = matrix_(0, 1);
  parameters_(2, 0) = matrix_(0, 2);
  parameters_(3, 0) = matrix_(1, 0);
  parameters_(4, 0) = matrix_(1, 1);
  parameters_(5, 0) = matrix_(1, 2);
  parameters_(6, 0) = matrix_(2, 0);
  parameters_(7, 0) = matrix_(2, 1);
  //parameters_(8, 0) = matrix_(2, 2);
}

void c_homography_image_transform::set_translation(const cv::Vec2f & v)
{
  matrix_(0, 2) = v(0);
  matrix_(1, 2) = v(1);
  update_parameters();
}

cv::Vec2f c_homography_image_transform::translation() const
{
  return cv::Vec2f(matrix_(0, 2), matrix_(1, 2));
}


void c_homography_image_transform::set_matrix(const cv::Matx33f & a)
{
  matrix_ = a;
  update_parameters();
}

const cv::Matx33f & c_homography_image_transform::matrix() const
{
  return matrix_;
}

cv::Matx33f c_homography_image_transform::matrix(const cv::Mat1f & p) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("c_homography_image_transform: Invalid parameters size: %dx%d. Must be 8x1",
        p.rows, p.cols);
  }

  return cv::Matx33f(p(0, 0), p(1, 0), p(2, 0),
      p(3, 0), p(4, 0), p(5, 0),
      p(6, 0), p(7, 0), matrix_(2, 2));
}

bool c_homography_image_transform::set_parameters(const cv::Mat1f & p)
{
  set_matrix(matrix(p));
  update_parameters();
  return true;
}

void c_homography_image_transform::scale_transfrom(double factor)
{
  matrix_(0, 2) *= factor;
  matrix_(1, 2) *= factor;
  matrix_(2, 0) /= factor;
  matrix_(2, 1) /= factor;

  update_parameters();
}

double c_homography_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  // FIXME?: this estimate does not account for w

  const float eps =
      sqrt(square(dp(2, 0)) + square(dp(5, 0)) +
          square(image_size.width * dp(0, 0)) + square(image_size.height * dp(1, 0)) +
          square(image_size.width * dp(3, 0)) + square(image_size.height * dp(4, 0)));

  return eps;
}

bool c_homography_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("c_homography_image_transform: Invalid parameters size: %dx%d. Must be 8x1",
        p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}

bool c_homography_image_transform::create_remap(const cv::Matx33f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  rmap.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, rmap.rows, tbb_block_size),
      [a, &rmap](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
#else
          for( int y = 0; y < map.rows; ++y ) {

#endif
          cv::Vec2f * mp = rmap[y];

          for ( int x = 0; x < rmap.cols; ++x ) {

            const float w = a(2,0) * x + a(2,1) * y + a(2,2);

            mp[x][0] = (a(0,0) * x + a(0,1) * y + a(0,2)) / w;
            mp[x][1] = (a(1,0) * x + a(1,1) * y + a(1,2)) / w;
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
        });
#endif

  return true;
}

bool c_homography_image_transform::create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
{
  INSTRUMENT_REGION("");

  // w  =  (x * a20 + y * a21 + a22)
  // x' =  (x * a00 + y * a01 + a02) / w
  // y' =  (x * a10 + y * a11 + a12) / w

  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 9x1", p.rows, p.cols);
    return false;
  }

  const cv::Size size =
      gx.size();

  const cv::Matx33f a(matrix(p));

  for( int i = 0; i < 8; ++i ) {
    J[i].create(size);
  }

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
      [a, size, &gx, &gy, &J](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < size.height; ++y ) {
#endif

          for ( int x = 0; x < size.width; ++x ) {

            const float den = 1.f / (x * a(2,0) + y * a(2,1) + 1.f );
            const float hatX = -(x * a(0,0) + y * a(0,1) + a(0,2)) * den;
            const float hatY = -(x * a(1,0) + y * a(1,1) + a(1,2)) * den;

            const float ggx = gx[y][x] * den;
            const float ggy = gy[y][x] * den;
            const float gg = hatX * ggx + hatY * ggy;

            J[0][y][x] = ggx * x;
            J[1][y][x] = ggx * y;
            J[2][y][x] = ggx;

            J[3][y][x] = ggy * x;
            J[4][y][x] = ggy * y;
            J[5][y][x] = ggy;

            J[6][y][x] = gg * x;
            J[7][y][x] = gg * y;

          }
        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform()
{
  set_matrix(cv::Matx24f::eye());
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const float data[2][4])
{
  set_matrix(cv::Matx24f((const float *)data));
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const cv::Matx24f & a)
{
  set_matrix(cv::Matx24f(a));
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(float a00, float a01, float a02, float a03,
    float a10, float a11, float a12, float a13)
{
  set_matrix(cv::Matx24f(
      a00, a01, a02, a03,
      a10, a11, a12, a13));
}


void c_semi_quadratic_image_transform::reset()
{
  set_matrix(cv::Matx24f::eye());
}


void c_semi_quadratic_image_transform::set_translation(const cv::Vec2f & v)
{
  parameters_(2, 0) = v(0);
  parameters_(6, 0) = v(1);
}

cv::Vec2f c_semi_quadratic_image_transform::translation() const
{
  return cv::Vec2f ((const float*)parameters_.data);
}

void c_semi_quadratic_image_transform::set_matrix(const cv::Matx24f & a)
{
  if ( parameters_.rows != 8 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(8, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = a(0, 3);
  parameters_(4, 0) = a(1, 0);
  parameters_(5, 0) = a(1, 1);
  parameters_(6, 0) = a(1, 2);
  parameters_(7, 0) = a(1, 3);
}

void c_semi_quadratic_image_transform::set_matrix(const cv::Mat1f & a)
{
  if ( a.rows != 2 || a.cols != 4 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x4", a.rows, a.cols);
    return;
  }

  if ( parameters_.rows != 8 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(8, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = a(0, 3);
  parameters_(4, 0) = a(1, 0);
  parameters_(5, 0) = a(1, 1);
  parameters_(6, 0) = a(1, 2);
  parameters_(7, 0) = a(1, 3);
}

cv::Matx24f c_semi_quadratic_image_transform::matrix() const
{
  return cv::Matx24f((const float*)parameters_.data);
}

cv::Matx24f c_semi_quadratic_image_transform::matrix(const cv::Mat1f & p) const
{
  return cv::Matx24f((const float*)p.data);
}


void c_semi_quadratic_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  if ( parameters_.rows != 8 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(8, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = 0;
  parameters_(4, 0) = a(1, 0);
  parameters_(5, 0) = a(1, 1);
  parameters_(6, 0) = a(1, 2);
  parameters_(7, 0) = 0;
}

cv::Matx23f c_semi_quadratic_image_transform::affine_matrix() const
{
  const cv::Mat1f & a =
      parameters_;

  return cv::Matx23f(a(0, 0), a(1, 0), a(2, 0),
      a(4, 0), a(5, 0), a(6, 0));
}

bool c_semi_quadratic_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 8x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(parameters_);
  return true;
}

void c_semi_quadratic_image_transform::scale_transfrom(double factor)
{
  parameters_(2, 0) *= factor;
  parameters_(3, 0) /= factor;

  parameters_(6, 0) *= factor;
  parameters_(7, 0) /= factor;
}

double c_semi_quadratic_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  // x' =  a00 * x + a01 * y + a02 + a03 * x * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y

//  dp(0, 0) = a(0, 0);
//  dp(1, 0) = a(0, 1);
//  dp(2, 0) = a(0, 2);
//  dp(3, 0) = a(0, 3);
//  dp(4, 0) = a(1, 0);
//  dp(5, 0) = a(1, 1);
//  dp(6, 0) = a(1, 2);
//  dp(7, 0) = a(1, 3);

  const float eps =
      std::sqrt( square(dp(2, 0)) + square(dp(6, 0)) +
            square(image_size.width * dp(0, 0)) + square(image_size.height * dp(1, 0)) +
            square(image_size.width * dp(4, 0)) + square(image_size.height * dp(5, 0)) +
            square(image_size.width * image_size.height * dp(3, 0)) +
            square(image_size.width * image_size.height * dp(7, 0)));

  return eps;
}

bool c_semi_quadratic_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 8x1", p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}

bool c_semi_quadratic_image_transform::create_remap(const cv::Matx24f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  rmap.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, rmap.rows, tbb_block_size),
      [a, &rmap](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
#else
        for( int y = 0; y < map.rows; ++y ) {
#endif // TBB

          cv::Vec2f * mp = rmap[y];

          for ( int x = 0; x < rmap.cols; ++x ) {
            mp[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y;
            mp[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y;
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  });
#endif // TBB
  return true;
}

bool c_semi_quadratic_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/, const cv::Mat1f & gx, const cv::Mat1f & gy,
    cv::Mat1f J[]) const
{
  INSTRUMENT_REGION("");

  // x' =  a00 * x + a01 * y + a02 + a03 * x * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y

  const cv::Size size =
      gx.size();

  for( int i = 0; i < 8; ++i ) {
    J[i].create(size);
  }

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
      [size, &gx, &gy, &J](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < size.height; ++y ) {
#endif
          for ( int x = 0; x < size.width; ++x ) {
            J[0][y][x] = gx[y][x] * x; // a00
            J[1][y][x] = gx[y][x] * y; // a01
            J[2][y][x] = gx[y][x]; // a02
            J[3][y][x] = gx[y][x] * x * y; // a03

            J[4][y][x] = gy[y][x] * x; // a10
            J[5][y][x] = gy[y][x] * y; // a11
            J[6][y][x] = gy[y][x]; // a12
            J[7][y][x] = gy[y][x] * x * y; // a13
          }
        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_quadratic_image_transform::c_quadratic_image_transform()
{
  set_matrix(cv::Matx26f::eye());
}

c_quadratic_image_transform::c_quadratic_image_transform(const float data[2][6])
{
  set_matrix(cv::Matx26f( (const float*)data));
}

c_quadratic_image_transform::c_quadratic_image_transform(const cv::Matx26f & a)
{
  set_matrix(a);
}

c_quadratic_image_transform::c_quadratic_image_transform(float a00, float a01, float a02, float a03, float a04, float a05,
    float a10, float a11, float a12, float a13, float a14, float a15)
{
  set_matrix(cv::Matx26f(
      a00, a01, a02, a03, a04, a05,
      a10, a11, a12, a13, a14, a15));
}

void c_quadratic_image_transform::reset()
{
  set_matrix(cv::Matx26f::eye());
}


void c_quadratic_image_transform::set_translation(const cv::Vec2f & v)
{
  parameters_(2, 0) = v(0);
  parameters_(8, 0) = v(1);
}

cv::Vec2f c_quadratic_image_transform::translation() const
{
  return cv::Vec2f(parameters_(2, 0), parameters_(8, 0));
}

void c_quadratic_image_transform::set_matrix(const cv::Matx26f & a)
{
  if ( parameters_.rows != 12 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(12, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = a(0, 3);
  parameters_(4, 0) = a(0, 4);
  parameters_(5, 0) = a(0, 5);

  parameters_(6, 0) = a(1, 0);
  parameters_(7, 0) = a(1, 1);
  parameters_(8, 0) = a(1, 2);
  parameters_(9, 0) = a(1, 3);
  parameters_(10, 0) = a(1, 4);
  parameters_(11, 0) = a(1, 5);
}

void c_quadratic_image_transform::set_matrix(const cv::Mat1f & a)
{
  if ( a.rows != 2 || a.cols != 6 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x6", a.rows, a.cols);
    return;
  }

  if ( parameters_.rows != 12 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(12, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = a(0, 3);
  parameters_(4, 0) = a(0, 4);
  parameters_(5, 0) = a(0, 5);

  parameters_(6, 0) = a(1, 0);
  parameters_(7, 0) = a(1, 1);
  parameters_(8, 0) = a(1, 2);
  parameters_(9, 0) = a(1, 3);
  parameters_(10, 0) = a(1, 4);
  parameters_(11, 0) = a(1, 5);
}

cv::Matx26f c_quadratic_image_transform::matrix() const
{
  return cv::Matx26f((const float*) parameters_.data);
}

cv::Matx26f c_quadratic_image_transform::matrix(const cv::Mat1f & p) const
{
  return cv::Matx26f((const float*) p.data);
}

void c_quadratic_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  if ( parameters_.rows != 12 || parameters_.cols != 1 ) {
    parameters_.release();
    parameters_.create(12, 1);
  }

  parameters_(0, 0) = a(0, 0);
  parameters_(1, 0) = a(0, 1);
  parameters_(2, 0) = a(0, 2);
  parameters_(3, 0) = 0;
  parameters_(4, 0) = 0;
  parameters_(5, 0) = 0;

  parameters_(6, 0) = a(1, 0);
  parameters_(7, 0) = a(1, 1);
  parameters_(8, 0) = a(1, 2);
  parameters_(9, 0) = 0;
  parameters_(10, 0) = 0;
  parameters_(11, 0) = 0;
}

cv::Matx23f c_quadratic_image_transform::affine_matrix() const
{
  const cv::Mat1f & a =
      parameters_;

  return cv::Matx23f(a(0, 0), a(1, 0), a(2, 0),
      a(6, 0), a(7, 0), a(8, 0));
}

bool c_quadratic_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows != 12 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 12x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(parameters_);
  return true;
}

void c_quadratic_image_transform::scale_transfrom(double factor)
{
  parameters_(2, 0) *= factor;
  parameters_(3, 0) /= factor;
  parameters_(4, 0) /= factor;
  parameters_(5, 0) /= factor;
  parameters_(8, 0) *= factor;
  parameters_(9, 0) /= factor;
  parameters_(10, 0) /= factor;
  parameters_(11, 0) /= factor;
}


double c_quadratic_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  const float eps =
      std::sqrt(square(dp(2, 0)) + square(dp(8, 0)) +
        square(image_size.width * dp(0, 0)) + square(image_size.height * dp(1, 0)) +
        square(image_size.width * dp(6, 0)) + square(image_size.height * dp(7, 0)) +
        square(image_size.width * image_size.height * dp(3, 0)) + square(image_size.width * image_size.height * dp(9, 0)) +
        square(image_size.width * image_size.width * dp(4, 0)) + square(image_size.width * image_size.width * dp(10, 0)) +
        square(image_size.height * image_size.height * dp(5, 0)) + square(image_size.height * image_size.height * dp(11, 0)));

  return eps;
}


bool c_quadratic_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 12 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 12x1", p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}

bool c_quadratic_image_transform::create_remap(const cv::Matx26f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  rmap.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, rmap.rows, tbb_block_size),
      [a, &rmap](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
#else
        for( int y = 0; y < map.rows; ++y ) {
#endif // TBB

          cv::Vec2f * mp = rmap[y];

          for ( int x = 0; x < rmap.cols; ++x ) {
            mp[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y + a(0,4) * x * x + a(0,5) * y * y;
            mp[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y + a(1,4) * x * x + a(1,5) * y * y;
          }
        }
#if HAVE_TBB && !defined(Q_MOC_RUN)
      });
#endif // TBB
  return true;
}

bool c_quadratic_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/, const cv::Mat1f & gx, const cv::Mat1f & gy,
    cv::Mat1f J[]) const
{
  INSTRUMENT_REGION("");

  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  const cv::Size size =
      gx.size();

  for( int i = 0; i < 12; ++i ) {
    J[i].create(size);
  }

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, size.height, tbb_block_size),
      [size, &gx, &gy, &J](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
#else
        for ( int y = 0; y < size.height; ++y ) {
#endif
          for ( int x = 0; x < size.width; ++x ) {

            J[0][y][x] = gx[y][x] * x;
            J[1][y][x] = gx[y][x] * y;
            J[2][y][x] = gx[y][x];
            J[3][y][x] = gx[y][x] * x * y;
            J[4][y][x] = gx[y][x] * x * x;
            J[5][y][x] = gx[y][x] * y * y;

            J[6][y][x] = gy[y][x] * x;
            J[7][y][x] = gy[y][x] * y;
            J[8][y][x] = gy[y][x];
            J[9][y][x] = gy[y][x] * x * y;
            J[10][y][x] = gy[y][x] * x * x;
            J[11][y][x] = gy[y][x] * y * y;
          }
        }
      }
#if HAVE_TBB && !defined(Q_MOC_RUN)
  );
#endif

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
