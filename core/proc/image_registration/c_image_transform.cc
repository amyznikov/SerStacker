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
constexpr int tbb_block_size = 512;
#endif
} // namespace


///////////////////////////////////////////////////////////////////////////////////////////////////

c_translation_image_transform::c_translation_image_transform(float Tx, float Ty)
{
  Tx_ = Tx;
  Ty_ = Ty;
}

c_translation_image_transform::c_translation_image_transform(const cv::Vec2f & T)
{
  Tx_ = T[0];
  Ty_ = T[1];
}

void c_translation_image_transform::reset()
{
  a[0] = 0;
  a[1] = 0;
}

void c_translation_image_transform::set_translation(const cv::Vec2f & T)
{
  Tx_ = T[0];
  Ty_ = T[1];
}

void c_translation_image_transform::set_translation(float x, float y)
{
  Tx_ = x;
  Ty_ = y;
}

cv::Vec2f c_translation_image_transform::translation() const
{
  return cv::Vec2f(Tx_, Ty_);
}

cv::Mat1f c_translation_image_transform::parameters() const
{
  return cv::Mat1f(2, 1, (float*)a).clone();
}

bool c_translation_image_transform::set_parameters(const cv::Mat1f & p)
{
  if ( p.rows == 2 && p.cols == 1 ) {
    Tx_ = p(0, 0);
    Ty_ = p(1, 0);
    return true;
  }

  if ( p.rows == 1 && p.cols == 2 ) {
    Tx_ = p(0, 0);
    Ty_ = p(0, 1);
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
  return false;
}

cv::Mat1f c_translation_image_transform::scale_transfrom(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 2 && p.cols == 1 ) {

    cv::Mat1f ss =
        p.clone();

    ss(0, 0) *= factor;
    ss(1, 0) *= factor;

    return ss;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_translation_image_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  //  Wx =  x + tx
  //  Wy =  y + ty

  INSTRUMENT_REGION("");

  const float tx = Tx_;
  const float ty = Ty_;

  map.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map, tx, ty](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            m[x][0] = x + tx;
            m[x][1] = y + ty;

          }
        }
      });

#else

  for ( int y = 0; y < map.rows; ++y ) {

    cv::Vec2f * m = map[y];

    for ( int x = 0; x < map.cols; ++x ) {

      m[x][0] = x + tx;
      m[x][1] = y + ty;
    }

  }

#endif // TBB

  return true;

}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_euclidean_image_transform::c_euclidean_image_transform(float Tx, float Ty, float angle, float scale)
{
  Tx_ = Tx;
  Ty_ = Ty;
  angle_ = angle;
  scale_ = scale;
}

c_euclidean_image_transform::c_euclidean_image_transform(const cv::Vec2f & T, float angle, float scale)
{
  Tx_ = T[0];
  Ty_ = T[1];
  angle_ = angle;
  scale_ = scale;
}

c_euclidean_image_transform::c_euclidean_image_transform(const cv::Vec2f & C, const cv::Vec2f & T, float angle, float scale)
{
  Tx_ = T[0];
  Ty_ = T[1];
  angle_ = angle;
  scale_ = scale;
  Cx_ = C[0];
  Cy_ = C[1];
}

void c_euclidean_image_transform::reset()
{
  a[0] = 0;
  a[1] = 0;
  a[2] = 0;
  a[3] = 1;
  a[4] = 0;
  a[5] = 0;
}

void c_euclidean_image_transform::set_translation(const cv::Vec2f & v)
{
  Tx_ = v[0];
  Ty_ = v[1];
}

cv::Vec2f c_euclidean_image_transform::translation() const
{
  return cv::Vec2f(Tx_, Ty_);
}

void c_euclidean_image_transform::set_center(const cv::Vec2f & v)
{
  Cx_ = v[0];
  Cy_ = v[1];
}

cv::Vec2f c_euclidean_image_transform::center() const
{
  return cv::Vec2f(Cx_, Cy_);
}

void c_euclidean_image_transform::set_rotation(float v)
{
  angle_ = v;
}

float c_euclidean_image_transform::rotation() const
{
  return angle_;
}

void c_euclidean_image_transform::set_scale(float v)
{
  scale_ = v;
}

float c_euclidean_image_transform::scale() const
{
  return scale_;
}

void c_euclidean_image_transform::set_fix_translation(bool v)
{
  fix_translation_ = v;
}

bool c_euclidean_image_transform::fix_translation() const
{
  return c_euclidean_image_transform::fix_translation_;
}

void c_euclidean_image_transform::set_fix_rotation(bool v)
{
  fix_rotation_ = v;
}

bool c_euclidean_image_transform::fix_rotation() const
{
  return fix_rotation_;
}

void c_euclidean_image_transform::set_fix_scale(bool v)
{
  fix_scale_ = v;
}

bool c_euclidean_image_transform::fix_scale() const
{
  return fix_scale_;
}

cv::Mat1f c_euclidean_image_transform::parameters() const
{
  return cv::Mat1f(6, 1, (float*)a).clone();
}

bool c_euclidean_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 6 && p.cols == 1 ) {
    Tx_ = p(0, 0);
    Ty_ = p(1, 0);
    angle_ = p(2, 0);
    scale_ = p(3, 0);
    Cx_ = p(4, 0);
    Cy_ = p(5, 0);
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 6x1", p.rows, p.cols);
  return false;
}

cv::Mat1f c_euclidean_image_transform::scale_transfrom(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 6 && p.cols == 1 ) {

    cv::Mat1f ss =
        p.clone();

    ss(0, 0) *= factor;
    ss(1, 0) *= factor;
    ss(4, 0) *= factor;
    ss(5, 0) *= factor;

    return ss;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 6x1", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_euclidean_image_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  //  Wx =  s * ( ca * x  - sa * y ) + tx
  //  Wy =  s * ( sa * x  + ca * y ) + ty

  INSTRUMENT_REGION("");

  const float cx = Cx_;
  const float cy = Cy_;
  const float tx = Tx_;
  const float ty = Ty_;
  const float ca = std::cos(angle_);
  const float sa = std::sin(angle_);
  const float s = scale_;

  map.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map, cx, cy, tx, ty, sa, ca, s](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          const float yy = y - cy;

          for ( int x = 0; x < map.cols; ++x ) {

            const float xx = x - cx;

            m[x][0] = s * (ca * xx - sa * yy) + tx;
            m[x][1] = s * (sa * xx + ca * yy) + ty;
          }

        }
      });

#else

  for ( int y = 0; y < map.rows; ++y ) {

    cv::Vec2f * m = map[y];

    const float yy = y - cy;

    for ( int x = 0; x < map.cols; ++x ) {

      const float xx = x - cx;

      m[x][0] = s * (ca * xx - sa * yy) + tx;
      m[x][1] = s * (sa * xx + ca * yy) + ty;
    }

  }

#endif // TBB

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_affine_image_transform::c_affine_image_transform()
{
}

c_affine_image_transform::c_affine_image_transform(const float _a[2][3])
{
  memcpy(this->a.val, _a, sizeof(this->a.val));
}

c_affine_image_transform::c_affine_image_transform(const cv::Matx23f & _a)
{
  this->a = _a;
}

c_affine_image_transform::c_affine_image_transform(float a00, float a01, float a02, float a10, float a11, float a12)
{
  a(0,0) = a00;
  a(0,1) = a01;
  a(0,2) = a02;

  a(1,0) = a10;
  a(1,1) = a11;
  a(1,2) = a12;
}


void c_affine_image_transform::set_translation(const cv::Vec2f & v)
{
  a(0, 2) = v[0];
  a(1, 2) = v[1];
}

cv::Vec2f c_affine_image_transform::translation() const
{
  return cv::Vec2f(a(0, 2), a(1, 2));
}


void c_affine_image_transform::reset()
{
  a = cv::Matx23f::eye();
}


void c_affine_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  this->a = a;
}

const cv::Matx23f& c_affine_image_transform::affine_matrix() const
{
  return a;
}

cv::Mat1f c_affine_image_transform::parameters() const
{
  return cv::Mat1f(a.rows, a.cols, (float*) a.val).clone();
}

bool c_affine_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 2 && p.cols == 3 ) {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        a(i,j) = p[i][j];
      }
    }

    return true;
  }

  if( p.rows == 6 && p.cols == 1 ) {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        a(i,j) = p(i * 3 + j, 0);
      }
    }
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x3", p.rows, p.cols);
  return false;
}

cv::Mat1f c_affine_image_transform::scale_transfrom(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 2 && p.cols == 3 ) {

    cv::Mat1f sp(2, 3);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;
    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x3", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_affine_image_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  // Wx =  a00 * x + a01 * y + a02
  // Wy =  a11 * x + a11 * y + a12

  map.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2);
            m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2);
          }
        }
      });

#else

  for ( int y = 0; y < map.rows; ++y ) {

    cv::Vec2f * m = map[y];

    for ( int x = 0; x < map.cols; ++x ) {

      m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2);
      m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2);
    }
  }

#endif // TBB

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////


c_homography_image_transform::c_homography_image_transform()
{
}

c_homography_image_transform::c_homography_image_transform(const float a[3][3])
{
  memcpy(this->a.val, a, sizeof(this->a.val));
}

c_homography_image_transform::c_homography_image_transform(const cv::Matx33f & a)
{
  this->a = a;
}

c_homography_image_transform::c_homography_image_transform(float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22)
{
  a(0,0) = a00;
  a(0,1) = a01;
  a(0,2) = a02;

  a(1,0) = a10;
  a(1,1) = a11;
  a(1,2) = a12;

  a(2,0) = a20;
  a(2,1) = a21;
  a(2,2) = a22;
}

void c_homography_image_transform::set_translation(const cv::Vec2f & v)
{
  Tx_ = v[0];
  Ty_ = v[1];
}

cv::Vec2f c_homography_image_transform::translation() const
{
  return cv::Vec2f(Tx_, Ty_);
}

void c_homography_image_transform::set_homography_matrix(const cv::Matx33f & a)
{
  this->a = a;
}

const cv::Matx33f& c_homography_image_transform::homography_matrix() const
{
  return this->a;
}

cv::Mat1f c_homography_image_transform::parameters() const
{
  return cv::Mat1f(3, 3, (float*) a.val).clone();
}

bool c_homography_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 3 && p.cols == 3 ) {
    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        a(i, j) = p[i][j];
      }
    }
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 3x3", p.rows, p.cols);
  return false;
}

cv::Mat1f c_homography_image_transform::scale_transfrom(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 3 && p.cols == 3 ) {

    cv::Mat1f sp(3, 3);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;

    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;

    sp(2, 0) = p(2, 0) / factor;
    sp(2, 1) = p(2, 1) / factor;
    sp(2, 2) = p(2, 2);

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 3x3", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_homography_image_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            float w = a(2,0) * x + a(2,1) * y + a(2,2);
            if ( w ) {
              w = 1 / w;
            }
            else {
              w = 1;
            }

            m[x][0] = (a(0,0) * x + a(0,1) * y + a(0,2)) * w;
            m[x][1] = (a(1,0) * x + a(1,1) * y + a(1,2)) * w;
          }
        }
      });

#else

  for( int y = 0; y < map.rows; ++y ) {

    cv::Vec2f *m = map[y];

    for( int x = 0; x < map.cols; ++x ) {

      float w = a(2,0) * x + a(2,1) * y + a(2,2);
      if( w ) {
        w = 1 / w;
      }
      else {
        w = 1;
      }

      m[x][0] = (a(0,0) * x + a(0,1) * y + a(0,2)) * w;
      m[x][1] = (a(1,0) * x + a(1,1) * y + a(1,2)) * w;
    }
  }

#endif

  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform()
{

}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const float a[2][4])
{
  memcpy(this->a.val, a, sizeof(this->a.val));
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const cv::Matx24f & a)
{
  this->a = a;
}


c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(float a00, float a01, float a02, float a03,
    float a10, float a11, float a12, float a13)
{
  a(0,0) = a00;
  a(0,1) = a01;
  a(0,2) = a02;
  a(0,3) = a03;

  a(1,0) = a10;
  a(1,1) = a11;
  a(1,2) = a12;
  a(1,3) = a13;
}


void c_semi_quadratic_image_transform::set_translation(const cv::Vec2f & v)
{
  Tx_ = v[0];
  Ty_ = v[1];
}

cv::Vec2f c_semi_quadratic_image_transform::translation() const
{
  return cv::Vec2f (Tx_, Ty_);
}

void c_semi_quadratic_image_transform::set_matrix(const cv::Matx24f & a)
{
  this->a = a;
}

void c_semi_quadratic_image_transform::set_matrix(const cv::Mat1f & a)
{
  if( a.rows == 4 && a.cols == 2 ) {
    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 4; ++j ) {
        this->a(i, j) = a[j][i];
      }
    }
  }
  else if( a.rows == 2 && a.cols == 4 ) {
    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 4; ++j ) {
        this->a(i, j) = a[i][j];
      }
    }
  }
  else {
    CF_ERROR("Invalid matrix size %dx%d. Must be 2x4", a.rows, a.cols);
  }
}

const cv::Matx24f & c_semi_quadratic_image_transform::matrix() const
{
  return this->a;
}

void c_semi_quadratic_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  for ( int i = 0; i < 2; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      this->a(i, j) = a(i, j);
    }
  }
}

cv::Matx23f c_semi_quadratic_image_transform::affine_matrix() const
{
  return cv::Matx23f(
      a(0, 0), a(0, 1), a(0, 2),
      a(1, 0), a(1, 1), a(1, 2));
}

cv::Mat1f c_semi_quadratic_image_transform::parameters() const
{
  return cv::Mat1f(2, 4, (float*) a.val).clone();
}

bool c_semi_quadratic_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 2 && p.cols == 4 ) {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 4; ++j ) {
        a(i,j) = p[i][j];
      }
    }

    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x4", p.rows, p.cols);
  return false;
}

cv::Mat1f c_semi_quadratic_image_transform::scale_transfrom(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 2 && p.cols == 4 ) {

    cv::Mat1f sp(2, 4);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;
    sp(0, 3) = p(0, 3) / factor;

    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;
    sp(1, 3) = p(1, 3) / factor;

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x4", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_semi_quadratic_image_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y;
            m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y;
          }
        }
      });
#else

  for( int y = 0; y < map.rows; ++y ) {
    cv::Vec2f * m = map[y];

    for ( int x = 0; x < map.cols; ++x ) {
      m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y;
      m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y;
    }
  }

#endif // TBB
  return true;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_quadratic_image_transform::c_quadratic_image_transform()
{

}

c_quadratic_image_transform::c_quadratic_image_transform(const float a[2][6])
{
  memcpy(this->a.val, a, sizeof(this->a.val));
}

c_quadratic_image_transform::c_quadratic_image_transform(const cv::Matx26f & a)
{
  this->a = a;
}


c_quadratic_image_transform::c_quadratic_image_transform(float a00, float a01, float a02, float a03, float a04, float a05,
    float a10, float a11, float a12, float a13, float a14, float a15)
{
  a(0,0) = a00;
  a(0,1) = a01;
  a(0,2) = a02;
  a(0,3) = a03;
  a(0,4) = a04;
  a(0,5) = a05;

  a(1,0) = a10;
  a(1,1) = a11;
  a(1,2) = a12;
  a(1,3) = a13;
  a(1,4) = a14;
  a(1,5) = a15;
}


void c_quadratic_image_transform::set_translation(const cv::Vec2f & v)
{
  Tx_ = v[0];
  Ty_ = v[1];
}

cv::Vec2f c_quadratic_image_transform::translation() const
{
  return cv::Vec2f (Tx_, Ty_);
}

void c_quadratic_image_transform::set_matrix(const cv::Matx26f & a)
{
  this->a = a;
}

void c_quadratic_image_transform::set_matrix(const cv::Mat1f & a)
{
  if( a.rows == 6 && a.cols == 2 ) {
    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 6; ++j ) {
        this->a(i, j) = a[j][i];
      }
    }
  }
  else if( a.rows == 2 && a.cols == 6 ) {
    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 6; ++j ) {
        this->a(i, j) = a[i][j];
      }
    }
  }
  else {
    CF_ERROR("Invalid matrix size %dx%d. Must be 2x6", a.rows, a.cols);
  }
}

const cv::Matx26f & c_quadratic_image_transform::matrix() const
{
  return this->a;
}

void c_quadratic_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  for ( int i = 0; i < 2; ++i ) {
    for ( int j = 0; j < 3; ++j ) {
      this->a(i, j) = a(i, j);
    }
  }
}

cv::Matx23f c_quadratic_image_transform::affine_matrix() const
{
  return cv::Matx23f(
      a(0, 0), a(0, 1), a(0, 2),
      a(1, 0), a(1, 1), a(1, 2));
}

cv::Mat1f c_quadratic_image_transform::parameters() const
{
  return cv::Mat1f(2, 6, (float*) a.val).clone();
}

bool c_quadratic_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 2 && p.cols == 6 ) {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 6; ++j ) {
        a(i,j) = p[i][j];
      }
    }

    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x6", p.rows, p.cols);
  return false;
}

cv::Mat1f c_quadratic_image_transform::scale_transfrom(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 2 && p.cols == 6 ) {

    cv::Mat1f sp(2, 6);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;
    sp(0, 3) = p(0, 3) / factor;
    sp(0, 4) = p(0, 4) / factor;
    sp(0, 5) = p(0, 5) / factor;

    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;
    sp(1, 3) = p(1, 3) / factor;
    sp(1, 4) = p(1, 4) / factor;
    sp(1, 5) = p(1, 5) / factor;

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x6", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_quadratic_image_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

#if HAVE_TBB && !defined(Q_MOC_RUN)
  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y + a(0,4) * x * x + a(0,5) * y * y;
            m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y + a(1,4) * x * x + a(1,5) * y * y;
          }
        }
      });
#else

  for( int y = 0; y < map.rows; ++y ) {
    cv::Vec2f * m = map[y];

    for ( int x = 0; x < map.cols; ++x ) {
      m[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y + a(0,4) * x * x + a(0,5) * y * y;
      m[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y + a(1,4) * x * x + a(1,5) * y * y;
    }
  }

#endif // TBB
  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////
