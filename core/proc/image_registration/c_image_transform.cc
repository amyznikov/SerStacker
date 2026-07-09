/*
 * c_image_transform.cc
 *
 *  Created on: Feb 11, 2023
 *      Author: amyznikov
 */

#include "c_image_transform.h"
#include <core/proc/run-loop.h>
#include <core/debug.h>

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
  if ( _parameters.rows != 2 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(2, 1);
  }

  _parameters(0, 0) = T(0);
  _parameters(1, 0) = T(1);
}

void c_translation_image_transform::set_translation(float x, float y)
{
  if ( _parameters.rows != 2 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(2, 1);
  }

  _parameters(0, 0) = x;
  _parameters(1, 0) = y;
}

cv::Vec2f c_translation_image_transform::translation() const
{
  return cv::Vec2f((const float*)_parameters.data);
}

bool c_translation_image_transform::set_parameters(const cv::Mat1f & p)
{
  if ( p.rows != 2 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(_parameters);

  return true;
}

void c_translation_image_transform::scale_transfrom(double factor)
{
  _parameters(0, 0) *= factor;
  _parameters(1, 0) *= factor;
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
  //  x' =  x + tx
  //  y' =  y + ty

  rmap.create(size);

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    const float t0 = T[0], t1 = T[1];
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float *)(rmap[y]);
      for( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        mp[0] = x + t0;
        mp[1] = y + t1;
      }
    }
  });

  return true;
}

bool c_translation_image_transform::remap(const cv::Vec2f & T, const std::vector<cv::Point2f> & rpts,
    std::vector<cv::Point2f> & cpts) const
{
  //  x' =  x + tx
  //  y' =  y + ty

  cpts.resize(rpts.size());

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];
    cp.x = rp.x + T[0];
    cp.y = rp.y + T[1];
  }

  return true;
}

bool c_translation_image_transform::remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts,
    std::vector<cv::Point2f> & cpts) const
{
  if( p.rows != 2 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1", p.rows, p.cols);
    return false;
  }
  return remap(cv::Vec2f((const float*) p.data), rpts, cpts);
}


bool c_translation_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/,
    const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
{
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
  set_parameters(0, 0, 0, 1, 0, 0);
}

int c_euclidean_image_transform::num_adjustable_parameters() const
{
  int np = 0;

  if( !_fix_translation ) {
    np += 2;
  }

  if( !_fix_rotation ) {
    np += 1;
  }

  if( !_fix_scale ) {
    np += 1;
  }

  return np;
}

void c_euclidean_image_transform::update_parameters()
{
  const int np = num_adjustable_parameters();

  if ( np < 1 ) {
    CF_ERROR("c_euclidean_image_transform: No adjusteble parameters");
    _parameters.release();
  }
  else {
    if ( _parameters.rows != np || _parameters.cols != 1 ) {
      _parameters.release();
      _parameters.create(np, 1);
    }

    int ip = 0;

    if( !_fix_translation ) {
      _parameters(ip++, 0) = _T(0);
      _parameters(ip++, 0) = _T(1);
    }

    if( !_fix_rotation ) {
      _parameters(ip++, 0) = _angle;
    }

    if( !_fix_scale ) {
      _parameters(ip++, 0) = _scale;
    }
  }
}


void c_euclidean_image_transform::set_parameters(float Tx, float Ty, float angle, float scale, float Cx, float Cy)
{
  _T(0) = Tx;
  _T(1) = Ty;
  _C(0) = Cx;
  _C(1) = Cy;
  _angle = angle;
  _scale = scale;

  update_parameters() ;
}

bool c_euclidean_image_transform::set_parameters(const cv::Mat1f & p)
{
  const int np = num_adjustable_parameters();
  if( p.rows != np || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, np);
    return false;
  }

  int i = 0;

  if( !_fix_translation ) {
    _T(0) = p(i++, 0);
    _T(1) = p(i++, 0);
  }

  if( !_fix_rotation ) {
    _angle = p(i++, 0);
  }

  if( !_fix_scale ) {
    _scale = p(i++, 0);
  }

  update_parameters();

  return true;
}

bool c_euclidean_image_transform::get_parameters(const cv::Mat1f & p, float * Tx, float * Ty, float * angle,
    float * scale, float * Cx, float * Cy) const
{
  const int np = num_adjustable_parameters();

  if( p.rows != np || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, np);
    return false;
  }

  *Cx = _C[0];
  *Cy = _C[1];

  int ip = 0;

  if( _fix_translation ) {
    *Tx = _T(0);
    *Ty = _T(1);
  }
  else {
    *Tx = p(ip++, 0);
    *Ty = p(ip++, 0);
  }

  if( _fix_rotation ) {
    *angle = _angle;
  }
  else {
    *angle = p(ip++, 0);
  }

  if( _fix_scale ) {
    *scale = _scale;
  }
  else {
    *scale = p(ip++, 0);
  }

  return true;
}

void c_euclidean_image_transform::set_translation(const cv::Vec2f & v)
{
  _T = v;
  update_parameters() ;
}

cv::Vec2f c_euclidean_image_transform::translation() const
{
  return _T;
}

void c_euclidean_image_transform::set_center(const cv::Vec2f & v)
{
  _C = v;
}

cv::Vec2f c_euclidean_image_transform::center() const
{
  return _C;
}

void c_euclidean_image_transform::set_rotation(float angle)
{
  _angle = angle;
  update_parameters();
}

float c_euclidean_image_transform::rotation() const
{
  return _parameters(2, 0);
}

void c_euclidean_image_transform::set_scale(float scale)
{
  _scale = scale;
  update_parameters();
}

float c_euclidean_image_transform::scale() const
{
  return _scale;
}

void c_euclidean_image_transform::set_fix_translation(bool v)
{
  _fix_translation = v;
  update_parameters();
}

bool c_euclidean_image_transform::fix_translation() const
{
  return _fix_translation;
}

void c_euclidean_image_transform::set_fix_rotation(bool v)
{
  _fix_rotation = v;
  update_parameters();
}

bool c_euclidean_image_transform::fix_rotation() const
{
  return _fix_rotation;
}

void c_euclidean_image_transform::set_fix_scale(bool v)
{
  _fix_scale = v;
  update_parameters();
}

bool c_euclidean_image_transform::fix_scale() const
{
  return _fix_scale;
}


void c_euclidean_image_transform::scale_transfrom(double factor)
{
  _T *= factor;
  _C *= factor;
  update_parameters();
}

double c_euclidean_image_transform::eps(const cv::Mat1f & dp,
    const cv::Size & image_size)
{

  float dTx = 0, dTy = 0, da = 0, ds = 0, dCx = 0, dCy = 0;

  get_parameters(dp, &dTx, &dTy, &da, &ds, &dCx, &dCy);

  const float sa = std::sin(da);
  const float eps =
      std::sqrt(square(dTx) + square(dTy) + square(image_size.width * sa) + square(image_size.height * sa) +
          square(std::max(image_size.width, image_size.height) * ds));

  return eps;
}

bool c_euclidean_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  //  Wx =  s * ( ca * x  - sa * y ) + tx
  //  Wy =  s * ( sa * x  + ca * y ) + ty

  float Tx, Ty, angle, scale, Cx, Cy;

  if( !get_parameters(p, &Tx, &Ty, &angle, &scale, &Cx, &Cy) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }

  const float sa = std::sin(angle);
  const float ca = std::cos(angle);

  rmap.create(size);

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float*)rmap[y];
      const float yy = y - Cy;
      for ( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        const float xx = x - Cx;
        mp[0] = scale * (ca * xx - sa * yy) + Tx;
        mp[1] = scale * (sa * xx + ca * yy) + Ty;
      }
    }
  });

  return true;
}

bool c_euclidean_image_transform::remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts,
    std::vector<cv::Point2f> & cpts) const
{
  //  Wx =  s * ( ca * x  - sa * y ) + tx
  //  Wy =  s * ( sa * x  + ca * y ) + ty

  float Tx, Ty, angle, scale, Cx, Cy;

  if ( !get_parameters(p, &Tx, &Ty, &angle, &scale, &Cx, &Cy) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }

  const float sa = std::sin(angle);
  const float ca = std::cos(angle);

  cpts.resize(rpts.size());

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];
    const float xx = rp.x - Cx;
    const float yy = rp.y - Cy;
    cp.x = scale * (ca * xx - sa * yy) + Tx;
    cp.y = scale * (sa * xx + ca * yy) + Ty;
  }

  return true;
}

bool c_euclidean_image_transform::create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
{
  INSTRUMENT_REGION("");

  const int np = num_adjustable_parameters();
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


  const cv::Size size = gx.size();
  const bool fix_translation = _fix_translation;
  const bool fix_rotation = _fix_rotation;
  const bool fix_scale = _fix_scale;
  const float sa = std::sin(angle);
  const float ca = std::cos(angle);

  for ( int i = 0; i < np; ++i ) {
    J[i].create(size);
  }

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
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
  });

  return true;
}

cv::Mat1f c_euclidean_image_transform::invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
{
  // current parameters p
  float Tx, Ty, angle, scale, Cx, Cy;
  if( !get_parameters(p, &Tx, &Ty, &angle, &scale, &Cx, &Cy) ) {
    CF_ERROR("invert_and_compose: get_parameters(p) failed");
    return cv::Mat1f();
  }

  // dp increments
  float dTx, dTy, dAngle, dScale, dCx, dCy;
  if( !get_parameters(dp, &dTx, &dTy, &dAngle, &dScale, &dCx, &dCy) ) {
    CF_ERROR("invert_and_compose: get_parameters(dp) failed");
    return cv::Mat1f();
  }

  // If a parameter is FIXED, its increment in the dp increment must be exactly zero
  // (for translation/angle) or one (for scale),
  // even if get_parameters returned the absolute value of the field.
  if (_fix_translation) {
    dTx = 0.0f;
    dTy = 0.0f;
  }
  if (_fix_rotation)    {
    dAngle = 0.0f;
  }

  // CRITICAL MAGIC: The base increment scale is always 1.0.
  // If the scale is not fixed, add the dScale increment to it.
  const float scale_dp = _fix_scale ? 1.0f : (1.0f + dScale);

  // Lambda for matrix assembly
  static const auto get_matrix3x3 =
      [](float tX, float tY, float ang, float scl, float cX, float cY) {
        const float sa = std::sin(ang);
        const float ca = std::cos(ang);

        cv::Matx33f M = cv::Matx33f::eye();
        M(0, 0) = scl * ca;
        M(0, 1) = -scl * sa;
        M(0, 2) = tX - scl * ca * cX + scl * sa * cY;

        M(1, 0) = scl * sa;
        M(1, 1) = scl * ca;
        M(1, 2) = tY - scl * sa * cX - scl * ca * cY;
        return M;
      };

  // Mp uses absolute scale, Mdp uses incremental scale (around 1.0)
  const cv::Matx33f Mp  = get_matrix3x3(Tx, Ty, angle, scale, Cx, Cy);
  const cv::Matx33f Mdp = get_matrix3x3(dTx, dTy, dAngle, scale_dp, Cx, Cy);

  // Invert the incremental matrix Mdp (it is now guaranteed NOT to be singular)
  const cv::Matx23f Mdp_2x3(Mdp(0, 0), Mdp(0, 1), Mdp(0, 2), Mdp(1, 0), Mdp(1, 1), Mdp(1, 2));
  cv::Matx23f Mdp_inv_2x3;
  cv::invertAffineTransform(Mdp_2x3, Mdp_inv_2x3);

  cv::Matx33f Mdp_inv = cv::Matx33f::eye();
  for( int r = 0; r < 2; ++r ) {
    for( int c = 0; c < 3; ++c ) {
      Mdp_inv(r, c) = Mdp_inv_2x3(r, c);
    }
  }

  // Composition: Mp_new = Mp * (Mdp^-1)
  const cv::Matx33f M_res = Mp * Mdp_inv;

  // Extraction of parameters
  const float m00 = M_res(0, 0);
  const float m10 = M_res(1, 0);

  const float res_scale = _fix_scale ? scale : std::sqrt(m00 * m00 + m10 * m10);
  const float res_angle = _fix_rotation ? angle : std::atan2(m10, m00);

  const float res_ca = std::cos(res_angle);
  const float res_sa = std::sin(res_angle);

  const float res_Tx = _fix_translation ? Tx : M_res(0, 2) + res_scale * res_ca * Cx - res_scale * res_sa * Cy;
  const float res_Ty = _fix_translation ? Ty : M_res(1, 2) + res_scale * res_sa * Cx + res_scale * res_ca * Cy;

  // Pack back
  const int np = num_adjustable_parameters();
  cv::Mat1f res_p(np, 1);
  int ip = 0;

  if( !_fix_translation ) {
    res_p(ip++, 0) = res_Tx;
    res_p(ip++, 0) = res_Ty;
  }
  if( !_fix_rotation ) {
    res_p(ip++, 0) = res_angle;
  }
  if( !_fix_scale ) {
    res_p(ip++, 0) = res_scale;
  }

  return res_p;
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
  _parameters(2, 0) = v(0);
  _parameters(5, 0) = v(1);
}

cv::Vec2f c_affine_image_transform::translation() const
{
  return cv::Vec2f(_parameters(2, 0), _parameters(5, 0));
}

void c_affine_image_transform::set_matrix(const cv::Matx23f & a)
{
  if ( _parameters.rows != 6 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(6, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = a(1, 0);
  _parameters(4, 0) = a(1, 1);
  _parameters(5, 0) = a(1, 2);
}

cv::Matx23f c_affine_image_transform::matrix(const cv::Mat1f & p) const
{
  return cv::Matx23f((const float*) p.data);
}

cv::Matx23f c_affine_image_transform::matrix() const
{
  return matrix(_parameters);
}

bool c_affine_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows != 6 || p.cols != 1 ) {
    CF_ERROR("c_affine_image_transform: Invalid parameters size: %dx%d. Must be 6x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(_parameters);
  return true;
}

void c_affine_image_transform::scale_transfrom(double scale)
{
  _parameters(2, 0) *= scale;
  _parameters(5, 0) *= scale;
}

double c_affine_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  const float eps =
      std::sqrt(square(image_size.width * dp(0, 0)) + square(image_size.height * dp(1, 0)) + square(dp(2, 0)) +
          square(image_size.width * dp(3, 0)) + square(image_size.height * dp(4, 0)) + square(dp(5, 0)));

  return eps;
}

cv::Mat1f c_affine_image_transform::invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
{
  cv::Matx23f a;

  cv::invertAffineTransform(matrix(p), a);
  cv::invertAffineTransform(a + matrix(dp), a);

  return cv::Mat1f(6, 1, (float*) a.val).clone();
}


bool c_affine_image_transform::create_remap(const cv::Matx23f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  INSTRUMENT_REGION("");

  //  x' =  a00 * x  + a01 * y + a02
  //  y' =  a10 * x  + a11 * y + a12

  rmap.create(size);

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float * )rmap[y];
      for( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        mp[0] = a(0, 0) * x + a(0, 1) * y + a(0, 2);
        mp[1] = a(1, 0) * x + a(1, 1) * y + a(1, 2);
      }
    }
  });

  return true;
}

bool c_affine_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 6 || p.cols != 1 ) {
    CF_ERROR("c_affine_image_transform: Invalid parameters size: %dx%d. Must be 6x1", p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}


bool c_affine_image_transform::remap(const cv::Matx23f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  //  x' =  a00 * x  + a01 * y + a02
  //  y' =  a10 * x  + a11 * y + a12
  cpts.resize(rpts.size());

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];
    cp.x = a(0, 0) * rp.x + a(0, 1) * rp.y + a(0, 2);
    cp.y = a(1, 0) * rp.x + a(1, 1) * rp.y + a(1, 2);
  }

  return true;
}

bool c_affine_image_transform::remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  if( p.rows != 6 || p.cols != 1 ) {
    CF_ERROR("c_affine_image_transform: Invalid parameters size: %dx%d. Must be 6x1", p.rows, p.cols);
    return false;
  }

  return remap(matrix(p), rpts, cpts);
}

bool c_affine_image_transform::create_steepest_descent_images(const cv::Mat1f& /*p*/, const cv::Mat1f & gx,
    const cv::Mat1f & gy, cv::Mat1f J[]) const
{
  const cv::Size size = gx.size();

  for( int i = 0; i < 6; ++i ) {
    J[i].create(size);
  }

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      for( int x = 0; x < size.width; ++x ) {

        J[0][y][x] = gx[y][x] * x;   // a00
        J[1][y][x] = gx[y][x] * y;// a01
        J[2][y][x] = gx[y][x];// a02

        J[3][y][x] = gy[y][x] * x;// a10
        J[4][y][x] = gy[y][x] * y;// a11
        J[5][y][x] = gy[y][x];// a12
      }
    }
  });

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
  if ( _parameters.rows != 8 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(8, 1);
  }

  _parameters(0, 0) = _matrix(0, 0);
  _parameters(1, 0) = _matrix(0, 1);
  _parameters(2, 0) = _matrix(0, 2);
  _parameters(3, 0) = _matrix(1, 0);
  _parameters(4, 0) = _matrix(1, 1);
  _parameters(5, 0) = _matrix(1, 2);
  _parameters(6, 0) = _matrix(2, 0);
  _parameters(7, 0) = _matrix(2, 1);
  //parameters_(8, 0) = matrix_(2, 2);
}

void c_homography_image_transform::set_translation(const cv::Vec2f & v)
{
  _matrix(0, 2) = v(0);
  _matrix(1, 2) = v(1);
  update_parameters();
}

cv::Vec2f c_homography_image_transform::translation() const
{
  return cv::Vec2f(_matrix(0, 2), _matrix(1, 2));
}


void c_homography_image_transform::set_matrix(const cv::Matx33f & a)
{
  _matrix = a;
  update_parameters();
}

const cv::Matx33f & c_homography_image_transform::matrix() const
{
  return _matrix;
}

cv::Matx33f c_homography_image_transform::matrix(const cv::Mat1f & p) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("c_homography_image_transform: Invalid parameters size: %dx%d. Must be 8x1",
        p.rows, p.cols);
  }

  return cv::Matx33f(p(0, 0), p(1, 0), p(2, 0),
      p(3, 0), p(4, 0), p(5, 0),
      p(6, 0), p(7, 0), _matrix(2, 2));
}

cv::Matx33f c_homography_image_transform::dmatrix(const cv::Mat1f & dp) const
{
  if( dp.rows != 8 || dp.cols != 1 ) {
    CF_ERROR("c_homography_image_transform: Invalid parameters size: %dx%d. Must be 8x1",
        dp.rows, dp.cols);
  }

  return cv::Matx33f(dp(0, 0), dp(1, 0), dp(2, 0),
      dp(3, 0), dp(4, 0), dp(5, 0),
      dp(6, 0), dp(7, 0), 0);
}


bool c_homography_image_transform::set_parameters(const cv::Mat1f & p)
{
  set_matrix(matrix(p));
  update_parameters();
  return true;
}

void c_homography_image_transform::scale_transfrom(double factor)
{
  _matrix(0, 2) *= factor;
  _matrix(1, 2) *= factor;
  _matrix(2, 0) /= factor;
  _matrix(2, 1) /= factor;

  update_parameters();
}

cv::Mat1f c_homography_image_transform::invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
{
  cv::Matx33f aii = (matrix(p).inv() + dmatrix(dp)).inv();
  return cv::Mat1f(8, 1, (float*)aii.val).clone();
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

bool c_homography_image_transform::create_remap(const cv::Matx33f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  rmap.create(size);

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      cv::Vec2f * mp = rmap[y];
      for ( int x = 0; x < rmap.cols; ++x ) {
        const float w = a(2,0) * x + a(2,1) * y + a(2,2);
        mp[x][0] = (a(0,0) * x + a(0,1) * y + a(0,2)) / w;
        mp[x][1] = (a(1,0) * x + a(1,1) * y + a(1,2)) / w;
      }
    }
  });

  return true;
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


bool c_homography_image_transform::remap(const cv::Matx33f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  cpts.resize(rpts.size());

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];

    const float w = 1.f / (a(2,0) * rp.x + a(2,1) * rp.y + a(2,2));
    cp.x = (a(0,0) * rp.x + a(0,1) * rp.y + a(0,2)) * w;
    cp.x = (a(1,0) * rp.x + a(1,1) * rp.y + a(1,2)) * w;
  }

  return true;
}

bool c_homography_image_transform::remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("c_homography_image_transform: Invalid parameters size: %dx%d. Must be 8x1",
        p.rows, p.cols);
    return false;
  }

  return remap(matrix(p), rpts, cpts);
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

  const cv::Size size = gx.size();
  const cv::Matx33f a(matrix(p));

  for( int i = 0; i < 8; ++i ) {
    J[i].create(size);
  }

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
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
  });

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
  _parameters(2, 0) = v(0);
  _parameters(6, 0) = v(1);
}

cv::Vec2f c_semi_quadratic_image_transform::translation() const
{
  return cv::Vec2f ((const float*)_parameters.data);
}

void c_semi_quadratic_image_transform::set_matrix(const cv::Matx24f & a)
{
  if ( _parameters.rows != 8 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(8, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = a(0, 3);
  _parameters(4, 0) = a(1, 0);
  _parameters(5, 0) = a(1, 1);
  _parameters(6, 0) = a(1, 2);
  _parameters(7, 0) = a(1, 3);
}

void c_semi_quadratic_image_transform::set_matrix(const cv::Mat1f & a)
{
  if ( a.rows != 2 || a.cols != 4 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x4", a.rows, a.cols);
    return;
  }

  if ( _parameters.rows != 8 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(8, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = a(0, 3);
  _parameters(4, 0) = a(1, 0);
  _parameters(5, 0) = a(1, 1);
  _parameters(6, 0) = a(1, 2);
  _parameters(7, 0) = a(1, 3);
}

cv::Matx24f c_semi_quadratic_image_transform::matrix() const
{
  return cv::Matx24f((const float*)_parameters.data);
}

cv::Matx24f c_semi_quadratic_image_transform::matrix(const cv::Mat1f & p) const
{
  return cv::Matx24f((const float*)p.data);
}


void c_semi_quadratic_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  if ( _parameters.rows != 8 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(8, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = 0;
  _parameters(4, 0) = a(1, 0);
  _parameters(5, 0) = a(1, 1);
  _parameters(6, 0) = a(1, 2);
  _parameters(7, 0) = 0;
}

cv::Matx23f c_semi_quadratic_image_transform::affine_matrix() const
{
  const cv::Mat1f & a =
      _parameters;

  return cv::Matx23f(a(0, 0), a(1, 0), a(2, 0),
      a(4, 0), a(5, 0), a(6, 0));
}

bool c_semi_quadratic_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 8x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(_parameters);
  return true;
}

void c_semi_quadratic_image_transform::scale_transfrom(double factor)
{
  _parameters(2, 0) *= factor;
  _parameters(3, 0) /= factor;

  _parameters(6, 0) *= factor;
  _parameters(7, 0) /= factor;
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

bool c_semi_quadratic_image_transform::create_remap(const cv::Matx24f & a, const cv::Size & size,
    cv::Mat2f & rmap) const
{
  rmap.create(size);

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float * )rmap[y];
      for ( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        mp[0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y;
        mp[1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y;
      }
    }
  });

  return true;
}

bool c_semi_quadratic_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 8x1", p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}

bool c_semi_quadratic_image_transform::remap(const cv::Matx24f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  cpts.resize(rpts.size());

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];
    cp.x = a(0,0) * rp.x + a(0,1) * rp.y + a(0,2) + a(0,3) * rp.x * rp.y;
    cp.y = a(1,0) * rp.x + a(1,1) * rp.y + a(1,2) + a(1,3) * rp.x * rp.y;
  }

  return true;
}

bool c_semi_quadratic_image_transform::remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  if( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 8x1", p.rows, p.cols);
    return false;
  }

  return remap(matrix(p), rpts, cpts);
}

bool c_semi_quadratic_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/, const cv::Mat1f & gx, const cv::Mat1f & gy,
    cv::Mat1f J[]) const
{
  // x' =  a00 * x + a01 * y + a02 + a03 * x * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y

  const cv::Size size = gx.size();

  for( int i = 0; i < 8; ++i ) {
    J[i].create(size);
  }

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      for ( int x = 0; x < size.width; ++x ) {
        J[0][y][x] = gx[y][x] * x; // a00
        J[1][y][x] = gx[y][x] * y;// a01
        J[2][y][x] = gx[y][x];// a02
        J[3][y][x] = gx[y][x] * x * y;// a03

        J[4][y][x] = gy[y][x] * x;// a10
        J[5][y][x] = gy[y][x] * y;// a11
        J[6][y][x] = gy[y][x];// a12
        J[7][y][x] = gy[y][x] * x * y;// a13
      }
    }
  });

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
  _parameters(2, 0) = v(0);
  _parameters(8, 0) = v(1);
}

cv::Vec2f c_quadratic_image_transform::translation() const
{
  return cv::Vec2f(_parameters(2, 0), _parameters(8, 0));
}

void c_quadratic_image_transform::set_matrix(const cv::Matx26f & a)
{
  if ( _parameters.rows != 12 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(12, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = a(0, 3);
  _parameters(4, 0) = a(0, 4);
  _parameters(5, 0) = a(0, 5);

  _parameters(6, 0) = a(1, 0);
  _parameters(7, 0) = a(1, 1);
  _parameters(8, 0) = a(1, 2);
  _parameters(9, 0) = a(1, 3);
  _parameters(10, 0) = a(1, 4);
  _parameters(11, 0) = a(1, 5);
}

void c_quadratic_image_transform::set_matrix(const cv::Mat1f & a)
{
  if ( a.rows != 2 || a.cols != 6 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x6", a.rows, a.cols);
    return;
  }

  if ( _parameters.rows != 12 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(12, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = a(0, 3);
  _parameters(4, 0) = a(0, 4);
  _parameters(5, 0) = a(0, 5);

  _parameters(6, 0) = a(1, 0);
  _parameters(7, 0) = a(1, 1);
  _parameters(8, 0) = a(1, 2);
  _parameters(9, 0) = a(1, 3);
  _parameters(10, 0) = a(1, 4);
  _parameters(11, 0) = a(1, 5);
}

cv::Matx26f c_quadratic_image_transform::matrix() const
{
  return cv::Matx26f((const float*) _parameters.data);
}

cv::Matx26f c_quadratic_image_transform::matrix(const cv::Mat1f & p) const
{
  return cv::Matx26f((const float*) p.data);
}

void c_quadratic_image_transform::set_affine_matrix(const cv::Matx23f & a)
{
  if ( _parameters.rows != 12 || _parameters.cols != 1 ) {
    _parameters.release();
    _parameters.create(12, 1);
  }

  _parameters(0, 0) = a(0, 0);
  _parameters(1, 0) = a(0, 1);
  _parameters(2, 0) = a(0, 2);
  _parameters(3, 0) = 0;
  _parameters(4, 0) = 0;
  _parameters(5, 0) = 0;

  _parameters(6, 0) = a(1, 0);
  _parameters(7, 0) = a(1, 1);
  _parameters(8, 0) = a(1, 2);
  _parameters(9, 0) = 0;
  _parameters(10, 0) = 0;
  _parameters(11, 0) = 0;
}

cv::Matx23f c_quadratic_image_transform::affine_matrix() const
{
  const cv::Mat1f & a = _parameters;
  return cv::Matx23f(a(0, 0), a(1, 0), a(2, 0),
      a(6, 0), a(7, 0), a(8, 0));
}

bool c_quadratic_image_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows != 12 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 12x1", p.rows, p.cols);
    return false;
  }

  p.copyTo(_parameters);
  return true;
}

void c_quadratic_image_transform::scale_transfrom(double factor)
{
  _parameters(2, 0) *= factor;
  _parameters(3, 0) /= factor;
  _parameters(4, 0) /= factor;
  _parameters(5, 0) /= factor;
  _parameters(8, 0) *= factor;
  _parameters(9, 0) /= factor;
  _parameters(10, 0) /= factor;
  _parameters(11, 0) /= factor;
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

bool c_quadratic_image_transform::create_remap(const cv::Matx26f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  rmap.create(size);

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      cv::Vec2f * mp = rmap[y];
      for ( int x = 0; x < rmap.cols; ++x ) {
        mp[x][0] = a(0,0) * x + a(0,1) * y + a(0,2) + a(0,3) * x * y + a(0,4) * x * x + a(0,5) * y * y;
        mp[x][1] = a(1,0) * x + a(1,1) * y + a(1,2) + a(1,3) * x * y + a(1,4) * x * x + a(1,5) * y * y;
      }
    }
  });

  return true;
}

bool c_quadratic_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  if( p.rows != 12 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 12x1", p.rows, p.cols);
    return false;
  }

  return create_remap(matrix(p), size, rmap);
}

bool c_quadratic_image_transform::remap(const cv::Matx26f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  cpts.resize(rpts.size());

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];
    cp.x = a(0,0) * rp.x + a(0,1) * rp.y + a(0,2) + a(0,3) * rp.x * rp.y + a(0,4) * rp.x * rp.x + a(0,5) * rp.y * rp.y;
    cp.y = a(1,0) * rp.x + a(1,1) * rp.y + a(1,2) + a(1,3) * rp.x * rp.y + a(1,4) * rp.x * rp.x + a(1,5) * rp.y * rp.y;
  }

  return true;
}

bool c_quadratic_image_transform::remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
{
  if( p.rows != 12 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 12x1", p.rows, p.cols);
    return false;
  }

  return remap(matrix(p), rpts, cpts);
}

bool c_quadratic_image_transform::create_steepest_descent_images(const cv::Mat1f & /*p*/, const cv::Mat1f & gx, const cv::Mat1f & gy,
    cv::Mat1f J[]) const
{
  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  const cv::Size size = gx.size();

  for( int i = 0; i < 12; ++i ) {
    J[i].create(size);
  }

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
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
  });

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

