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

void c_translation_image_transform::set_reference_point(const cv::Vec2f & C)
{
  if( C != _C ) {
    _parameters(0, 0) += C[0] - _C[0];
    _parameters(1, 0) += C[1] - _C[1];
    _C = C;
  }
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
  _C *= factor;
}

double c_translation_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  return sqrt(dp(0, 0) * dp(0, 0) + dp(1, 0) * dp(1, 0));
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
  //  x_cur =  x_ref - Cx + Tx
  //  y_cur =  y_ref - Cy + Ty

  rmap.create(size);

  const float Tx = T[0] - _C[0];
  const float Ty = T[1] - _C[1];

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float *)(rmap[y]);
      for( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        mp[0] = x + Tx;
        mp[1] = y + Ty;
      }
    }
  });

  return true;
}

bool c_translation_image_transform::remap(const cv::Vec2f & T, const std::vector<cv::Point2f> & rpts,
    std::vector<cv::Point2f> & cpts) const
{
  INSTRUMENT_REGION("");
  //  x_cur = x_ref - Cx_ref + Tx
  //  y_cur = y_ref - Cy_ref + Ty

  cpts.resize(rpts.size());

  const float Tx = T[0] - _C[0];
  const float Ty = T[1] - _C[1];

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    auto & cp = cpts[i];
    cp.x = rp.x + Tx;
    cp.y = rp.y + Ty;
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

c_euclidean_image_transform::c_euclidean_image_transform(float Tx, float Ty, float angle, float scale,
    const cv::Vec2f & C) : base(C)
{
  set_parameters(Tx, Ty, angle, scale);
}

c_euclidean_image_transform::c_euclidean_image_transform(const cv::Vec2f & T, float angle, float scale,
    const cv::Vec2f & C) : base(C)
{
  set_parameters(T[0], T[1], angle, scale);
}

void c_euclidean_image_transform::reset()
{
  set_parameters(0, 0, 0, 1);
}

// Ths should update current translation component when reference point C is changed
void c_euclidean_image_transform::set_reference_point(const cv::Vec2f & C)
{
  if( C != _C ) {
    const float dx = C[0] - _C[0];
    const float dy = C[1] - _C[1];
    const float ca = _scale * std::cos(_angle);
    const float sa = _scale * std::sin(_angle);
    _T[0] += ca * dx - sa * dy;
    _T[1] += sa * dx + ca * dy;
    _C = C;
    update_parameters();
  }
}

void c_euclidean_image_transform::scale_transfrom(double factor)
{
  _T *= factor;
  _C *= factor;
  update_parameters();
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


void c_euclidean_image_transform::set_parameters(float Tx, float Ty, float angle, float scale)
{
  _T(0) = Tx;
  _T(1) = Ty;
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

bool c_euclidean_image_transform::get_parameters(const cv::Mat1f & p, float * Tx, float * Ty,
    float * angle, float * scale) const
{
  const int np = num_adjustable_parameters();

  if( p.rows != np || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, np);
    return false;
  }

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
  if( _fix_rotation ) {
    return _angle;
  }
  return _fix_translation ?  _parameters(0, 0) : _parameters(2, 0);
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

double c_euclidean_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  float dTx = 0.0f, dTy = 0.0f, da = 0.0f, ds = 0.0f;

  if ( !get_parameters(dp, &dTx, &dTy, &da, &ds) ) {
    CF_ERROR("c_euclidean_image_transform::eps: get_parameters failed");
    return 0.0;
  }

  if ( _fix_scale ) {
    ds = 0;
  }

  const float w = image_size.width * 0.5f;
  const float h = image_size.height * 0.5f;
  const float maxr2 = w * w + h * h;
  const float sda = std::sin(da);

  const float error_translation = dTx * dTx + dTy * dTy;
  const float error_rotation = maxr2 * sda * sda;
  const float error_scale = maxr2 * ds * ds;
  return std::sqrt(error_translation + error_rotation + error_scale);
}


bool c_euclidean_image_transform::create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const
{
  //  Wx =  s * ( ca * x  - sa * y ) + Tx
  //  Wy =  s * ( sa * x  + ca * y ) + Ty

  float Tx = 0.0f, Ty = 0.0f, angle = 0.0f, scale = 1.0f;

  if( !get_parameters(p, &Tx, &Ty, &angle, &scale) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }

  rmap.create(size);

  const float sa = std::sin(angle);
  const float ca = std::cos(angle);
  const float Cx = _C[0]; // Cx_ref
  const float Cy = _C[1]; // Cy_ref

//  CF_DEBUG("\n"
//      "Cx=%g Cy=%g Tx=%g Ty=%g angle=%g scale=%g",
//      Cx, Cy, Tx, Ty, angle, scale);

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
  //  Wx =  s * (ca * x  - sa * y) + Tx
  //  Wy =  s * (sa * x  + ca * y) + Ty

  float Tx, Ty, angle, scale;

  if ( !get_parameters(p, &Tx, &Ty, &angle, &scale) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }

  cpts.resize(rpts.size());

  const float sa = std::sin(angle);
  const float ca = std::cos(angle);
  const float Cx = _C[0];
  const float Cy = _C[1];

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    const float xx = rp.x - Cx;
    const float yy = rp.y - Cy;
    auto & cp = cpts[i];
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

  float Tx, Ty, angle, scale;

  if ( !get_parameters(p, &Tx, &Ty, &angle, &scale) ) {
    CF_ERROR("get_parameters() fails");
    return false;
  }


  const cv::Size size = gx.size();
  for ( int i = 0; i < np; ++i ) {
    J[i].create(size);
  }

  const bool fix_translation = _fix_translation;
  const bool fix_rotation = _fix_rotation;
  const bool fix_scale = _fix_scale;
  const float sa = std::sin(angle);
  const float ca = std::cos(angle);
  const float Cx = _C[0]; // Cx_ref
  const float Cy = _C[1]; // Cy_ref

//
//  CF_DEBUG("\n"
//      "Cx=%g Cy=%g Tx=%g Ty=%g angle=%g scale=%g",
//      Cx, Cy, Tx, Ty, angle, scale);

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      int i = 0;

      const float * gxp = gx[y];
      const float * gyp = gy[y];

      if( !fix_translation ) {
        for ( int x = 0; x < size.width; ++x ) {
          J[0][y][x] = gxp[x];
          J[1][y][x] = gyp[x];
        }
        i += 2;
      }

      if( !fix_rotation ) {
        const float yy = y - Cy;
        for ( int x = 0; x < size.width; ++x ) {
          const float xx = x - Cx;
          J[i][y][x] = scale * ( -gxp[x] * (sa * xx + ca * yy) + gyp[x] * (ca * xx - sa * yy));
        }
        i += 1;
      }

      if( !fix_scale ) {
        const float yy = y - Cy;
        for ( int x = 0; x < size.width; ++x ) {
          const float xx = x - Cx;
          J[i][y][x] = gxp[x] * (ca * xx - sa * yy) + gyp[x] * (sa * xx + ca * yy);
        }
      }
    }
  });

  return true;
}

cv::Mat1f c_euclidean_image_transform::invert(const cv::Mat1f & p) const
{
  float Tx = 0.0f, Ty = 0.0f, angle = 0.0f, scale = 1.0f;
  if ( !get_parameters(p, &Tx, &Ty, &angle, &scale) ) {
    CF_ERROR("c_euclidean_image_transform::invert: get_parameters failed");
    return cv::Mat1f();
  }

  const float ca = scale * std::cos(angle);
  const float sa = scale * std::sin(angle);
  const float Cx = _C[0];
  const float Cy = _C[1];
  const float m02 = Tx - (ca * Cx - sa * Cy);
  const float m12 = Ty - (sa * Cx + ca * Cy);
  const cv::Matx23f M_full(ca, -sa, m02, sa,  ca, m12);

  cv::Matx23f M_full_inv;
  cv::invertAffineTransform(M_full, M_full_inv);

  const float inv_m00 = M_full_inv(0, 0);
  const float inv_m10 = M_full_inv(1, 0);
  constexpr float safe_min = std::numeric_limits<float>::min();
  const float res_scale = std::sqrt(inv_m00 * inv_m00 + inv_m10 * inv_m10);
  const float res_angle = (res_scale > safe_min) ? std::atan2(inv_m10, inv_m00) : 0.0f;

  const float res_ca = res_scale * std::cos(res_angle);
  const float res_sa = res_scale * std::sin(res_angle);

  const float res_Tx = M_full_inv(0, 2) + (res_ca * Cx - res_sa * Cy);
  const float res_Ty = M_full_inv(1, 2) + (res_sa * Cx + res_ca * Cy);

  const int np = num_adjustable_parameters();
  cv::Mat1f res_p(np, 1);
  int ip = 0;

  if ( !_fix_translation ) {
    res_p(ip++, 0) = res_Tx;
    res_p(ip++, 0) = res_Ty;
  }
  if ( !_fix_rotation ) {
    res_p(ip++, 0) = res_angle;
  }
  if ( !_fix_scale ) {
    res_p(ip++, 0) = res_scale;
  }

  return res_p;
}

cv::Mat1f c_euclidean_image_transform::invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
{
  // Current parameters p
  //  *  ca = cos(angle)
  //  *  sa = sin(angle)
  //  *  x_cur = s * ((x_ref - Cx) * ca - (y_ref - Cy) * sa) + Tx_cur
  //  *  y_cur = s * ((x_ref - Cx) * sa + (y_ref - Cy) * ca) + Ty_cur

  float Tx, Ty, a, s, dTx, dTy, da, ds;
  if( !get_parameters(p, &Tx, &Ty, &a, &s) ) {
    CF_ERROR("invert_and_compose: get_parameters(p) failed");
    return cv::Mat1f();
  }

  if( !get_parameters(dp, &dTx, &dTy, &da, &ds) ) {
    CF_ERROR("invert_and_compose: get_parameters(p) failed");
    return cv::Mat1f();
  }

  if( _fix_translation ) {
    dTx = 0.0f;
    dTy = 0.0f;
  }
  if( _fix_rotation ) {
    da = 0.0f;
  }
  if( _fix_scale ) {
    ds = 0.0f;
  }

  const float Cx = _C[0];
  const float Cy = _C[1];

  const float ca = s * std::cos(a);
  const float sa = s * std::sin(a);

  const cv::Matx33f M(
      ca, -sa, Tx - ca * Cx + sa * Cy,
      sa, ca, Ty - sa * Cx - ca * Cy,
      0, 0, 1);

  const float cda = (1 + ds) * std::cos(da);
  const float sda = (1 + ds) * std::sin(da);

  const cv::Matx33f dM(
      cda, -sda, dTx - cda * Cx + sda * Cy,
      sda, cda, dTy - sda * Cx + cda * Cy,
      0, 0, 1);

  const cv::Matx33f MM = M * dM.inv();

  // Extract parameters back
  const float m00 = MM(0, 0);
  const float m10 = MM(1, 0);

  const float res_s = _fix_scale ? s : std::sqrt(m00 * m00 + m10 * m10);
  const float res_a = _fix_rotation ? a : std::atan2(m10, m00);
  const float res_ca = res_s * std::cos(res_a);
  const float res_sa = res_s * std::sin(res_a);
  const float res_Tx = _fix_translation ? Tx : MM(0, 2) + res_ca * Cx - res_sa * Cy;
  const float res_Ty = _fix_translation ? Ty : MM(1, 2) + res_sa * Cx + res_ca * Cy;

  // Pack
  const int np = num_adjustable_parameters();
  cv::Mat1f res_p(np, 1);
  int ip = 0;

  if( !_fix_translation ) {
    res_p(ip++, 0) = res_Tx;
    res_p(ip++, 0) = res_Ty;
  }
  if( !_fix_rotation ) {
    res_p(ip++, 0) = res_a;
  }
  if( !_fix_scale ) {
    res_p(ip++, 0) = res_s;
  }

  return res_p;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_affine_image_transform::c_affine_image_transform(const cv::Vec2f & C) :
    base(C)
{
  set_matrix(cv::Matx23f::eye());
}

c_affine_image_transform::c_affine_image_transform(const float data[2][3], const cv::Vec2f & C) :
    base(C)
{
  set_matrix(cv::Matx23f((const float*) data));
}

c_affine_image_transform::c_affine_image_transform(const cv::Matx23f & a, const cv::Vec2f & C) :
    base(C)
{
  set_matrix(a);
}

c_affine_image_transform::c_affine_image_transform(float a00, float a01, float a02,
    float a10, float a11, float a12,
    const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx23f(
      a00, a01, a02,
      a10, a11, a12));
}

void c_affine_image_transform::reset()
{
  set_matrix(cv::Matx23f::eye());
}


void c_affine_image_transform::set_reference_point(const cv::Vec2f & C)
{
  // t_new = t_old + (A - I) * (C - _C)
  if( C != _C ) {
    const float dx = C[0] - _C[0];
    const float dy = C[1] - _C[1];
    const float a00 = _parameters(0, 0);
    const float a01 = _parameters(1, 0);
    const float a10 = _parameters(3, 0);
    const float a11 = _parameters(4, 0);
    _parameters(2, 0) += (a00 - 1.0f) * dx + a01 * dy;
    _parameters(5, 0) += a10 * dx + (a11 - 1.0f) * dy;
    _C = C;
  }
}

cv::Mat1f c_affine_image_transform::invert(const cv::Mat1f & p) const
{
  cv::Matx23f a;
  cv::invertAffineTransform(matrix(p), a);
  return cv::Mat1f(a, true).reshape(1, 6);
}

cv::Mat1f c_affine_image_transform::invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
{
  cv::Matx23f a;
  cv::invertAffineTransform(matrix(p), a);
  cv::invertAffineTransform(a + matrix(dp), a);
  return cv::Mat1f(a, true).reshape(1, 6);
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
  _C *= scale;
}

double c_affine_image_transform::eps(const cv::Mat1f & dp, const cv::Size & image_size)
{
  const float eps =
      std::sqrt(square(image_size.width * dp(0, 0)) + square(image_size.height * dp(1, 0)) + square(dp(2, 0)) +
          square(image_size.width * dp(3, 0)) + square(image_size.height * dp(4, 0)) + square(dp(5, 0)));

  return eps;
}

bool c_affine_image_transform::create_remap(const cv::Matx23f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  INSTRUMENT_REGION("");

  // x_cur =  a00 * (x_ref-Cx_ref) + a01 * (y_ref-Cy_ref) + a02
  // y_cur =  a10 * (x_ref-Cx_ref) + a11 * (y_ref-Cy_ref) + a12

  rmap.create(size);

  const float Cx = _C[0];
  const float Cy = _C[1];

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float * )rmap[y];
      const float yy = y - Cy;
      for( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        const float xx = x - Cx;
        mp[0] = a(0, 0) * xx + a(0, 1) * yy + a(0, 2);
        mp[1] = a(1, 0) * xx + a(1, 1) * yy + a(1, 2);
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
  // x_cur =  a00 * (x_ref-Cx_ref) + a01 * (y_ref-Cy_ref) + a02
  // y_cur =  a10 * (x_ref-Cx_ref) + a11 * (y_ref-Cy_ref) + a12

  cpts.resize(rpts.size());
  const float Cx = _C[0];
  const float Cy = _C[1];

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    const float xx = rp.x - Cx;
    const float yy = rp.y - Cy;
    auto & cp = cpts[i];
    cp.x = a(0, 0) * xx + a(0, 1) * yy + a(0, 2);
    cp.y = a(1, 0) * xx + a(1, 1) * yy + a(1, 2);
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

  const float Cx = _C[0];
  const float Cy = _C[1];

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      // Derivatives are computed in a local coordinate system
      const float yy = y - Cy;
      for( int x = 0; x < size.width; ++x ) {
        const float xx = x - Cx;

        J[0][y][x] = gx[y][x] * xx;  // dWx / da00
        J[1][y][x] = gx[y][x] * yy;  // dWx / da01
        J[2][y][x] = gx[y][x];       // dWx / da02

        J[3][y][x] = gy[y][x] * xx;  // dWy / da10
        J[4][y][x] = gy[y][x] * yy;  // dWy / da11
        J[5][y][x] = gy[y][x];       // dWy / da12
      }
    }
  });

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_homography_image_transform::c_homography_image_transform(const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx33f::eye());
}

c_homography_image_transform::c_homography_image_transform(const float data[3][3],
    const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx33f((const float*) data));
}

c_homography_image_transform::c_homography_image_transform(const cv::Matx33f & a,
    const cv::Vec2f & C) : base(C)
{
  set_matrix(a);
}

c_homography_image_transform::c_homography_image_transform(float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22,
    const cv::Vec2f & C) : base(C)
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

void c_homography_image_transform::set_reference_point(const cv::Vec2f & C)
{
  if( C != _C ) {

    const float dx = C[0] - _C[0];
    const float dy = C[1] - _C[1];

    // Transition matrix from the new local basis to the old one (a shift by +delta)
    // [ 1  0  dx ]
    // [ 0  1  dy ]
    // [ 0  0   1 ]
    const cv::Matx33f T_forward(
        1.0f, 0.0f, dx,
        0.0f, 1.0f, dy,
        0.0f, 0.0f, 1.0f
        );

    // Transition matrix from the old basis to the new one (shift by -delta)
    // [ 1  0 -dx ]
    // [ 0  1 -dy ]
    // [ 0  0   1 ]
    const cv::Matx33f T_backward(
        1.0f, 0.0f, -dx,
        0.0f, 1.0f, -dy,
        0.0f, 0.0f, 1.0f
        );

    // New homography in a new local basis:
    // H_new = T_backward * H_old * T_forward
    cv::Matx33f H_new = T_backward * _matrix * T_forward;

    // Normalize the matrix so that the bottom-right element H(2,2)
    // strictly corresponds to the old basis
    constexpr float safe_min = std::numeric_limits<float>::min();
    if( std::abs(H_new(2, 2)) > safe_min ) {
      H_new = H_new * (_matrix(2, 2) / H_new(2, 2));
    }

    _matrix = H_new;
    _C = C;
    update_parameters();
  }
}

cv::Mat1f c_homography_image_transform::invert(const cv::Mat1f & p) const
{
  cv::Matx33f aii = matrix(p).inv();
  constexpr float safe_min = std::numeric_limits<float>::min();
  const float scale_factor = (std::abs(aii(2, 2)) > safe_min) ? (1.0f / aii(2, 2)) : 1.0f;
  aii = aii * scale_factor;
  return cv::Mat1f(aii, true).reshape(1, 9)(cv::Rect(0, 0, 1, 8));
}

cv::Mat1f c_homography_image_transform::invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
{
  // H_new = ( H_p.inv() + dH_dp ).inv()
  cv::Matx33f aii = (matrix(p).inv() + dmatrix(dp)).inv();
  constexpr float safe_min = std::numeric_limits<float>::min();
  const float scale_factor = (std::abs(aii(2, 2)) > safe_min) ? (1.0f / aii(2, 2)) : 1.0f;
  aii = aii * scale_factor;
  return cv::Mat1f(aii, true).reshape(1, 9)(cv::Rect(0, 0, 1, 8));
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
  constexpr float safe_min = std::numeric_limits<float>::min();
  const float den = std::abs(_matrix(2, 2)) > safe_min ? _matrix(2, 2) : 1.0f;
  return cv::Vec2f(_matrix(0, 2) / den, _matrix(1, 2) / den);
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
  _C[0] *= factor;
  _C[1] *= factor;
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

bool c_homography_image_transform::create_remap(const cv::Matx33f & a, const cv::Size & size, cv::Mat2f & rmap) const
{
  INSTRUMENT_REGION("");
  rmap.create(size);

  const float Cx = _C[0];
  const float Cy = _C[1];

  constexpr float safe_min =
      std::numeric_limits<float>::min();

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      cv::Vec2f * __restrict mp = rmap[y];

      const float yy = y - Cy;

      // invariant parts for the current row yy
      const float intermediate_w     = a(2, 1) * yy + a(2, 2);
      const float intermediate_num_x = a(0, 1) * yy + a(0, 2);
      const float intermediate_num_y = a(1, 1) * yy + a(1, 2);

      for ( int x = 0; x < rmap.cols; ++x ) {
        const float xx = x - Cx;
        const float den = a(2, 0) * xx + intermediate_w;
        const float w = (std::abs(den) > safe_min) ? (1.0f / den) : 0.0f;
        mp[x][0] = (a(0, 0) * xx + intermediate_num_x) * w;
        mp[x][1] = (a(1, 0) * xx + intermediate_num_y) * w;
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

  const float Cx = _C[0];
  const float Cy = _C[1];

  constexpr float safe_min = std::numeric_limits<float>::min();
  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    const float xx = rp.x - Cx;
    const float yy = rp.y - Cy;
    const float den = a(2, 0) * xx + a(2, 1) * yy + a(2, 2);
    const float w = (std::abs(den) > safe_min) ? (1.0f / den) : 0.0f;
    cpts[i].x = (a(0, 0) * xx + a(0, 1) * yy + a(0, 2)) * w;
    cpts[i].y = (a(1, 0) * xx + a(1, 1) * yy + a(1, 2)) * w;
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

  if ( p.rows != 8 || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 8x1", p.rows, p.cols);
    return false;
  }

  const cv::Size size = gx.size();
  const cv::Matx33f a(matrix(p));

  for ( int i = 0; i < 8; ++i ) {
    J[i].create(size);
  }

  const float Cx = _C[0];
  const float Cy = _C[1];

  constexpr float safe_min = std::numeric_limits<float>::min();

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {

      const float yy = y - Cy;

      for ( int x = 0; x < size.width; ++x ) {
        const float xx = x - Cx;

        const float den = xx * a(2,0) + yy * a(2,1) + 1.0f;
        const float w = (std::abs(den) > safe_min) ? (1.0f / den) : 0.0f;

        // local projections (relative to ref point )
        const float hatX = (xx * a(0,0) + yy * a(0,1) + a(0,2)) * w;
        const float hatY = (xx * a(1,0) + yy * a(1,1) + a(1,2)) * w;

        const float ggx = gx[y][x] * w;
        const float ggy = gy[y][x] * w;

        const float gg = -hatX * ggx - hatY * ggy;

        // Derivatives for X (a00, a01, a02)
        J[0][y][x] = ggx * xx;
        J[1][y][x] = ggx * yy;
        J[2][y][x] = ggx;

        // Derivatives for Y (a10, a11, a12)
        J[3][y][x] = ggy * xx;
        J[4][y][x] = ggy * yy;
        J[5][y][x] = ggy;

        // The derivatives of the denominator (a20, a21)
        // weighted to the local xx and yy
        J[6][y][x] = gg * xx;
        J[7][y][x] = gg * yy;
      }
    }
  });

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const cv::Vec2f & C) :
    base(C)
{
  set_matrix(cv::Matx24f::eye());
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const float data[2][4],
    const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx24f((const float *)data));
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(const cv::Matx24f & a,
    const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx24f(a));
}

c_semi_quadratic_image_transform::c_semi_quadratic_image_transform(float a00, float a01, float a02, float a03,
    float a10, float a11, float a12, float a13,
    const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx24f(
      a00, a01, a02, a03,
      a10, a11, a12, a13));
}


void c_semi_quadratic_image_transform::reset()
{
  set_matrix(cv::Matx24f::eye());
}

void c_semi_quadratic_image_transform::set_reference_point(const cv::Vec2f & C)
{
  if( C != _C ) {

    const float dx = C[0] - _C[0];
    const float dy = C[1] - _C[1];

    float a00 = _parameters(0, 0);
    float a01 = _parameters(1, 0);
    float a02 = _parameters(2, 0);
    float a03 = _parameters(3, 0);

    float a10 = _parameters(4, 0);
    float a11 = _parameters(5, 0);
    float a12 = _parameters(6, 0);
    float a13 = _parameters(7, 0);

    _parameters(2, 0) = a02 + (a00 - 1.0f) * dx + a01 * dy + a03 * dx * dy;
    _parameters(6, 0) = a12 + a10 * dx + (a11 - 1.0f) * dy + a13 * dx * dy;

    _parameters(0, 0) += a03 * dy; // Change with xx (a00)
    _parameters(1, 0) += a03 * dx; // Change with yy (a01)
    _parameters(4, 0) += a13 * dy; // Change with xx (a10)
    _parameters(5, 0) += a13 * dx; // Change with yy (a11)
    _C = C;
  }
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
  const cv::Mat1f & a = _parameters;
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
  _C *= factor;
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
  INSTRUMENT_REGION("");

  rmap.create(size);

  const float Cx = _C[0];
  const float Cy = _C[1];

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      float * __restrict mp = (float * )rmap[y];
      const float yy = y - Cy;
      for ( int x = 0; x < rmap.cols; ++x, mp += 2 ) {
        const float xx = x - Cx;
        mp[0] = a(0,0) * xx + a(0,1) * yy + a(0,2) + a(0,3) * xx * yy;
        mp[1] = a(1,0) * xx + a(1,1) * yy + a(1,2) + a(1,3) * xx * yy;
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

  const float Cx = _C[0];
  const float Cy = _C[1];

  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    const float xx = rp.x - Cx;
    const float yy = rp.y - Cy;
    auto & cp = cpts[i];
    cp.x = a(0,0) * xx + a(0,1) * yy + a(0,2) + a(0,3) * xx * yy;
    cp.y = a(1,0) * xx + a(1,1) * yy + a(1,2) + a(1,3) * xx * yy;
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

  const float Cx = _C[0];
  const float Cy = _C[1];

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      const float yy = y - Cy;
      for ( int x = 0; x < size.width; ++x ) {
        const float xx = x - Cx;

        const float g_x = gx[y][x];
        const float g_y = gy[y][x];

        J[0][y][x] = g_x * xx;      // a00
        J[1][y][x] = g_x * yy;      // a01
        J[2][y][x] = g_x;           // a02
        J[3][y][x] = g_x * xx * yy; // a03

        J[4][y][x] = g_y * xx;      // a10
        J[5][y][x] = g_y * yy;      // a11
        J[6][y][x] = g_y;           // a12
        J[7][y][x] = g_y * xx * yy; // a13
      }
    }
  });

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_quadratic_image_transform::c_quadratic_image_transform(const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx26f::eye());
}

c_quadratic_image_transform::c_quadratic_image_transform(const float data[2][6], const cv::Vec2f & C) :
    base(C)
{
  set_matrix(cv::Matx26f( (const float*)data));
}

c_quadratic_image_transform::c_quadratic_image_transform(const cv::Matx26f & a, const cv::Vec2f & C) :
    base(C)
{
  set_matrix(a);
}

c_quadratic_image_transform::c_quadratic_image_transform(float a00, float a01, float a02, float a03, float a04, float a05,
    float a10, float a11, float a12, float a13, float a14, float a15,
    const cv::Vec2f & C) : base(C)
{
  set_matrix(cv::Matx26f(
      a00, a01, a02, a03, a04, a05,
      a10, a11, a12, a13, a14, a15));
}

void c_quadratic_image_transform::reset()
{
  set_matrix(cv::Matx26f::eye());
}


void c_quadratic_image_transform::set_reference_point(const cv::Vec2f & C)
{
  if( C != _C ) {
    const float dx = C[0] - _C[0];
    const float dy = C[1] - _C[1];
    const float dx2 = dx * dx;
    const float dy2 = dy * dy;
    const float dx_dy = dx * dy;

    const float a00 = _parameters(0, 0);
    const float a01 = _parameters(1, 0);
    const float a02 = _parameters(2, 0);
    const float a03 = _parameters(3, 0);
    const float a04 = _parameters(4, 0);
    const float a05 = _parameters(5, 0);

    const float a10 = _parameters(6, 0);
    const float a11 = _parameters(7, 0);
    const float a12 = _parameters(8, 0);
    const float a13 = _parameters(9, 0);
    const float a14 = _parameters(10, 0);
    const float a15 = _parameters(11, 0);

    // Update local shifts (a02, a12)
    _parameters(2, 0) = a02 + (a00 - 1.0f) * dx + a01 * dy + a03 * dx_dy + a04 * dx2 + a05 * dy2;
    _parameters(8, 0) = a12 + a10 * dx + (a11 - 1.0f) * dy + a13 * dx_dy + a14 * dx2 + a15 * dy2;

    // linear part for X (a00, a10)
    _parameters(0, 0) += a03 * dy + 2.0f * a04 * dx; // a00
    _parameters(6, 0) += a13 * dy + 2.0f * a14 * dx; // a10

    // linear part for Y (a01, a11)
    _parameters(1, 0) += a03 * dx + 2.0f * a05 * dy; // a01
    _parameters(7, 0) += a13 * dx + 2.0f * a15 * dy; // a11

    _C = C;
  }
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
  // X: a00=[0], a01=[1], a02=[2], a03=[3], a04=[4], a05=[5]
  // Y: a10=[6], a11=[7], a12=[8], a13=[9], a14=[10], a15=[11]
  _parameters(2, 0) *= factor;
  _parameters(8, 0) *= factor;
  _parameters(3, 0) /= factor;
  _parameters(4, 0) /= factor;
  _parameters(5, 0) /= factor;
  _parameters(9, 0) /= factor;
  _parameters(10, 0) /= factor;
  _parameters(11, 0) /= factor;
  _C *= factor;
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

  const float Cx = _C[0];
  const float Cy = _C[1];

  parallel_for(0, rmap.rows, [=, &rmap](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      cv::Vec2f * __restrict mp = rmap[y];
      const float yy = y - Cy;
      for ( int x = 0; x < rmap.cols; ++x ) {
        const float xx = x - Cx;
        mp[x][0] = a(0,0) * xx + a(0,1) * yy + a(0,2) + a(0,3) * xx * yy + a(0,4) * xx * xx + a(0,5) * yy * yy;
        mp[x][1] = a(1,0) * xx + a(1,1) * yy + a(1,2) + a(1,3) * xx * yy + a(1,4) * xx * xx + a(1,5) * yy * yy;
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

  const float Cx = _C[0];
  const float Cy = _C[1];
  for( size_t i = 0, n = rpts.size(); i < n; ++i ) {
    const auto & rp = rpts[i];
    const float xx = rp.x - Cx;
    const float yy = rp.y - Cy;
    auto & cp = cpts[i];
    cp.x = a(0,0) * xx + a(0,1) * yy + a(0,2) + a(0,3) * xx * yy + a(0,4) * xx * xx + a(0,5) * yy * yy;
    cp.y = a(1,0) * xx + a(1,1) * yy + a(1,2) + a(1,3) * xx * yy + a(1,4) * xx * xx + a(1,5) * yy * yy;
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

  const float Cx = _C[0];
  const float Cy = _C[1];

  parallel_for(0, size.height, [=, &gx, &gy, &J](const auto & range) {
    for ( int y = rbegin(range), ny = rend(range); y < ny; ++y ) {
      const float yy = y - Cy;
      for ( int x = 0; x < size.width; ++x ) {
        const float xx = x - Cx;

        const float g_x = gx[y][x];
        const float g_y = gy[y][x];

        J[0][y][x] = g_x * xx;
        J[1][y][x] = g_x * yy;
        J[2][y][x] = g_x;
        J[3][y][x] = g_x * xx * yy;
        J[4][y][x] = g_x * xx * xx;
        J[5][y][x] = g_x * yy * yy;

        J[6][y][x] = g_y * xx;
        J[7][y][x] = g_y * yy;
        J[8][y][x] = g_y;
        J[9][y][x] = g_y * xx * yy;
        J[10][y][x] = g_y * xx * xx;
        J[11][y][x] = g_y * yy * yy;
      }
    }
  });

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

