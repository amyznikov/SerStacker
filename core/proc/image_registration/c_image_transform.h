/*
 * c_image_transform.h
 *
 *  Created on: Feb 11, 2023
 *      Author: amyznikov
 */

#pragma once
#ifndef __c_image_transform_h__
#define __c_image_transform_h__

#include <opencv2/opencv.hpp>
#if (CV_VERSION_MAJOR >= 5)
# include <opencv2/geometry.hpp>
#endif

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

#ifndef Matx26f_defined
#define Matx26f_defined 1
namespace cv {
typedef cv::Matx<float, 2, 6> Matx26f;
}
#endif

#ifndef Matx24f_defined
#define Matx24f_defined 1
namespace cv {
typedef cv::Matx<float, 2, 4> Matx24f;
}
#endif

/**
 * Image transform relative to center [Crx, Cry] given in reference (destination)
 * coordinate system designed mainly for generating backward remaps:
 *
 *  x_cur = Wx(x_ref - Crx, y_ref - Cry, params) + Ccx
 *  y_cur = Wy(x_ref - Crx, y_ref - Cry, params) + Ccy
 */
class c_image_transform
{
public:
  typedef c_image_transform this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_image_transform() = default;

  c_image_transform(const cv::Vec2f & C) : _Cr(C), _Cc(C)
  {
  }

  virtual ~c_image_transform() = default;

  virtual void reset() = 0;
  virtual bool set_parameters(const cv::Mat1f & p) = 0;
  virtual void scale_transfrom(double factor) = 0;
  virtual bool create_remap(const cv::Mat1f & params, const cv::Size & size, cv::Mat2f & map) const = 0;
  virtual bool remap(const cv::Mat1f & params, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const = 0;
  virtual bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const = 0;
  virtual double eps(const cv::Mat1f & dp, const cv::Size & image_size) = 0;

  virtual void set_translation(const cv::Vec2f & T) = 0;
  virtual cv::Vec2f translation() const = 0;


  // Set both reference points on source and destination images
  inline void set_reference_points(const cv::Vec2f & Cr, const cv::Vec2f & Cc)
  {
    _Cr = Cr;
    _Cc = Cc;
  }

  // Set both reference points on source and destination images
  inline void set_reference_point(const cv::Vec2f & C)
  {
    _Cr = C;
    _Cc = C;
  }

  // Set reference point on reference (destination) image Cr
  inline void set_destination_reference_point(const cv::Vec2f & Cr)
  {
    _Cr = Cr;
  }

  // Get reference point on destination image Cr
  inline const cv::Vec2f & destination_reference_point() const
  {
    return _Cr;
  }

  // Set reference point on source image Cc
  inline void set_source_reference_point(const cv::Vec2f & Cc)
  {
    _Cc = Cc;
  }

  // Get reference point on source image Cc
  inline const cv::Vec2f & source_reference_point() const
  {
    return _Cc;
  }

  inline bool set_parameters(const std::vector<float> & v)
  {
    return set_parameters(cv::Mat1f(int(v.size()), 1, (float*)(v.data())));
  }

  inline const cv::Mat1f & parameters() const
  {
    return _parameters;
  }

  inline cv::Mat1f clone_parameters() const
  {
    return _parameters.clone();
  }

  inline bool create_remap(const cv::Size & size, cv::Mat2f & map) const
  {
    return create_remap(parameters(), size, map);
  }

  inline bool remap(const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
  {
    return remap(parameters(), rpts, cpts) ;
  }

  inline bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
  {
    return create_steepest_descent_images(parameters(), gx, gy, J);
  }

  bool remap(cv::InputArray src, cv::InputArray src_mask, const cv::Size & size,
      cv::OutputArray dst, cv::OutputArray dst_mask,
      cv::InterpolationFlags interpolation = cv::INTER_LINEAR,
      cv::BorderTypes borderMode = cv::BORDER_CONSTANT,
      const cv::Scalar & borderValue = cv::Scalar()) const;

  virtual cv::Mat1f invert_and_compose(const cv::Mat1f & /*p*/, const cv::Mat1f & /*dp*/) const
  {
    return cv::Mat1f();
  }

  virtual cv::Mat1f invert(const cv::Mat1f & /*p*/) const
  {
    return cv::Mat1f();
  }

  virtual bool invertible() const
  {
    return false;
  }

protected:
  cv::Mat1f _parameters;
  cv::Vec2f _Cr; // reference point on reference (destination) image
  cv::Vec2f _Cc; // reference point on current (source) image
};

/**
 * Translation transform:
 *  x_cur = x_ref - Crx + Tx + Ccx
 *  y_cur = y_ref - Cry + Ty + Ccy
 */
class c_translation_image_transform:
    public c_image_transform
{
public:
  typedef c_translation_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  using base::create_steepest_descent_images;
  using base::remap;

  c_translation_image_transform(float Tx = 0, float Ty = 0, const cv::Vec2f & C = cv::Vec2f());
  c_translation_image_transform(const cv::Vec2f & T, const cv::Vec2f & C = cv::Vec2f());
  void reset() final;

  void set_translation(float x, float y);
  void set_translation(const cv::Vec2f & v) final;
  cv::Vec2f translation() const final;

  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;

  bool create_remap(const cv::Vec2f & T, const cv::Size & size, cv::Mat2f & map) const;
  bool create_remap(const cv::Mat1f & params, const cv::Size & size, cv::Mat2f & map) const final;
  bool remap(const cv::Vec2f & T, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const;
  bool remap(const cv::Mat1f & , const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

  cv::Mat1f invert(const cv::Mat1f & p) const final
  {
    return -p;
  }

  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const final
  {
    return p - dp;
  }

  bool invertible() const final
  {
    return true;
  }

};


/**
 * Euclidean image transform relative to center [Crx, Cry] given in reference (destination)
 * coordinate system with optional translation, rotation and scale:
 *  xx = x_ref - Crx
 *  yy = y_ref - Cry
 *
 *  sca = s * cos(angle)
 *  ssa = s * sin(angle)
 *
 *  x_cur = sca * xx - ssa * yy + Tx + Ccx
 *  y_cur = ssa * xx + sca * yy + Ty + Ccy
 *
 *  For the signs of angles see opencv doc for estimateAffinePartial2D()
 *    <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
 */
class c_euclidean_image_transform :
    public c_image_transform
{
public:
  typedef c_euclidean_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  using base::create_steepest_descent_images;
  using base::remap;

  c_euclidean_image_transform(float Tx = 0, float Ty = 0, float angle = 0, float scale = 1, const cv::Vec2f & C = cv::Vec2f());
  c_euclidean_image_transform(const cv::Vec2f & T, float angle = 0, float scale = 1, const cv::Vec2f & C = cv::Vec2f());

  void reset() final;

  void set_translation(const cv::Vec2f & v) final;
  cv::Vec2f translation() const final;

  void set_center(const cv::Vec2f & v);
  cv::Vec2f center() const;

  void set_rotation(float v);
  float rotation() const;

  void set_scale(float v);
  float scale() const;

  void set_fix_translation(bool v);
  bool fix_translation() const;

  void set_fix_rotation(bool v);
  bool fix_rotation() const;

  void set_fix_scale(bool v);
  bool fix_scale() const;

  int num_adjustable_parameters() const;
  void set_parameters(float Tx, float Ty, float angle, float scale);
  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;
  bool get_parameters(const cv::Mat1f & p, float * Tx, float * Ty, float * angle, float * scale) const;

  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

  cv::Mat1f invert(const cv::Mat1f & p) const final;
  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const;
  bool invertible() const final
  {
    return true;
  }

protected:
  void update_parameters();

protected:
  cv::Vec2f _T;
  float _angle = 0;
  float _scale = 1;
  bool _fix_translation = false;
  bool _fix_rotation = false;
  bool _fix_scale = false;
};

inline cv::Matx23f create_euclidean_transform(const cv::Vec2f & C, const cv::Vec2f & T, float angle = 0, float scale = 1)
{
  const float ca = scale * std::cos(angle);
  const float sa = scale * std::sin(angle);

  return cv::Matx23f(ca, -sa, T[0] - (ca * C[0] - sa * C[1]),
      sa, ca, T[1] - (sa * C[0] + ca * C[1]));
}


/**
 * Image affine transform:
 *  xx = x_ref - Crx
 *  yy = y_ref - Cry
 *
 *  x_cur =  a00 * xx + a01 * yy + a02 + Ccx
 *  y_cur =  a10 * xx + a11 * yy + a12 + Ccy
 *
 *  For the signs of angles see opencv doc
 *    <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
 */
class c_affine_image_transform :
    public c_image_transform
{
public:
  typedef c_affine_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  using base::create_steepest_descent_images;
  using base::remap;

  c_affine_image_transform(const cv::Vec2f & C = cv::Vec2f());
  c_affine_image_transform(const float a[2][3], const cv::Vec2f & C = cv::Vec2f());
  c_affine_image_transform(const cv::Matx23f & a, const cv::Vec2f & C = cv::Vec2f());
  c_affine_image_transform(float a00, float a01, float a02, float a10, float a11, float a12,
      const cv::Vec2f & C = cv::Vec2f());

  void reset() final;

  void set_matrix(const cv::Matx23f & a);
  cv::Matx23f matrix() const;
  cv::Matx23f matrix(const cv::Mat1f & p) const;

  void set_translation(const cv::Vec2f & v) final;
  cv::Vec2f translation() const final;

  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;

  bool create_remap(const cv::Matx23f & a, const cv::Size & size, cv::Mat2f & rmap) const;
  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Matx23f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

  cv::Mat1f invert(const cv::Mat1f & p) const final;
  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const final;
  bool invertible() const final
  {
    return true;
  }

};


/**
 * Image homography transform relative to reference point [Crx, Cry]:
 *  xx = x_ref - Crx
 *  yy = y_ref - Cry
 *
 *  w  =  a20 * xx + a21 * yy + a22
 *  x_cur =  (a00 * xx + a01 * yy + a02) / w + Ccx
 *  y_cur =  (a10 * xx + a11 * yy + a12) / w + Ccy
 */
class c_homography_image_transform :
    public c_image_transform
{
public:
  typedef c_homography_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  using base::create_steepest_descent_images;
  using base::remap;

  c_homography_image_transform(const cv::Vec2f & C = cv::Vec2f());
  c_homography_image_transform(const float a[3][3], const cv::Vec2f & C = cv::Vec2f());
  c_homography_image_transform(const cv::Matx33f & a, const cv::Vec2f & C = cv::Vec2f());
  c_homography_image_transform(const cv::Mat1f & a, const cv::Vec2f & C = cv::Vec2f());
  c_homography_image_transform(float a00, float a01, float a02,
      float a10, float a11, float a12,
      float a20, float a21, float a22,
      const cv::Vec2f & C = cv::Vec2f());

  void reset() final;

  void set_matrix(const cv::Matx33f & a);
  const cv::Matx33f & matrix() const;
  cv::Matx33f matrix(const cv::Mat1f & p) const;
  cv::Matx33f dmatrix(const cv::Mat1f & dp) const;

  void set_translation(const cv::Vec2f & v) final;
  cv::Vec2f translation() const final;

  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;

  bool create_remap(const cv::Matx33f & a, const cv::Size & size, cv::Mat2f & rmap) const;
  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Matx33f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

  cv::Mat1f invert(const cv::Mat1f & p) const;
  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const;
  bool invertible() const final
  {
    return true;
  }

protected:
  void update_parameters();

protected:
  cv::Matx33f _matrix;
};

/*
 * Semi-quadratic image transform, 2x4 = 8 parameters are estimated
 *   xx = x_ref - Crx
 *   yy = y_ref - Cry
 *   x_cur = a00 * xx + a01 * yy + a02 + a03 * xx * yy + Ccx
 *   y_cur = a10 * xx + a11 * yy + a12 + a13 * xx * yy + Ccy
 */
class c_semi_quadratic_image_transform :
    public c_image_transform
{
public:
  typedef c_semi_quadratic_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  using base::create_steepest_descent_images;
  using base::remap;

  c_semi_quadratic_image_transform(const cv::Vec2f & C = cv::Vec2f());
  c_semi_quadratic_image_transform(const float a[2][4], const cv::Vec2f & C = cv::Vec2f());
  c_semi_quadratic_image_transform(const cv::Matx24f & a, const cv::Vec2f & C = cv::Vec2f());
  c_semi_quadratic_image_transform(float a00, float a01, float a02, float a03,
      float a10, float a11, float a12, float a13,
      const cv::Vec2f & C = cv::Vec2f());

  void reset() final;

  void set_matrix(const cv::Matx24f & a);
  void set_matrix(const cv::Mat1f & a);
  cv::Matx24f matrix() const;
  cv::Matx24f matrix(const cv::Mat1f & p) const;

  void set_affine_matrix(const cv::Matx23f & a);
  cv::Matx23f affine_matrix() const;

  void set_translation(const cv::Vec2f & v) final;
  cv::Vec2f translation() const final;

  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;

  bool create_remap(const cv::Matx24f & a, const cv::Size & size, cv::Mat2f & rmap) const;
  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Matx24f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const ;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

  cv::Mat1f invert(const cv::Mat1f & p) const final;
  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const final;

  bool invertible() const final
  {
    return true;
  }

};

/*
 * Image full quadratic transform, 12 parameters are estimated
 *   xx = x_ref - Crx
 *   yy = y_ref - Cry
 *   x_cur =  a00 * xx + a01 * yy + a02 + a03 * xx * yy + a04 * xx * xx + a05 * yy * yy + Ccx
 *   y_cur =  a10 * xx + a11 * yy + a12 + a13 * xx * yy + a14 * xx * xx + a15 * yy * yy + Ccy
 */
class c_quadratic_image_transform :
    public c_image_transform
{
public:
  typedef c_quadratic_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;
  using base::create_steepest_descent_images;
  using base::remap;

  c_quadratic_image_transform(const cv::Vec2f & C = cv::Vec2f());
  c_quadratic_image_transform(const float a[2][6], const cv::Vec2f & C = cv::Vec2f());
  c_quadratic_image_transform(const cv::Matx26f & a, const cv::Vec2f & C = cv::Vec2f());
  c_quadratic_image_transform(float a00, float a01, float a02, float a03, float a04, float a05,
      float a10, float a11, float a12, float a13, float a14, float a15,
      const cv::Vec2f & C = cv::Vec2f());

  void reset() final;

  void set_matrix(const cv::Matx26f & a);
  void set_matrix(const cv::Mat1f & a);
  cv::Matx26f matrix() const;
  cv::Matx26f matrix(const cv::Mat1f & p) const;

  void set_affine_matrix(const cv::Matx23f & a);
  cv::Matx23f affine_matrix() const;

  void set_translation(const cv::Vec2f & v) final;
  cv::Vec2f translation() const final;

  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;

  bool create_remap(const cv::Matx26f & a, const cv::Size & size, cv::Mat2f & rmap) const;
  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Matx26f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const ;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;
};



/**
 * Epipolar derotation image transform.
 *
 * Actually this is just a homography,
 * but specific method is required for estimation.
 *
 * w  =  (x * a20 + y * a21 + a22)
 * x' =  (x * a00 + y * a01 + a02) / w
 * y' =  (x * a10 + y * a11 + a12) / w
 */
class c_epipolar_derotation_image_transform:
    public c_homography_image_transform
{
public:
  typedef c_epipolar_derotation_image_transform this_class;
  typedef c_homography_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_epipolar_derotation_image_transform()
  {
  }

  c_epipolar_derotation_image_transform(const float a[3][3]) :
      base(a)
  {
  }

  c_epipolar_derotation_image_transform(const cv::Matx33f & a) :
      base(a)
  {
  }

  c_epipolar_derotation_image_transform(const cv::Mat1f & a) :
      base(a)
  {
  }

  c_epipolar_derotation_image_transform(float a00, float a01, float a02,
      float a10, float a11, float a12,
      float a20, float a21, float a22) :
      base(a00, a01, a02,
          a10, a11, a12,
          a20, a21, a22)
  {
  }

};


/**
 * Pure rotation homography transform:
 * w  =  (x * a20 + y * a21 + a22)
 * x' =  (x * a00 + y * a01 + a02) / w
 * y' =  (x * a10 + y * a11 + a12) / w
 */
class c_prh_image_transform :
    public c_image_transform
{
public:
  typedef c_homography_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_prh_image_transform();

  void reset() final;

  // void set_matrix(const cv::Matx33f & a);
  cv::Matx33f matrix() const;
  cv::Matx33f matrix(const cv::Mat1f & p) const;
  cv::Matx33f dmatrix(const cv::Mat1f & dp) const;

  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;

  bool create_remap(const cv::Matx33f & a, const cv::Size & size, cv::Mat2f & rmap) const;
  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Matx33f & a, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const ;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

  bool invertible() const final
  {
    return true;
  }

  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const final;

protected:
  void update_parameters();

protected:
  cv::Matx33f _K;
  cv::Vec3f _A;
};

#endif /* __c_image_transform_h__ */
