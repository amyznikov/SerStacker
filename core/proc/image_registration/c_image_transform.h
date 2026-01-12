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


class c_image_transform
{
public:
  typedef c_image_transform this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

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


  cv::Mat1f parameters() const
  {
    return parameters_.clone();
  }

  bool create_remap(const cv::Size & size, cv::Mat2f & map) const
  {
    return create_remap(parameters(), size, map);
  }

  bool remap(const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const
  {
    return remap(parameters(), rpts, cpts) ;
  }

  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const
  {
    return create_steepest_descent_images(parameters(), gx, gy, J);
  }

  bool remap(cv::InputArray src, cv::InputArray src_mask, const cv::Size & size,
      cv::OutputArray dst, cv::OutputArray dst_mask,
      cv::InterpolationFlags interpolation = cv::INTER_LINEAR,
      cv::BorderTypes borderMode = cv::BORDER_CONSTANT,
      const cv::Scalar & borderValue = cv::Scalar()) const;


  virtual bool invertible() const
  {
    return false;
  }

  virtual cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
  {
    return cv::Mat1f();
  }

protected:
  cv::Mat1f parameters_;
};



/**
 * Translation transform
 *  x' = x + tx
 *  y' = y + ty
 */
class c_translation_image_transform:
    public c_image_transform
{
public:
  typedef c_translation_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_translation_image_transform(float Tx = 0, float Ty = 0);
  c_translation_image_transform(const cv::Vec2f & T);
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

  bool invertible() const final
  {
    return true;
  }

  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const
  {
    return p - dp;
  }

};


/**
 * Euclidean image transform relative to center [Cx, Cy] with optional translation, rotation and scale:
 *
 *  x' =  s * (cos(angle) * (x-Cx) - sin(angle) * (y-Cy)) + tx
 *  y' =  s * (sin(angle) * (x-Cx) + cos(angle) * (y-Cy)) + ty
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

  c_euclidean_image_transform(float Tx = 0, float Ty = 0, float angle = 0, float scale = 1);
  c_euclidean_image_transform(const cv::Vec2f & T, float angle = 0, float scale = 1);
  c_euclidean_image_transform(const cv::Vec2f & C, const cv::Vec2f & T, float angle = 0, float scale = 1);

  void reset() final;

  void set_translation(const cv::Vec2f & v) override;
  cv::Vec2f translation() const  override;

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
  void set_parameters(float Tx, float Ty, float angle, float scale, float Cx, float Cy);
  bool set_parameters(const cv::Mat1f & p) final;
  void scale_transfrom(double factor) final;
  double eps(const cv::Mat1f & dp, const cv::Size & image_size) final;
  bool get_parameters(const cv::Mat1f & p, float * Tx, float * Ty, float * angle, float * scale, float * Cx, float * Cy) const;

  bool create_remap(const cv::Mat1f & p, const cv::Size & size, cv::Mat2f & rmap) const final;
  bool remap(const cv::Mat1f & p, const std::vector<cv::Point2f> & rpts, std::vector<cv::Point2f> & cpts) const final;
  bool create_steepest_descent_images(const cv::Mat1f & p, const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f J[]) const final;

protected:
  void update_parameters();

protected:
  cv::Vec2f T_;
  cv::Vec2f C_;
  float angle_ = 0;
  float scale_ = 1;
  bool fix_translation_ = false;
  bool fix_rotation_ = false;
  bool fix_scale_ = false;
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
 *  x' =  a00 * x + a01 * y + a02
 *  y' =  a10 * x + a11 * y + a12
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

  c_affine_image_transform();
  c_affine_image_transform(const float a[2][3]);
  c_affine_image_transform(const cv::Matx23f & a);
  c_affine_image_transform(float a00, float a01, float a02, float a10, float a11, float a12);
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


  bool invertible() const final
  {
    return true;
  }

  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const final;

protected:
  mutable cv::Mat1f xx, yy;
};



/**
 * Image homography transform:
 * w  =  (x * a20 + y * a21 + a22)
 * x' =  (x * a00 + y * a01 + a02) / w
 * y' =  (x * a10 + y * a11 + a12) / w
 */
class c_homography_image_transform :
    public c_image_transform
{
public:
  typedef c_homography_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_homography_image_transform();
  c_homography_image_transform(const float a[3][3]);
  c_homography_image_transform(const cv::Matx33f & a);
  c_homography_image_transform(const cv::Mat1f & a);
  c_homography_image_transform(float a00, float a01, float a02,
      float a10, float a11, float a12,
      float a20, float a21, float a22);

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

  bool invertible() const final
  {
    return true;
  }

  cv::Mat1f invert_and_compose(const cv::Mat1f & p, const cv::Mat1f & dp) const final;

protected:
  void update_parameters();

protected:
  cv::Matx33f matrix_;
};




/*
 * Semi-quadratic image transform, 2x4 = 8 parameters are estimated
 *   x' =  a00 * x + a01 * y + a02 + a03 * x * y
 *   y' =  a10 * x + a11 * y + a12 + a13 * x * y
 */
class c_semi_quadratic_image_transform :
    public c_image_transform
{
public:
  typedef c_semi_quadratic_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_semi_quadratic_image_transform();
  c_semi_quadratic_image_transform(const float a[2][4]);
  c_semi_quadratic_image_transform(const cv::Matx24f & a);
  c_semi_quadratic_image_transform(float a00, float a01, float a02, float a03,
      float a10, float a11, float a12, float a13);

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
};

/*
 * Image quadratic transform, 12 parameters are estimated
 *   x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
 *   y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y
 */
class c_quadratic_image_transform :
    public c_image_transform
{
public:
  typedef c_quadratic_image_transform this_class;
  typedef c_image_transform base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_quadratic_image_transform();
  c_quadratic_image_transform(const float a[2][6]);
  c_quadratic_image_transform(const cv::Matx26f & a);
  c_quadratic_image_transform(float a00, float a01, float a02, float a03, float a04, float a05,
      float a10, float a11, float a12, float a13, float a14, float a15);

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

  //  void set_translation(const cv::Vec2f & v) final;
  //  cv::Vec2f translation() const final;

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
