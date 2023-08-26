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

  virtual ~c_image_transform() = default;

  virtual void set_translation(const cv::Vec2f & T) = 0;
  virtual cv::Vec2f translation() const = 0;

  virtual cv::Mat1f parameters() const = 0;
  virtual bool set_parameters(const cv::Mat1f & p) = 0;
  virtual cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const = 0;

  virtual bool create_remap(cv::Mat2f & map, const cv::Size & size) const = 0;

  bool scale_transfrom(double factor)
  {
    return set_parameters(scale_transfrom(parameters(), factor));
  }

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

  c_translation_image_transform(float Tx = 0, float Ty = 0);
  c_translation_image_transform(const cv::Vec2f & T);

  void set_translation(float x, float y);
  void set_translation(const cv::Vec2f & v) override;
  cv::Vec2f translation() const override;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;
  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;


protected:
  float a[2] = { 0, 0 };
  float & Tx_ = a[0];
  float & Ty_ = a[1];
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

  c_euclidean_image_transform(float Tx = 0, float Ty = 0, float angle = 0, float scale = 1);
  c_euclidean_image_transform(const cv::Vec2f & T, float angle = 0, float scale = 1);
  c_euclidean_image_transform(const cv::Vec2f & C, const cv::Vec2f & T, float angle = 0, float scale = 1);

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

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;
  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;

protected:
  float a[6] = { 0, 0, 0, 1, 0, 0 };
  float & Tx_ = a[0];
  float & Ty_ = a[1];
  float & angle_ = a[2];
  float & scale_ = a[3];
  float & Cx_ = a[4];
  float & Cy_ = a[5];

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

  c_affine_image_transform();
  c_affine_image_transform(const float a[2][3]);
  c_affine_image_transform(const cv::Matx23f & a);
  c_affine_image_transform(float a00, float a01, float a02, float a10, float a11, float a12);


  void set_affine_matrix(const cv::Matx23f & a);
  const cv::Matx23f& affine_matrix() const;

  void set_translation(const cv::Vec2f & v) override;
  cv::Vec2f translation() const  override;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;
  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;

protected:
  cv::Matx23f a = cv::Matx23f::eye();
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

  c_homography_image_transform();
  c_homography_image_transform(const float a[3][3]);
  c_homography_image_transform(const cv::Matx33f & a);
  c_homography_image_transform(const cv::Mat1f & a);
  c_homography_image_transform(float a00, float a01, float a02,
      float a10, float a11, float a12,
      float a20, float a21, float a22);

  void set_homography_matrix(const cv::Matx33f & a);
  const cv::Matx33f& homography_matrix() const;

  void set_translation(const cv::Vec2f & v) override;
  cv::Vec2f translation() const override;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;
  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;

protected:
  cv::Matx33f a = cv::Matx33f::eye();
  float &Tx_ = a(0, 2);
  float &Ty_ = a(1, 2);
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

  c_semi_quadratic_image_transform();
  c_semi_quadratic_image_transform(const float a[2][4]);
  c_semi_quadratic_image_transform(const cv::Matx24f & a);
  c_semi_quadratic_image_transform(float a00, float a01, float a02, float a03,
      float a10, float a11, float a12, float a13);

  void set_matrix(const cv::Matx24f & a);
  void set_matrix(const cv::Mat1f & a);
  const cv::Matx24f & matrix() const;

  void set_affine_matrix(const cv::Matx23f & a);
  cv::Matx23f affine_matrix() const;

  void set_translation(const cv::Vec2f & v) override;
  cv::Vec2f translation() const override;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;
  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;

protected:
  cv::Matx24f a = cv::Matx24f::eye();
  float &Tx_ = a(0, 2);
  float &Ty_ = a(1, 2);
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

  c_quadratic_image_transform();
  c_quadratic_image_transform(const float a[2][6]);
  c_quadratic_image_transform(const cv::Matx26f & a);
  c_quadratic_image_transform(float a00, float a01, float a02, float a03, float a04, float a05,
      float a10, float a11, float a12, float a13, float a14, float a15);

  void set_matrix(const cv::Matx26f & a);
  void set_matrix(const cv::Mat1f & a);
  const cv::Matx26f & matrix() const;

  void set_affine_matrix(const cv::Matx23f & a);
  cv::Matx23f affine_matrix() const;

  void set_translation(const cv::Vec2f & v) override;
  cv::Vec2f translation() const override;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;
  cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;

protected:
  cv::Matx26f a = cv::Matx26f::eye();
  float &Tx_ = a(0, 2);
  float &Ty_ = a(1, 2);
};

#endif /* __c_image_transform_h__ */
