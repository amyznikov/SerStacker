/*
 * ecc2.h
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 *
 *  Raster image alignment (registration) using enhanced correlation coefficients
 */
#pragma once
#ifndef __ecc2_h__
#define __ecc2_h__

#include <opencv2/opencv.hpp>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif



// For the images destined for later sharpening use only LINEAR and AREA interpolation
// to prevent high-frequency moire patterns.
enum ECC2_INTERPOLATION_METHOD {
  ECC2_INTER_UNKNOWN        = -1,
  ECC2_INTER_NEAREST        = cv::INTER_NEAREST,
  ECC2_INTER_LINEAR         = cv::INTER_LINEAR,
  ECC2_INTER_CUBIC          = cv::INTER_CUBIC,
  ECC2_INTER_AREA           = cv::INTER_AREA,
  ECC2_INTER_LANCZOS4       = cv::INTER_LANCZOS4,
  ECC2_INTER_LINEAR_EXACT   = cv::INTER_LINEAR_EXACT,
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0) )
  ECC2_INTER_NEAREST_EXACT  = cv::INTER_NEAREST_EXACT,
#endif
};


enum ECC2_BORDER_MODE {
  ECC2_BORDER_UNKNOWN     = -1,
  ECC2_BORDER_CONSTANT    = cv::BORDER_CONSTANT,
  ECC2_BORDER_REPLICATE   = cv::BORDER_REPLICATE,
  ECC2_BORDER_REFLECT     = cv::BORDER_REFLECT,
  ECC2_BORDER_WRAP        = cv::BORDER_WRAP,
  ECC2_BORDER_REFLECT101  = cv::BORDER_REFLECT101,
  ECC2_BORDER_TRANSPARENT = cv::BORDER_TRANSPARENT,
  ECC2_BORDER_DEFAULT     = cv::BORDER_DEFAULT,
  ECC2_BORDER_ISOLATED    = cv::BORDER_ISOLATED
};


// Base interface
class c_ecc_transform
{
public:
  typedef c_ecc_transform this_class;
  typedef std::shared_ptr<this_class> sptr;

  virtual ~c_ecc_transform() = default;


  virtual cv::Mat1f parameters() const = 0;
  virtual bool set_parameters(const cv::Mat1f & p) = 0;

  virtual cv::Mat1f scale(const cv::Mat1f & p, double factor) const;

  virtual int  num_adustable_parameters() const = 0;
  virtual bool create_remap(cv::Mat2f & map, const cv::Size & size) const = 0;
  virtual bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const = 0;
  virtual bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) = 0;
  virtual bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) = 0;
};

/**
 * Image translation:
 *  x' = x + Tx
 *  y' = y + Ty
 */
class c_ecc_translation_transform :
    public c_ecc_transform
{
public:
  typedef c_ecc_translation_transform this_class;
  typedef c_ecc_transform base;
  typedef std::shared_ptr<this_class> sptr;

  c_ecc_translation_transform(float Tx = 0, float Ty = 0);
  c_ecc_translation_transform(const cv::Vec2f & t);

  void set_translation(const cv::Vec2f & v);
  const cv::Vec2f & translation() const;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;

  cv::Mat1f scale(const cv::Mat1f & p, double factor) const override;

  int  num_adustable_parameters() const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
  cv::Vec2f T_;
};

/**
 * Image Euclidean transform with optional scale, rotation and translation :
 *  x' =  scale * cos(angle) * x - scale * sin(angle) * y + Tx
 *  y' =  scale * sin(angle) * x + scale * cos(angle) * y + Ty
 *
 *  x' =  scale * ( cos(angle) * (x - tx) - sin(angle) * (y - ty))
 *  y' =  scale * ( sin(angle) * (x - tx) + cos(angle) * (y - ty))
 *
 *  For the signs of angles see opencv doc
 *    <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
 */
class c_ecc_euclidean_transform :
    public c_ecc_transform
{
public:
  typedef c_ecc_euclidean_transform this_class;
  typedef c_ecc_transform base;
  typedef std::shared_ptr<this_class> sptr;

  c_ecc_euclidean_transform(float Tx = 0, float Ty = 0, float angle = 0, float scale = 1);
  c_ecc_euclidean_transform(const cv::Vec2f & T, float angle = 0, float scale = 1);

  void set_translation(const cv::Vec2f & v);
  cv::Vec2f translation() const;

  void set_fix_translation(bool v);
  bool fix_translation() const;

  void set_rotation(float v);
  float rotation() const;

  void set_fix_rotation(bool v);
  bool fix_rotation() const;

  void set_scale(float v);
  float scale() const;

  void set_fix_scale(bool v);
  bool fix_scale() const;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;

  cv::Mat1f scale(const cv::Mat1f & p, double factor) const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;

  int  num_adustable_parameters() const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
  // Tx, Ty, angle, scale
  float a[4] = { 0, 0, 0, 1 };
  float & Tx_ = a[0];
  float & Ty_ = a[1];
  float & angle_ = a[2];
  float & scale_ = a[3];

  bool fix_translation_ = false;
  bool fix_rotation_ = false;
  bool fix_scale_ = false;
};

/**
 * Image affine transform:
 *  x' =  a00 * x + a01 * y + a02
 *  y' =  a10 * x + a11 * y + a12
 *
 *  For the signs of angles see opencv doc
 *    <https://docs.opencv.org/master/dd/d52/tutorial_js_geometric_transformations.html>
 */
class c_ecc_affine_transform :
    public c_ecc_transform
{
public:
  typedef c_ecc_affine_transform this_class;
  typedef c_ecc_transform base;
  typedef std::shared_ptr<this_class> sptr;

  c_ecc_affine_transform();
  c_ecc_affine_transform(const float a[2][3]);
  c_ecc_affine_transform(const cv::Matx23f & a);
  c_ecc_affine_transform(const cv::Mat1f & a);
  c_ecc_affine_transform(float a00, float a01, float a02, float a10, float a11, float a12);

  void set_translation(const cv::Vec2f & v);
  cv::Vec2f translation() const;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;

  cv::Mat1f scale(const cv::Mat1f & p, double factor) const override;

  int  num_adustable_parameters() const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
  float a[2][3] = {
      { 1, 0, 0 },
      { 0, 1, 0 }
  };
};


/**
 * Image homography transform:
 * w  =  (x * a20 + y * a21 + a22)
 * x' =  (x * a00 + y * a01 + a02) / w
 * y' =  (x * a10 + y * a11 + a12) / w
 */
class c_ecc_homography_transform :
    public c_ecc_transform
{
public:
  typedef c_ecc_homography_transform this_class;
  typedef c_ecc_transform base;
  typedef std::shared_ptr<this_class> sptr;

  c_ecc_homography_transform();
  c_ecc_homography_transform(const float a[3][3]);
  c_ecc_homography_transform(const cv::Matx33f & a);
  c_ecc_homography_transform(const cv::Mat1f & a);
  c_ecc_homography_transform(float a00, float a01, float a02,
      float a10, float a11, float a12,
      float a20, float a21, float a22);

  void set_translation(const cv::Vec2f & v);
  cv::Vec2f translation() const;

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;

  cv::Mat1f scale(const cv::Mat1f & p, double factor) const override;

  int  num_adustable_parameters() const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
  float a[3][3] = {
      { 1, 0, 0 },
      { 0, 1, 0 },
      { 0, 0, 1 }
  };
};


/*
 * Image quadratic transform, 12 parameters are estimated
 *   x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
 *   y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y
 */
class c_ecc_quadratic_transform :
    public c_ecc_transform
{
public:
  typedef c_ecc_quadratic_transform this_class;
  typedef c_ecc_transform base;
  typedef std::shared_ptr<this_class> sptr;

  c_ecc_quadratic_transform();
  c_ecc_quadratic_transform(const float a[2][6]);
  c_ecc_quadratic_transform(const cv::Matx<float, 2, 6> & a);
  c_ecc_quadratic_transform(const cv::Mat1f & a);
  c_ecc_quadratic_transform(float a00, float a01, float a02, float a03, float a04, float a05,
      float a10, float a11, float a12, float a13, float a14, float a15);

  cv::Mat1f parameters() const override;
  bool set_parameters(const cv::Mat1f & p) override;

  cv::Mat1f scale(const cv::Mat1f & p, double factor) const override;

  int  num_adustable_parameters() const override;
  bool create_remap(cv::Mat2f & map, const cv::Size & size) const override;
  bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const override;
  bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) override;
  bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) override;

protected:
  float a[2][6] = {
      { 1, 0, 0, 0, 0, 0 },
      { 0, 1, 0, 0, 0, 0 },
  };
};


/**
 * Base interface to both forward-additive and
 * inverse compositional ECC image alignment
 */
class c_ecc2
{
public:
  typedef c_ecc2 this_class;
  typedef std::shared_ptr<this_class> ptr;

  c_ecc2( c_ecc_transform * transfrom = nullptr);
  virtual ~c_ecc2() = default;

  void set_transform(c_ecc_transform * transfrom);
  c_ecc_transform * transfrom() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_eps(double v);
  double eps() const;

  void set_min_rho(double v);
  double min_rho() const;

  void set_interpolation(enum ECC2_INTERPOLATION_METHOD v);
  enum ECC2_INTERPOLATION_METHOD interpolation() const;

  void set_input_smooth_sigma(double v);
  double input_smooth_sigma() const;

  void set_reference_smooth_sigma(double v);
  double reference_smooth_sigma() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  virtual bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) = 0;

  virtual bool align(cv::InputArray inputImage, cv::InputArray referenceImage,
      cv::InputArray inputMask = cv::noArray(),
      cv::InputArray referenceMask = cv::noArray()) = 0;

  virtual bool align_to_reference(cv::InputArray inputImage,
      cv::InputArray inputMask = cv::noArray()) = 0;

  double rho() const;
  double current_eps() const;
  int num_iterations() const;
  const cv::Mat2f & current_remap() const;


protected:
  // convert image to 32-bit float with optional gaussian smoothing
  virtual void prepare_input_image(cv::InputArray src, cv::InputArray src_mask,
      double smooth_sigma, bool force_create_mask,
      cv::Mat1f & dst, cv::Mat1b & dst_mask) const;

protected:
  c_ecc_transform * transfrom_ = nullptr;

  enum ECC2_INTERPOLATION_METHOD interpolation_ = ECC2_INTER_LINEAR;

  int num_iterations_  = 0;
  int max_iterations_ = 30;

  float reference_smooth_sigma_ = 1;
  float input_smooth_sigma_ = 1;
  float update_step_scale_ = 1.f;

  float eps_ = 0.2f;
  float min_rho_ = 0.75f;
  float current_eps_ = FLT_MAX;
  float rho_ = -1;

  cv::Mat2f current_remap_;
  int number_of_parameters_ = 0;

};

// Forward additive ECC
// "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2",
//          Simon Baker et.all.
// "Parametric Image Alignment Using Enhanced Correlation Coefficient Maximization",
//          Georgios D. Evangelidis and Emmanouil Z. Psarakis
class c_ecc2_forward_additive :
    public c_ecc2
{
public:
  typedef c_ecc2_forward_additive this_class;
  typedef c_ecc2 base;
  typedef std::shared_ptr<this_class> ptr;

  c_ecc2_forward_additive(c_ecc_transform * transfrom = nullptr);

  const cv::Mat1f & input_image() const;

  const cv::Mat1f & reference_image() const;

  const cv::Mat1b & input_mask() const;

  const cv::Mat1b & reference_mask() const;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray inputImage,
      cv::InputArray inputMask = cv::noArray()) override;

  bool align(cv::InputArray inputImage, cv::InputArray referenceImage,
      cv::InputArray inputMask = cv::noArray(),
      cv::InputArray referenceMask = cv::noArray()) override;


protected:
  cv::Mat1f g;  // input image
  cv::Mat1f f;  // reference image
  cv::Mat1b fmask, gmask, wmask, iwmask; // image masks
  cv::Mat1f gw; // warped input image
  cv::Mat1f gx, gy, gxw, gyw; // input image derivatives
  cv::Mat1f jac;  // jacobian [f.rows * numberOfParameters][f.cols]
  cv::Mat1f H, Hinv; // Hessian matrix and its inverse
  cv::Mat1f dp; // warping parameters matrix update [numberOfParameters][1]
  cv::Mat1f e, ep;  // error image and it's projection
};

// Inverse compositional ECC
// "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2", Simon Baker et.all.
// Formula (5) from the paper
//  "Fast, Robust and Accurate Digital Image Correlation Calculation Without Redundant Computations" B. Pan, K. Li, W. Tong
//  <http://www.ncorr.com/download/publications/panfast.pdf>
class c_ecc2_inverse_compositional:
    public c_ecc2
{
public:
  typedef c_ecc2_inverse_compositional this_class;
  typedef c_ecc2 base;
  typedef std::shared_ptr<this_class> ptr;

  c_ecc2_inverse_compositional(c_ecc_transform * transfrom = nullptr);

  const cv::Mat1f& input_image() const;

  const cv::Mat1f& reference_image() const;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) override;

  bool align(cv::InputArray inputImage, cv::InputArray referenceImage,
      cv::InputArray inputMask = cv::noArray(),
      cv::InputArray referenceMask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray inputImage,
      cv::InputArray inputMask = cv::noArray()) override;


protected: // Notations are from the paper of  B. Pan, K. Li and W. Tong
  cv::Mat1f g; // input image
  cv::Mat1f f; // reference image
  cv::Mat1b fmask, gmask, wmask, iwmask; // image masks
  cv::Mat1f gw; // warped input image
  cv::Mat1f e, ep; // error image and it's projection
  cv::Mat1f dp; // warping parameters matrix update [numberOfParameters][1], the size and meaning depends on motion model
  cv::Mat1f fx, fy; // gradient of reference image
  cv::Mat1f jac_; // steepest descend images ("jacobian")
  cv::Mat1f H; // inverse of Hessian matrix
};


// Coarse-To-Fine ECC registration using image pyramids
class c_ecch2
{
public:
  typedef c_ecch2 this_class;

  c_ecch2(c_ecc2 * method = nullptr) ;

  void set_method(c_ecc2 * method) ;
  c_ecc2 * method() const;

  void set_minimum_image_size(int v);
  int minimum_image_size() const;

  const std::vector<cv::Mat> & image_pyramid(int index) const;
  const std::vector<cv::Mat> & mask_pyramid(int index) const;
  const std::vector<cv::Mat1f> & transform_pyramid() const;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask);

  bool align(cv::InputArray inputImage,
      cv::InputArray inputMask);

protected:
  enum {
    current_image_index = 0,
    reference_image_index = 1,
  };

  c_ecc2 * method_ = nullptr;
  std::vector<cv::Mat> image_pyramids_[2];
  std::vector<cv::Mat> mask_pyramids_[2];
  std::vector<cv::Mat1f> transform_pyramid_;
  int minimum_image_size_ = 12;
};


#endif /* __ecc2_h__ */
