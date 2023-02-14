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


#include "c_image_transform.h"




// For the images destined for later sharpening use only LINEAR and AREA interpolation
// to prevent high-frequency moire patterns.
enum ECC_INTERPOLATION_METHOD {
  ECC_INTER_UNKNOWN        = -1,
  ECC_INTER_NEAREST        = cv::INTER_NEAREST,
  ECC_INTER_LINEAR         = cv::INTER_LINEAR,
  ECC_INTER_CUBIC          = cv::INTER_CUBIC,
  ECC_INTER_AREA           = cv::INTER_AREA,
  ECC_INTER_LANCZOS4       = cv::INTER_LANCZOS4,
  ECC_INTER_LINEAR_EXACT   = cv::INTER_LINEAR_EXACT,
#if ( CV_VERSION_CURRRENT >= CV_VERSION_INT(4,5,0) )
  ECC_INTER_NEAREST_EXACT  = cv::INTER_NEAREST_EXACT,
#endif
};


enum ECC_BORDER_MODE {
  ECC_BORDER_UNKNOWN     = -1,
  ECC_BORDER_CONSTANT    = cv::BORDER_CONSTANT,
  ECC_BORDER_REPLICATE   = cv::BORDER_REPLICATE,
  ECC_BORDER_REFLECT     = cv::BORDER_REFLECT,
  ECC_BORDER_WRAP        = cv::BORDER_WRAP,
  ECC_BORDER_REFLECT101  = cv::BORDER_REFLECT101,
  ECC_BORDER_TRANSPARENT = cv::BORDER_TRANSPARENT,
  ECC_BORDER_DEFAULT     = cv::BORDER_DEFAULT,
  ECC_BORDER_ISOLATED    = cv::BORDER_ISOLATED
};




// Base interface
class c_ecc_motion_model
{
public:
  typedef c_ecc_motion_model this_class;
  typedef std::shared_ptr<this_class> sptr;

  virtual ~c_ecc_motion_model() = default;

  virtual cv::Mat1f parameters() const = 0;
  virtual bool set_parameters(const cv::Mat1f & p) = 0;
  virtual cv::Mat1f scale_transfrom(const cv::Mat1f & p, double factor) const = 0;
  virtual bool create_remap(cv::Mat2f & map, const cv::Size & size) const = 0;

  virtual int  num_adustable_parameters() const = 0;
  virtual bool create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const = 0;
  virtual bool update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size) = 0;
  virtual bool update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size) = 0;
};

/**
 * Base interface to both forward-additive and
 * inverse compositional ECC image alignment
 */
class c_ecc_align
{
public:
  typedef c_ecc_align this_class;
  typedef std::shared_ptr<this_class> ptr;

  c_ecc_align( c_ecc_motion_model * model = nullptr);
  virtual ~c_ecc_align() = default;

  void set_model(c_ecc_motion_model * model);
  c_ecc_motion_model * model() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_max_eps(double v);
  double max_eps() const;

  void set_min_rho(double v);
  double min_rho() const;

  void set_interpolation(enum ECC_INTERPOLATION_METHOD v);
  enum ECC_INTERPOLATION_METHOD interpolation() const;

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

  bool failed() const;
  double rho() const;
  double eps() const;
  int num_iterations() const;

  const cv::Mat2f & current_remap() const;


protected:
  // convert image to 32-bit float with optional gaussian smoothing
  virtual void prepare_input_image(cv::InputArray src, cv::InputArray src_mask,
      double smooth_sigma, bool force_create_mask,
      cv::Mat1f & dst, cv::Mat1b & dst_mask) const;

protected:
  c_ecc_motion_model * model_ = nullptr;

  enum ECC_INTERPOLATION_METHOD interpolation_ = ECC_INTER_LINEAR;
  bool failed_ = false;

  int num_iterations_  = 0;
  int max_iterations_ = 30;

  float reference_smooth_sigma_ = 1;
  float input_smooth_sigma_ = 1;
  float update_step_scale_ = 1.f;

  float max_eps_ = 0.2f;
  float min_rho_ = 0.75f;
  float eps_ = FLT_MAX;
  float rho_ = -1;

  cv::Mat2f current_remap_;
  int nparams_ = 0;

};

// Forward additive ECC
// "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2",
//          Simon Baker et.all.
// "Parametric Image Alignment Using Enhanced Correlation Coefficient Maximization",
//          Georgios D. Evangelidis and Emmanouil Z. Psarakis
class c_ecc_forward_additive :
    public c_ecc_align
{
public:
  typedef c_ecc_forward_additive this_class;
  typedef c_ecc_align base;
  typedef std::shared_ptr<this_class> ptr;

  c_ecc_forward_additive(c_ecc_motion_model * model = nullptr);

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
  cv::Mat1f H; // Hessian matrix and its inverse
  cv::Mat1f dp; // warping parameters matrix update [numberOfParameters][1]
  cv::Mat1f e, ep;  // error image and it's projection
};

// Inverse compositional ECC
// "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2", Simon Baker et.all.
// Formula (5) from the paper
//  "Fast, Robust and Accurate Digital Image Correlation Calculation Without Redundant Computations" B. Pan, K. Li, W. Tong
//  <http://www.ncorr.com/download/publications/panfast.pdf>
class c_ecc_inverse_compositional:
    public c_ecc_align
{
public:
  typedef c_ecc_inverse_compositional this_class;
  typedef c_ecc_align base;
  typedef std::shared_ptr<this_class> ptr;

  c_ecc_inverse_compositional(c_ecc_motion_model * model = nullptr);

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
class c_ecch
{
public:
  typedef c_ecch this_class;

  c_ecch(c_ecc_align * method = nullptr) ;

  void set_method(c_ecc_align * method) ;
  c_ecc_align * method() const;

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

  c_ecc_align * method_ = nullptr;
  std::vector<cv::Mat> image_pyramids_[2];
  std::vector<cv::Mat> mask_pyramids_[2];
  std::vector<cv::Mat1f> transform_pyramid_;
  int minimum_image_size_ = 12;
};


// Smooth optical flow on image pyramids
class c_eccflow
{
public:
  typedef c_eccflow this_class;

  // made public for debug purposes
  struct pyramid_entry {
    cv::Mat1f current_image, reference_image;
    cv::Mat1b current_mask, reference_mask;
    cv::Mat2f rmap;
    cv::Mat1f Ix, Iy;//, It;
    cv::Mat4f D;
  };

  void set_support_scale(int v);
  int support_scale() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_update_multiplier(double v);
  double update_multiplier() const;

  // Use for images which violate brightness constancy assumption,
  // for example on strong vignetting or planetary disk derotation
  void set_normalization_scale(int v);
  int normalization_scale() const;

  void set_input_smooth_sigma(double v);
  double input_smooth_sigma() const;

  void set_reference_smooth_sigma(double v);
  double reference_smooth_sigma() const;

  void set_max_pyramid_level(int v);
  int max_pyramid_level() const;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray());

  bool compute(cv::InputArray inputImage, cv::InputArray referenceImage, cv::Mat2f & rmap,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray());

  bool compute(cv::InputArray inputImage, cv::Mat2f & rmap,
      cv::InputArray inputMask = cv::noArray());

public: // public access mainly for debug and visualization purposes

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;
  const cv::Mat2f & current_uv() const;
  const std::vector<pyramid_entry> & current_pyramid() const;

protected:

  bool convert_input_images(cv::InputArray src, cv::InputArray srcmask,
      cv::Mat1f & dst, cv::Mat1b & dst_mask) const;

  bool compute_uv(pyramid_entry & e,
      cv::Mat2f & outuv) const;

  bool pscale(cv::InputArray src, cv::Mat & dst,
      bool ismask = false) const;

  void pnormalize(cv::InputArray src, cv::InputArray mask, cv::OutputArray dst) const;

  double input_smooth_sigma_ = 0;
  double reference_smooth_sigma_ = 0;

  double update_multiplier_ = 1.5;
  int max_iterations_ = 1; // not used at this time
  int support_scale_ = 5;
  int normalization_scale_ = -1;
  int max_pyramid_level_ = -1; // to allow force limit max pyramid size

  std::vector<pyramid_entry> pyramid_;
  cv::Mat2f cuv;
};

#endif /* __ecc2_h__ */
