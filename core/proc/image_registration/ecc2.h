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


enum ECC_ALIGN_METHOD {
  ECC_ALIGN_FORWARD_ADDITIVE,
  ECC_ALIGN_INVERSE_COMPOSITIONAL,
  ECC_ALIGN_LM,
  ECC_ALIGN_INVERSE_COMPOSITIONAL_LM
};

/**
 * Base interface to both forward-additive and
 * inverse compositional ECC image alignment
 */
class c_ecc_align
{
public:
  typedef c_ecc_align this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecc_align(c_image_transform * transform = nullptr);
  virtual ~c_ecc_align() = default;

  virtual void set_image_transform(c_image_transform * image_transform);
  c_image_transform * image_transform() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_max_eps(double v);
  double max_eps() const;

  void set_interpolation(enum ECC_INTERPOLATION_METHOD v);
  enum ECC_INTERPOLATION_METHOD interpolation() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  virtual void copy_parameters(const this_class & rhs);

  virtual bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  virtual bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

  virtual void release_current_image();

  virtual bool align() = 0;

  virtual bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray());

  virtual bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());


  bool failed() const;
  double eps() const;
  int num_iterations() const;

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;

  const cv::Mat1f & current_image() const;
  const cv::Mat1b & current_mask() const;

//  const cv::Mat2f & current_remap() const;

  //bool create_current_remap(const cv::Size & size);

protected:
  cv::Mat1f reference_image_;
  cv::Mat1b reference_mask_;
  cv::Mat1f current_image_;
  cv::Mat1b current_mask_;

  c_image_transform * image_transform_ = nullptr;

  enum ECC_INTERPOLATION_METHOD interpolation_ = ECC_INTER_LINEAR;
  bool failed_ = false;

  int num_iterations_  = 0;
  int max_iterations_ = 30;
  float update_step_scale_ = 1.f;

  float max_eps_ = 0.2f;
  float eps_ = FLT_MAX;
  int nparams_ = 0;

};


///////////////////////////////////////////////////////////////////////////////////////////////////


// Coarse-To-Fine ECC registration using image pyramids
class c_ecch
{
public:
  typedef c_ecch this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecch(c_image_transform * image_transform = nullptr);
  c_ecch(ECC_ALIGN_METHOD method);
  c_ecch(c_image_transform * image_transform, ECC_ALIGN_METHOD method);

  virtual ~c_ecch() = default;

  void set_image_transform(c_image_transform * image_transform);
  c_image_transform * image_transform() const;

  void set_method(ECC_ALIGN_METHOD v);
  ECC_ALIGN_METHOD method() const;

  void set_maxlevel(int v);
  int maxlevel() const;

  void set_minimum_image_size(int v);
  int minimum_image_size() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_epsx(double v);
  double epsx() const;

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

  void copy_parameters(const this_class & rhs);

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool set_current_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool align();

  bool align(cv::InputArray current_image, cv::InputArray current_mask);

  bool align(cv::InputArray current_image, cv::InputArray current_mask,
      cv::InputArray reference_image, cv::InputArray reference_mask);

  void clear()
  {
    pyramid_.clear();
    image_transform_ = nullptr;
  }

  double eps() const;
  int num_iterations() const;

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;

  const cv::Mat1f & current_image() const;
  const cv::Mat1b & current_mask() const;

  cv::Mat2f create_remap() const;

protected:
  c_ecc_align::uptr create_ecc_align(double epsx) const;

protected:
  std::vector<c_ecc_align::uptr> pyramid_;
  c_image_transform  * image_transform_ = nullptr;
  ECC_ALIGN_METHOD method_ = ECC_ALIGN_FORWARD_ADDITIVE;

  double epsx_ = 1e-5;
  double min_rho_ = 0.8;
  double reference_smooth_sigma_ = 1;
  double input_smooth_sigma_ = 1;
  double update_step_scale_ = 1.f;
  enum ECC_INTERPOLATION_METHOD interpolation_ = ECC_INTER_LINEAR;

  int max_iterations_ = 50;
  int minimum_image_size_ = 8;
  int maxlevel_ = 0;
  int num_iterations_ = -1;
};

///////////////////////////////////////////////////////////////////////////////////////////////////




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
  typedef std::shared_ptr<this_class> sptr;

  c_ecc_forward_additive(c_image_transform * image_transform = nullptr);

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray()) override;

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

  bool align() override;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

protected:
  cv::Mat1b wmask, iwmask; // image masks
  cv::Mat1f gw; // warped input image
  cv::Mat1f /*gx, gy, */gxw, gyw; // input image derivatives
  cv::Mat1f H; // Hessian matrix and its inverse
  cv::Mat1f dp; // warping parameters matrix update [numberOfParameters][1]
  cv::Mat1f rhs, ep;  // error image and it's projection
  std::vector<cv::Mat1f> jac;
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
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecc_inverse_compositional(c_image_transform * image_transform = nullptr);

  void set_image_transform(c_image_transform * image_transform) override;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray()) override;

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

  bool align() override;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

protected:

protected:
  cv::Mat1f gx, gy;
  std::vector<cv::Mat1f> jac;
  cv::Mat1f H;
};


class c_ecclm :
    public c_ecc_align
{
public:
  typedef c_ecclm this_class;
  typedef c_ecc_align base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  //c_ecclm();
  c_ecclm(c_image_transform * transform = nullptr);
  virtual ~c_ecclm() = default;

  void set_image_transform(c_image_transform * image_transform) override;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray()) override;

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

  bool align() override;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray current_image, cv::InputArray current_mask);

protected:
  double compute_rhs(const cv::Mat1f & params);
  double compute_jac(const cv::Mat1f & params, bool recompute_remap, cv::Mat1f & H, cv::Mat1f & v);
  double compute_remap(const cv::Mat1f & params, cv::Mat1f & remapped_image, cv::Mat1b & remapped_mask, cv::Mat1f & rhs);

protected:
  cv::Mat1f gx_, gy_;
  std::vector<cv::Mat1f> J;
  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;
  cv::Mat1f rhs;
  // double nrms = 0;
  double rms = 0;
};


class c_ecclm_inverse_compositional:
    public c_ecc_align
{
public:
  typedef c_ecclm_inverse_compositional this_class;
  typedef c_ecc_align base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclm_inverse_compositional(c_image_transform * image_transform = nullptr);

  void set_image_transform(c_image_transform * image_transform) override;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray()) override;

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

  bool align() override;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) override;

protected:
  double compute_rhs(const cv::Mat1f & params);
  double compute_v(const cv::Mat1f & params, bool recompute_remap, cv::Mat1f & v);
  void compute_remap(const cv::Mat1f & params, cv::Mat1f & remapped_image, cv::Mat1b & remapped_mask, cv::Mat1f & rhs);

protected:
  cv::Mat1f gx, gy;
  std::vector<cv::Mat1f> jac;
  cv::Mat1f remapped_image;
  cv::Mat1b remapped_mask;
  cv::Mat1f rhs;
  cv::Mat1f Hp;
  double rms = 0;
  double RMA = 0;
  double CMA = 0;
};











/**
 * Coarse-to-fine SMOOTH optical flow on image pyramids
 */

class c_eccflow
{
public:
  typedef c_eccflow this_class;

  enum DownscaleMethod {
    DownscaleRecursiveResize,
    DownscaleFullResize,
    DownscalePyramid
  };

  // made public for debug purposes
  struct pyramid_entry {
    cv::Mat1f current_image, reference_image;
    cv::Mat1b current_mask, reference_mask;
    cv::Mat1f Ix, Iy;//, It;
    cv::Mat4f D;
  };

  void set_support_scale(int v);
  int support_scale() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_update_multiplier(double v);
  double update_multiplier() const;

  void set_input_smooth_sigma(double v);
  double input_smooth_sigma() const;

  void set_reference_smooth_sigma(double v);
  double reference_smooth_sigma() const;

  void set_downscale_method(DownscaleMethod v);
  DownscaleMethod downscale_method() const;

  void set_scale_factor(double v);
  double scale_factor() const;

  void set_min_image_size(int v);
  int min_image_size() const;

  void set_max_pyramid_level(int v);
  int max_pyramid_level() const;

  void set_noise_level(double v);
  double noise_level() const;

  void copy_parameters(const this_class & rhs);

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  bool compute(cv::InputArray input_image, cv::InputArray reference_image, cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray(), cv::InputArray reference_mask = cv::noArray());

  bool compute(cv::InputArray input_image, cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray());

public: // public access mainly for debug and visualization purposes

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;
  const cv::Mat2f & current_uv() const;
  const std::vector<pyramid_entry> & current_pyramid() const;

protected:

  bool setup_input_image(cv::InputArray inputImage,
      cv::InputArray inputMask = cv::noArray());

  bool convert_input_images(cv::InputArray src, cv::InputArray src_mask,
      cv::Mat1f & dst, cv::Mat1b & dst_mask) const;

  bool compute_uv(pyramid_entry & e, const cv::Mat2f & rmap, cv::Mat2f & uv) const;

  void downscale(cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask,
      const cv::Size & dst_size = cv::Size()) const;

  void upscale(cv::InputArray src, cv::InputArray src_mask,
      cv::OutputArray dst, cv::OutputArray dst_mask,
      const cv::Size & dst_size) const;

  void avgdown(cv::InputArray src, cv::Mat & dst) const;
  void avgup(cv::Mat & image, const cv::Size & dstSize) const;
  void avgp(cv::InputArray src1, cv::InputArray src2, cv::Mat & dst) const;

protected:
  double input_smooth_sigma_ = 0;
  double reference_smooth_sigma_ = 0;
  double update_multiplier_ = 1.5;
  double scale_factor_ = 0.5;
  double noise_level_ = -1;
  int max_iterations_ = 1; // not used at this time
  int support_scale_ = 5;
  int min_image_size_ = 4;
  int max_pyramid_level_ = -1;

  DownscaleMethod downscale_method_ =
      DownscaleRecursiveResize;

  std::vector<pyramid_entry> pyramid_;
  cv::Mat2f uv;

  // work arrays
  mutable cv::Mat1b M;
  mutable cv::Mat1f W, It, Itx, Ity;
  mutable cv::Mat1f DC[4];

};

bool ecc_convert_input_image(cv::InputArray src, cv::InputArray src_mask, cv::Mat1f & dst, cv::Mat1b & dst_mask);
void ecc_normalize_meanstdev(cv::InputArray src, cv::InputArray src_mask, cv::OutputArray dst, int lvl, double eps);
void ecc_remap_to_optflow(const cv::Mat2f & rmap, cv::Mat2f & flow);
void ecc_flow_to_remap(const cv::Mat2f & flow, cv::Mat2f & rmap);
double compute_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask);
double compute_correlation(cv::InputArray current_image, cv::InputArray current_mask, cv::InputArray reference_image, cv::InputArray reference_mask,  const cv::Mat2f & rmap);

#endif /* __ecc2_h__ */
