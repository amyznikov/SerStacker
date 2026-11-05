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
#include <core/ctrlbind/ctrlbind.h>

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
  ECC_ALIGN_INVERSE_COMPOSITIONAL_LM,
  ECC_ALIGN_FORWARD_ADDITIVE_LM,
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

  virtual void release_cached_memory();

  virtual void set_image_transform(c_image_transform * image_transform);
  c_image_transform * image_transform() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_xtol(double v);
  double xtol() const;

  void set_ftol(double v);
  double ftol() const;

  void set_gtol(double v);
  double gtol() const;

  void set_interpolation(enum ECC_INTERPOLATION_METHOD v);
  enum ECC_INTERPOLATION_METHOD interpolation() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  virtual void copy_parameters(const this_class & rhs);

  virtual bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray());

  virtual bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray());

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

  const cv::Mat2f & current_remap() const;

  const cv::Mat1f & remapped_current_image() const;
  const cv::Mat1b & remapped_current_zmask() const;

protected:
  cv::Mat1f _reference_image;
  cv::Mat1b _reference_mask;
  cv::Mat1f _current_image;
  cv::Mat1b _current_mask;
  cv::Mat1f _remapped_current_image;
  cv::Mat1b _remapped_current_zmask;
  cv::Mat2f _rmap;    // Current remap
  c_image_transform * _transform = nullptr;

  double _update_step_scale = 1.25;
  double _xtol = 1e-1;
  double _ftol = 1e-4;
  double _gtol = 1e-6;
  int _max_iterations = 30;
  enum ECC_INTERPOLATION_METHOD _interpolation = ECC_INTER_LINEAR;

  double _eps = FLT_MAX;
  int _nparams = 0;
  int _num_iterations  = 0;
  bool _failed = false;
};


///////////////////////////////////////////////////////////////////////////////////////////////////

// Coarse-To-Fine ECC registration using image pyramids

enum ECCH_PYRAMID_TYPE {
  ECCH_PYRAMID_PYRDOWN, // pyrDown
  ECCH_PYRAMID_SCHARR // pyrDown on Scharr magnitude
};

struct c_ecch_options
{
  double xtol = 1e-1;   // Parameter Tolerance
  double ftol = 1e-4;   // Function Tolerance
  double gtol = 1e-6;   // Function Gradient Tolerance

  double reference_smooth_sigma = 1;
  double input_smooth_sigma = 1;
  double update_step_scale = 1.25;

  enum ECC_ALIGN_METHOD method = ECC_ALIGN_INVERSE_COMPOSITIONAL_LM;
  enum ECC_INTERPOLATION_METHOD interpolation = ECC_INTER_LINEAR;
  enum ECCH_PYRAMID_TYPE pyramid_type = ECCH_PYRAMID_PYRDOWN;

  int max_iterations = 50;
  int minimum_image_size = 16;
  int maxlevel = 0;
  bool force_direct_methods_for_coarse_levels = true;
};

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_ecch_options> & ctx)
{
  using S = c_ecch_options;

  ctlbind(ctls, "ecc method",  ctx(&S::method), "");
  ctlbind(ctls, "pyramid type",  ctx(&S::pyramid_type), "");
  ctlbind(ctls, "minimum_image_size", ctx(&S::minimum_image_size),  "");
  ctlbind(ctls, "maxlevel",  ctx(&S::maxlevel), "");
  ctlbind(ctls, "max_iterations", ctx(&S::max_iterations), "");
  ctlbind(ctls, "xtol", ctx(&S::xtol), "Stop criteria: Parameter Tolerance (max error decrement in absolute pixels between iterations)");
  ctlbind(ctls, "ftol", ctx(&S::ftol), "Stop criteria: Function Tolerance in relative error decrement");
  ctlbind(ctls, "gtol", ctx(&S::gtol), "Stop criteria: Function Gradient Tolerance");
  ctlbind(ctls, "interpolation", ctx(&S::interpolation), "");
  ctlbind(ctls, "input_smooth_sigma", ctx(&S::input_smooth_sigma), "");
  ctlbind(ctls, "reference_smooth_sigma", ctx(&S::reference_smooth_sigma), "");
  ctlbind(ctls, "update_step_scale", ctx(&S::update_step_scale), "");
  ctlbind(ctls, "coarse_direct", ctx(&S::force_direct_methods_for_coarse_levels), "force_direct_methods_for_coarse_levels");
}

/**
 * ECC on image pyramids
 * */
class c_ecch
{
public:
  typedef c_ecch this_class;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecch(c_image_transform * image_transform = nullptr);
  c_ecch(ECC_ALIGN_METHOD method);
  c_ecch(const c_ecch_options& opts);
  c_ecch(c_image_transform * image_transform, ECC_ALIGN_METHOD method);
  c_ecch(c_image_transform * image_transform, const c_ecch_options& opts);

  virtual ~c_ecch() = default;

  c_ecch_options& options()
  {
    return _opts;
  }

  const c_ecch_options& options() const
  {
    return _opts;
  }

  void set_options(const c_ecch_options& opts)
  {
    _opts = opts;
  }

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

  void set_xtol(double v);
  double xtol() const;

  void set_ftol(double v);
  double ftol() const;

  void set_gtol(double v);
  double gtol() const;

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
    _pyramid.clear();
    _image_transform = nullptr;
  }

  int num_iterations() const;
  double eps() const;

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;

  const cv::Mat1f & current_image() const;
  const cv::Mat1b & current_mask() const;

  const cv::Mat2f & current_remap() const;
  const cv::Mat1f & remapped_current_image() const;
  const cv::Mat1b & remapped_current_zmask() const;

  bool create_remap(cv::Mat2f & rmap) const;
  cv::Mat2f create_remap() const;


  static cv::Size compute_next_pyramid_layer_size(const cv::Size & currentSize)
  {
    return cv::Size(((currentSize.width + 1) >> 1) & ~0x1, ((currentSize.height + 1) >> 1) & ~0x1);
  }

  inline void downscale_image(cv::Mat & image, cv::Mat & mask,
      const cv::Size & nextSize) const;

protected:
  c_ecc_align::uptr create_ecc_align(enum ECC_ALIGN_METHOD method, double epsx) const;

protected:
  c_ecch_options _opts;
  std::vector<c_ecc_align::uptr> _pyramid;
  c_image_transform  * _image_transform = nullptr;
  double sigmaRef = 0, sigmaCur = 0;
  cv::Mat1f Gref, Gcur;
  int _total_iterations = -1;

protected:
  static const cv::Mat1f _empty_image;
  static const cv::Mat1b _empty_mask;
  static const cv::Mat2f _empty_remap;
};

/**
 * Forward Additive Levenberg-Marquardt and Lucas-Kanade (1981)
 *
 * Lucas-Kanade 20 Years On: A Unifying Framework: Part 2,
 *    Simon Baker et.all.
 *
 * Parametric Image Alignment Using Enhanced Correlation Coefficient Maximization,
 *    Georgios D. Evangelidis and Emmanouil Z. Psarakis
 */
class c_ecclm_forward_additive :
    public c_ecc_align
{
public:
  typedef c_ecclm_forward_additive this_class;
  typedef c_ecc_align base;
  typedef std::shared_ptr<this_class> sptr;

  c_ecclm_forward_additive(c_image_transform * image_transform = nullptr);

  void release_cached_memory() final;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray()) final;

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) final;

  bool align() final;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray()) final;

  bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) final;

protected:
  double compute_rhs(const cv::Mat1f & params, double * K = nullptr);

protected:
  cv::Mat1b _current_zmask;
  cv::Mat1b _reference_zmask;
  cv::Mat1f _remapped_gx, _remapped_gy; // gradients of current remapped image
  cv::Mat1f _rhs;     // Current rhs = reference - remapped_current
  std::vector<cv::Mat1f> J; // Steepest descent images
};

/**
 * Inverse compositional Levenberg-Marquardt and Baker-Matthews (2001)
 */
class c_ecclm_inverse_compositional:
    public c_ecc_align
{
public:
  typedef c_ecclm_inverse_compositional this_class;
  typedef c_ecc_align base;
  typedef std::shared_ptr<this_class> sptr;
  typedef std::unique_ptr<this_class> uptr;

  c_ecclm_inverse_compositional(c_image_transform * image_transform = nullptr);

  void release_cached_memory() final;

  void set_image_transform(c_image_transform * image_transform) final;

  bool set_reference_image(cv::InputArray reference_image,
      cv::InputArray reference_mask = cv::noArray()) final;

  bool set_current_image(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) final;


  bool align() final;

  bool align(cv::InputArray current_image, cv::InputArray reference_image,
      cv::InputArray current_mask = cv::noArray(),
      cv::InputArray reference_mask = cv::noArray()) final;

  bool align_to_reference(cv::InputArray current_image,
      cv::InputArray current_mask = cv::noArray()) final;

protected:
  double compute_rhs(const cv::Mat1f & params, double * K = nullptr);

protected:
  cv::Mat1b _current_zmask;
  cv::Mat1b _reference_zmask;
  cv::Mat1f gx, gy;
  std::vector<cv::Mat1f> J;
  cv::Mat1f _rhs;
  cv::Mat1f Hp;
  bool _reference_image_changed = true;
};



/**
 * Coarse-to-fine SMOOTH optical flow on image pyramids
 */

enum ECCFlowDownscaleMethod {
  ECCFlowDownscaleRecursiveResize,
  ECCFlowDownscaleFullResize,
  ECCFlowDownscalePyramid
};

struct c_eccflow_options
{
  double input_smooth_sigma = 0;
  double reference_smooth_sigma = 0;
  double update_multiplier = 1.5;
  double scale_factor = 0.5;
  double noise_level = -1;
  int max_iterations = 1; // not used at this time
  int support_scale = 5;
  int min_image_size = 4;
  int max_pyramid_level = -1;
  ECCFlowDownscaleMethod downscale = ECCFlowDownscaleRecursiveResize;
};

template<class RootObjectType>
static inline void ctlbind(c_ctlist<RootObjectType> & ctls, const c_ctlbind_context<RootObjectType, c_eccflow_options> & ctx)
{
  using S = c_eccflow_options;
  ctlbind(ctls, "support_scale", ctx(&S::support_scale), "");
  ctlbind(ctls, "downscale_method", ctx(&S::downscale), "");
  ctlbind(ctls, "min_image_size", ctx(&S::min_image_size), "");
  ctlbind(ctls, "max_pyramid_level", ctx(&S::max_pyramid_level), "");
  ctlbind(ctls, "scale_factor", ctx(&S::scale_factor), "");
  ctlbind(ctls, "max_iterations", ctx(&S::max_iterations), "");
  ctlbind(ctls, "noise_level", ctx(&S::noise_level), "");
  ctlbind(ctls, "input_smooth_sigma", ctx(&S::input_smooth_sigma), "");
  ctlbind(ctls, "reference_smooth_sigma", ctx(&S::reference_smooth_sigma), "");
  ctlbind(ctls, "update_multiplier", ctx(&S::update_multiplier), "");
}

bool serialize_eccflow_options(c_config_setting section, bool save,
    c_eccflow_options & opts);

class c_eccflow
{
public:
  typedef c_eccflow this_class;


  // made public for debug purposes
  struct pyramid_entry {
    cv::Mat1f current_image, reference_image;
    cv::Mat1b current_mask, reference_mask;
    cv::Mat1f Ix, Iy;//, It;
    cv::Mat4f D;
  };

  c_eccflow_options & options()
  {
    return _opts;
  }

  const c_eccflow_options & options() const
  {
    return _opts;
  }

  void set_options(const c_eccflow_options & opts)
  {
    _opts = opts;
  }

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

  void set_downscale_method(ECCFlowDownscaleMethod v);
  ECCFlowDownscaleMethod downscale_method() const;

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

  // Must be called after set_reference_image()
  bool compute(cv::InputArray input_image, cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray());

  // Sets reference image and calls compute()
  bool compute(cv::InputArray input_image, cv::InputArray reference_image, cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray(), cv::InputArray reference_mask = cv::noArray());

  // Must be called after set_reference_image()
  bool compute_uv(cv::InputArray input_image, const cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray());

  // Sets reference image and calls compute_uv()
  bool compute_uv(cv::InputArray input_image, cv::InputArray reference_image, const cv::Mat2f & rmap,
      cv::InputArray input_mask = cv::noArray(), cv::InputArray reference_mask = cv::noArray());

  const cv::Mat2f & current_uv() const;

public: // public access mainly for debug and visualization purposes
  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;
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
  c_eccflow_options _opts;
  std::vector<pyramid_entry> _pyramid;
  cv::Mat2f uv;
  mutable cv::Mat1f W;
};

//bool ecc_convert_input_image(cv::InputArray src, cv::InputArray src_mask, cv::Mat1f & dst, cv::Mat1b & dst_mask, bool force_copy = true, bool scharr_gradient = false);
void ecc_normalize(cv::InputArray src, cv::InputArray src_mask, cv::OutputArray dst, int lvl, double eps);
void ecc_remap_to_optflow(const cv::Mat2f & rmap, cv::Mat2f & flow);
void ecc_flow_to_remap(const cv::Mat2f & flow, cv::Mat2f & rmap);
//double compute_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask);
//double compute_correlation(cv::InputArray current_image, cv::InputArray current_mask, cv::InputArray reference_image, cv::InputArray reference_mask,  const cv::Mat2f & rmap);

double ecc_compute_correlation(cv::InputArray src1, cv::InputArray src2, cv::InputArray mask);
double ecc_compute_correlation(cv::InputArray current_image, cv::InputArray current_mask,
    cv::InputArray reference_image, cv::InputArray reference_mask,
    const cv::Mat2f & rmap);


bool serialize_ecch_options(c_config_setting section, bool save,
    c_ecch_options & opts);

inline bool save_settings(c_config_setting section, const c_ecch_options & opts)
{
  return serialize_ecch_options(section, true,
      const_cast<c_ecch_options & >(opts));
}

inline bool load_settings(c_config_setting section, c_ecch_options * opts)
{
  return serialize_ecch_options(section, false, *opts);
}

#endif /* __ecc2_h__ */
