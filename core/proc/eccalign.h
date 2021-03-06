/*
 * eccalign.h
 *
 *  Created on: Jan 2, 2020
 *      Author: amyznikov
 *
 *  Raster image alignment (registration) using enhanced correlation coefficients
 */
#pragma once
#ifndef __ecc_align_h__
#define __ecc_align_h__

#include <opencv2/opencv.hpp>

// OpenCV version macro
#ifndef CV_VERSION_INT
# define CV_VERSION_INT(a,b,c) (((a)<<16)|((b)<<8)|(c))
#endif
#ifndef CV_VERSION_CURRRENT
# define CV_VERSION_CURRRENT CV_VERSION_INT(CV_VERSION_MAJOR, CV_VERSION_MINOR, CV_VERSION_REVISION)
#endif

enum ECC_MOTION_TYPE {
  ECC_MOTION_NONE = -1,
  ECC_MOTION_TRANSLATION = 0, // cv::MOTION_TRANSLATION, two parameters are estimated
  ECC_MOTION_EUCLIDEAN = 1, // cv::MOTION_EUCLIDEAN, Euclidean (rigid) transformation; three parameters are estimated
  ECC_MOTION_EUCLIDEAN_SCALED = 2, // cv::MOTION_EUCLIDEAN + SCALE; four parameters are estimated
  ECC_MOTION_AFFINE = 3,  // cv::MOTION_AFFINE, six parameters are estimated
  ECC_MOTION_HOMOGRAPHY = 4, // cv::MOTION_HOMOGRAPHY, eight parameters are estimated
  ECC_MOTION_QUADRATIC = 5, // Quadratic form, 12 parameters are estimated
};

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

//
// args:
//  motionType - enum ECC_MOTION_*
//  T - input transformation  matrix
//  rmap - output cv::Mat2f or cv::Mat2d matrix for cv::remap()
//  ddepth - destination type of rmap output matrix
bool createRemap(enum ECC_MOTION_TYPE motionType,
    cv::InputArray T,
    cv::OutputArray rmap,
    const cv::Size & size,
    int ddepth = -1 );

cv::Mat createRemap(enum ECC_MOTION_TYPE motionType,
    cv::InputArray T,
    const cv::Size & size,
    int ddepth = -1 );

bool addRemap(cv::InputArray src,
    cv::InputOutputArray dst);

void createIdentityRemap(cv::OutputArray rmap,
    const cv::Size & size,
    int ddepth = -1 );

// Initialize identity (eye) transform matrix of appropriate size for given motion type
cv::Mat createEyeTransform(enum ECC_MOTION_TYPE motionType,
    int ddepth = CV_32F);

// Initialize translation transform matrix of appropriate size for given motion type
cv::Mat createTranslationTransform(
    double Tx, double Ty,
    int ddepth = CV_32F);

// Extract translation component from current transform
bool getTranslationComponents(enum ECC_MOTION_TYPE motionType,
    const cv::Mat1f & T,
    double * tx,
    double * ty);


//
//  x' =  scale * ( cos(a) * (x-Cx) + sin(a) * (y-Cy)) + Tx
//  y' =  scale * (-sin(a) * (x-Cx) + cos(a) * (y-Cy)) + Ty
//
//  For simple rotation around of a point C call
//   createEuclideanTransform(C.x, C.y, C.x, C.y, scale, angle);
//
cv::Mat createEuclideanTransform(double Cx, double Cy,
    double Tx, double Ty,
    double scale,
    double angle,
    int ddepth = CV_32F);

// Extract Euclidean components from (scaled) Euclidean transfom matrix T of size 2x3
bool getEuclideanComponents(cv::InputArray T,
    double * Tx, double * Ty,
    double * scale,
    double * angle);


// Scale given transform matrix to reflect image scale change
void scaleTransform(enum ECC_MOTION_TYPE ecc_motion_type,
    cv::Mat1f & T,
    double scale);

// Scale given transform matrix to reflect image scale change
void scaleTransform(enum ECC_MOTION_TYPE ecc_motion_type,
    const cv::Mat1f & src,
    cv::Mat1f & dst,
    double scale);

// Expand given 2x3 affine transformation matrix T to given motion type (homography or quadratic),
// padding with zeros and/or ones as appropriate
cv::Mat1f expandAffineTransform(const cv::Mat1f T,
    enum ECC_MOTION_TYPE target_motion_type);

// apply transfromation T to a single point x,y
//bool applyTransform(double x, double y,
//    double *xt, double * yt,
//    const cv::Mat & T,
//    enum ECC_MOTION_TYPE motion_type);


// pyramide utils
bool ecc_downscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_REPLICATE);
bool ecc_upscale(cv::Mat & image, cv::Size dstSize);
bool ecc_normalize(cv::Mat1f & image, cv::InputArray _mask, int level, double regularization_term);
void ecc_differentiate(cv::InputArray src, cv::Mat & gx, cv::Mat & gy, int ddepth=-1);
///////////////////////////////////////////////////////////////////////////////


// Base interface
class c_ecc_align {
public:
  typedef c_ecc_align this_class;
  typedef std::shared_ptr<this_class> ptr;

  void set_motion_type(enum ECC_MOTION_TYPE ecc_motion_type);
  enum ECC_MOTION_TYPE motion_type() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_eps(double v);
  double eps() const;

  void set_min_rho(double v);
  double min_rho() const;

  void set_image_interpolation_method(enum ECC_INTERPOLATION_METHOD v);
  enum ECC_INTERPOLATION_METHOD image_interpolation_method() const;

  void set_input_smooth_sigma(double v);
  double input_smooth_sigma() const;

  void set_reference_smooth_sigma(double v);
  double reference_smooth_sigma() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  virtual bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) = 0;

  virtual bool align(cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray()) = 0;

  virtual bool align_to_reference(cv::InputArray inputImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray()) = 0;

  double rho() const;
  double current_eps() const;
  int num_iterations() const;
  const cv::Mat2f & current_remap() const;

  virtual ~c_ecc_align();

protected:
  enum ECC_MOTION_TYPE ecc_motion_type_ = ECC_MOTION_AFFINE;
  enum ECC_INTERPOLATION_METHOD image_interpolation_method_ = ECC_INTER_LINEAR;
  int max_iterations_ = 30;

  double reference_smooth_sigma_ = 1.;
  double input_smooth_sigma_ = 1.;
  double update_step_scale_ = 1.0;

  double eps_ = 0.2;
  double min_rho_ = 0.75;
  double current_eps_ = DBL_MAX;
  double rho_ = -1;
  int num_iterations_  = 0;

  cv::Mat2f current_remap_;
  int number_of_parameters_ = 0;


  // convert image to float with optional gaussian smoothing
  static void prepare_input_image(cv::InputArray src, cv::InputArray src_mask,
      double smooth_sigma, bool force_create_mask,
      cv::Mat1f & dst, cv::Mat1b & dst_mask);


};


///////////////////////////////////////////////////////////////////////////////

// Forward additive ECC
// "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2",
//          Simon Baker et.all.
// "Parametric Image Alignment Using Enhanced Correlation Coefficient Maximization",
//          Georgios D. Evangelidis and Emmanouil Z. Psarakis
class c_ecc_forward_additive
    : public c_ecc_align
{
public:
  typedef c_ecc_forward_additive this_class;
  typedef std::shared_ptr<this_class> ptr;


  c_ecc_forward_additive(enum ECC_MOTION_TYPE ecc_motion_type = ECC_MOTION_AFFINE);

  bool align(cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray templateMask = cv::noArray()) override;


  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray inputImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray()) override;

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;
  const cv::Mat1f & input_image() const;
  const cv::Mat1b & input_mask() const;

protected:
  cv::Mat1f f;  // reference image
  cv::Mat1f g;  // input image
  cv::Mat1b fmask, gmask, wmask, iwmask; // image masks
  cv::Mat1f gw; // warped input image
  cv::Mat1f gx, gy, gxw, gyw; // input image derivatives
  cv::Mat1f jac;// jacobian [f.rows * numberOfParameters][f.cols]
  cv::Mat1f Hinv; // inverse of Hessian matrix
  cv::Mat1f p; // warping parameters matrix (the size and meaning depends on motion model)
  cv::Mat1f dp;// transformation parameter correction [numberOfParameters][1]
  cv::Mat1f e, ep;// error image and it's projection
};


///////////////////////////////////////////////////////////////////////////////

// Inverse compositional ECC
// "Lucas-Kanade 20 Years On: A Unifying Framework: Part 2", Simon Baker et.all.
// Formula (5) from the paper
//  "Fast, Robust and Accurate Digital Image Correlation Calculation Without Redundant Computations" B. Pan, K. Li, W. Tong
//  <http://www.ncorr.com/download/publications/panfast.pdf>
class c_ecc_inverse_compositional
    : public c_ecc_align
{
public:
  typedef c_ecc_inverse_compositional this_class;
  typedef std::shared_ptr<this_class> ptr;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) override;

  bool align(cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray()) override;

  bool align_to_reference(cv::InputArray inputImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray()) override;

  const cv::Mat1f & reference_image() const;
  const cv::Mat1f & input_image() const;

protected: // Notations are from the paper of  B. Pan, K. Li and W. Tong
  cv::Mat1f f; // reference image
  cv::Mat1f g; // input image
  cv::Mat1b fmask, gmask, wmask, iwmask; // image masks
  //cv::Mat1f fzm; // zero-mean normalized reference image
  cv::Mat1f gw; // warped input image
  cv::Mat1f e, ep; // error image and it's projection
  cv::Mat1f p, dp; // warping parameters matrix (the size and meaning depends on motion model)
  cv::Mat1f fx, fy; // gradient of reference image
  cv::Mat1f jac_; // steepest descend images ("jacobian")
  cv::Mat1f Hinv; // inverse of Hessian matrix
};


///////////////////////////////////////////////////////////////////////////////

// Coarse-To-Fine registration using image pyramids
class c_ecch
{
public:
  typedef c_ecch this_class;

  c_ecch(c_ecc_align * method = nullptr) ;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask);

  bool align_to_reference(cv::InputArray inputImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray());

  bool align(cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray());

  bool align(c_ecc_align * method,
      cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray());

  void set_minimum_image_size(int v);
  int minimum_image_size() const;

  c_ecc_align * method() const;
  const std::vector<cv::Mat> & image_pyramid(int index) const;
  const std::vector<cv::Mat> & mask_pyramid(int index) const;
  const std::vector<cv::Mat1f> & transform_pyramid() const;

protected:
  enum {
    current_image_index = 0,
    reference_image_index = 1,
  };

  c_ecc_align * method_;
  std::vector<cv::Mat> image_pyramids_[2];
  std::vector<cv::Mat> mask_pyramids_[2];
  std::vector<cv::Mat1f> transform_pyramid_;
  int minimum_image_size_ = 12;
};

/////////////////////////////////////////////////////////////////////////////////

// Smooth optical flow on image pyramids
class c_ecch_flow
{
public:

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
  // for example on strong vigneting or planetary disk derotation
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

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;

  const cv::Mat2f & current_uv() const;

  // public access for debug purposes
  const std::vector<pyramid_entry> & current_pyramid() const {
    return pyramid_;
  }

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


bool ecc_flow2remap(cv::InputArray uv,
    cv::OutputArray rmap);

bool ecc_remap2flow(cv::InputArray rmap,
    cv::OutputArray uv);

/////////////////////////////////////////////////////////////////////////////////
#endif /* __ecc_align_h__ */

