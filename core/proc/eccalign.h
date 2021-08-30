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

enum ECC_MOTION_TYPE {
    ECC_MOTION_NONE = -1,
    ECC_MOTION_TRANSLATION = cv::MOTION_TRANSLATION,
    ECC_MOTION_EUCLIDEAN   = cv::MOTION_EUCLIDEAN,
    ECC_MOTION_AFFINE      = cv::MOTION_AFFINE,
    ECC_MOTION_HOMOGRAPHY  = cv::MOTION_HOMOGRAPHY,
    ECC_MOTION_QUADRATIC   = ECC_MOTION_HOMOGRAPHY + 1,
};


const extern struct ecc_motion_type_desc {
  const char * name;
  enum ECC_MOTION_TYPE value;
} ecc_motion_types[];

std::string toStdString(enum ECC_MOTION_TYPE m);
enum ECC_MOTION_TYPE fromStdString(const std::string  & s,
    enum ECC_MOTION_TYPE defval );


const extern struct ecc_interpolation_flags_desc {
  const char * name;
  enum cv::InterpolationFlags value;
} ecc_interpolation_flags[];

std::string toStdString(enum cv::InterpolationFlags m);
enum cv::InterpolationFlags fromStdString(const std::string  & s,
    enum cv::InterpolationFlags defval );

//
// args:
//  motionType - enum ECC_MOTION_*
//  T - input transformation  matrix
//  rmap - output cv::Mat2f or cv::Mat2d matrix for cv::remap()
//  ddepth - destination type of rmap output matrix
bool createRemap(int motionType,
    cv::InputArray T,
    cv::OutputArray rmap,
    const cv::Size & size,
    int ddepth = -1 );

void createIdentityRemap(cv::OutputArray rmap,
    const cv::Size & size,
    int ddepth = -1 );

// Initialize identity (eye) transform matrix of appropriate size for given motion type
cv::Mat1f createEyeTransform(
    int ecc_motion_type);

// Initialize translation transform matrix of appropriate size for given motion type
cv::Mat1f createTranslationTransform(
    double Tx, double Ty);

/*
*
*  x' =  scale * ( cos(a) * (x-Cx) + sin(a) * (y-Cy)) + Tx
*  y' =  scale * (-sin(a) * (x-Cx) + cos(a) * (y-Cy)) + Ty
*
*  For simple rotation arount of a point C call
*   createEuclideanTransform(C.x, C.y, C.x, C.y, scale, angle);
*
*/
cv::Mat1f createEuclideanTransform(double Cx, double Cy,
    double Tx, double Ty,
    double scale,
    double angle);


// Scale given transform matrix to reflect image scale change
void scaleTransform(int ecc_motion_type,
    cv::Mat1f & T,
    double scale);

// Scale given transform matrix to reflect image scale change
void scaleTransform(int ecc_motion_type,
    const cv::Mat1f & src,
    cv::Mat1f & dst,
    double scale);

// Expand given 2x3 affine transformation matrix T to given motion type (homography or quadratic),
// padding with zeros and/or ones as appropriate
cv::Mat1f expandAffineTransform(const cv::Mat1f T,
    int ecc_motion_type);

// Extract translation component from current transform
bool getTranslationComponent(int ecc_motion_type, const cv::Mat1f & T,
    double * tx, double * ty);


// pyramide utils
bool pdownscale(cv::InputArray src, cv::Mat & dst, int level, int border_mode = cv::BORDER_REPLICATE);
bool pupscale(cv::Mat & image, cv::Size dstSize);
bool pnormalize(cv::Mat1f & image, cv::InputArray _mask, int level, double regularization_term);
void ecc_differentiate(const cv::Mat & src, cv::Mat & gx, cv::Mat & gy);
///////////////////////////////////////////////////////////////////////////////


// Base interface
class c_ecc_align {
public:
  typedef c_ecc_align this_class;
  typedef std::shared_ptr<this_class> ptr;

  void set_motion_type(int ecc_motion_type);
  int motion_type() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_eps(double v);
  double eps() const;

  void set_min_rho(double v);
  double min_rho() const;

  void set_image_interpolation_flags(int v);
  int image_interpolation_flags() const;

  void set_input_smooth_sigma(double v);
  double input_smooth_sigma() const;

  void set_reference_smooth_sigma(double v);
  double reference_smooth_sigma() const;

  void set_update_step_scale(double v);
  double update_step_scale() const;

  //  void set_pyramid_normalization_level(int v);
  //  int pyramid_normalization_level() const;

  //  void set_pyramid_normalization_regularization_term(double v);
  //  double pyramid_normalization_regularization_term() const;

  virtual bool align(cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray()) = 0;

  virtual bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) = 0;
  virtual bool align_to_reference(cv::InputArray inputImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray()) = 0;

  double rho() const;
  double current_eps() const;
  int num_iterations() const;
  const cv::Mat2f & current_remap() const;

  virtual ~c_ecc_align();

protected:
  int ecc_motion_type_ = ECC_MOTION_AFFINE;
  int max_iterations_ = 30;
  int image_interpolation_flags_ = cv::INTER_LINEAR;
  //  int pyramid_normalization_level_ = 0;
  //  double pyramid_normalization_regularization_term_ = 1.;

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

  bool align(cv::InputArray inputImage, cv::InputArray referenceImage, cv::InputOutputArray warpMatrix,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray templateMask = cv::noArray()) override;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray()) override;
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

// Coarse-To-Fine registration using umage pyramids
class c_ecc_pyramide_align
{
public:
  typedef c_ecc_pyramide_align this_class;

  c_ecc_pyramide_align(c_ecc_align * method = nullptr) ;

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
  c_ecc_align * method_;
  std::vector<cv::Mat> image_pyramids_[2];
  std::vector<cv::Mat> mask_pyramids_[2];
  std::vector<cv::Mat1f> transform_pyramid_;
  int minimum_image_size_ = 12;
};

/////////////////////////////////////////////////////////////////////////////////

//// Smooth optical flow
//class c_ecc_flow
//{
//public:
//  void set_pyramid_level(int v);
//  int pyramid_level() const;
//
//  void set_max_iterations(int v);
//  int max_iterations() const;
//
//  void set_update_multiplier(double v);
//  double update_multiplier() const;
//
//  bool set_reference_image(cv::InputArray referenceImage,
//      cv::InputArray referenceMask = cv::noArray());
//
//  bool compute(cv::InputArray inputImage, cv::InputArray referenceImage, cv::Mat2f & rmap,
//      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray());
//
//  bool compute(cv::InputArray inputImage, cv::Mat2f & rmap,
//      cv::InputArray inputMask = cv::noArray());
//
//  const cv::Mat1f & reference_image() const;
//  const cv::Mat1b & reference_mask() const;
//
//  const cv::Mat1f & input_image() const;
//  const cv::Mat1b & input_mask() const;
//
//  const cv::Mat2f & current_uv() const;
//  const cv::Mat1f & current_It() const;
//  const cv::Mat1f & current_worker_image() const;
//
//  static void flow2remap(const cv::Mat2f & uv,
//      cv::Mat2f & rmap);
//
//  static void remap2flow(const cv::Mat2f & rmap,
//      cv::Mat2f & uv);
//
//protected:
//  bool convert_input_images(cv::InputArray src, cv::InputArray src_mask,
//      cv::Mat1f & dst, cv::Mat1b & dst_mask) const;
//
//  void pnormalize(const cv::Mat1f & src, cv::Mat1f & dst) const;
//
//  bool compute_uv(const cv::Mat1f & I1, const cv::Mat1f & I2, const cv::Mat1b & mask,
//      cv::Mat2f & outuv);
//
//  bool pscale(cv::InputArray src, cv::Mat & dst, bool ismask = false) const;
//
//  double update_multiplier_ = 1.5;
//  double reference_noise_ = 0;
//  int pyramid_level_ = 5;
//  int max_iterations_ = 1;
//
//  cv::Mat1f reference_image_;
//  cv::Mat1b reference_mask_;
//  cv::Mat1f current_image_;
//  cv::Mat1b current_mask_;
//  cv::Mat1f current_It_;
//  cv::Mat1f worker_image_;
//  cv::Mat2f uv_;
//
//  cv::Mat1f Ix, Iy;
//  cv::Mat4f D;
//};
//

// Smooth optical flow on image pyramids
class c_ecch_flow
{
public:
  void set_support_scale(int v);
  int support_scale() const;

  void set_max_iterations(int v);
  int max_iterations() const;

  void set_update_multiplier(double v);
  double update_multiplier() const;

  // Use only for images which violate brightness constancy assumption,
  // for example on strong vigneting or planetary disk derotation
  void set_normalization_scale(int v);
  int normalization_scale() const;

  void set_input_smooth_sigma(double v);
  double input_smooth_sigma() const;

  void set_reference_smooth_sigma(double v);
  double reference_smooth_sigma() const;

  bool set_reference_image(cv::InputArray referenceImage,
      cv::InputArray referenceMask = cv::noArray());

  bool compute(cv::InputArray inputImage, cv::InputArray referenceImage, cv::Mat2f & rmap,
      cv::InputArray inputMask = cv::noArray(), cv::InputArray referenceMask = cv::noArray());

  bool compute(cv::InputArray inputImage, cv::Mat2f & rmap,
      cv::InputArray inputMask = cv::noArray());

  const cv::Mat1f & reference_image() const;
  const cv::Mat1b & reference_mask() const;
  //
  //  const cv::Mat1f & input_image() const;
  //  const cv::Mat1b & input_mask() const;
  //
  const cv::Mat2f & current_uv() const;
  //  const cv::Mat1f & current_It() const;
  //  const cv::Mat1f & current_worker_image() const;
  //
  //  const std::vector<cv::Mat1f> & mu1() const;
  //  const std::vector<cv::Mat1f> & mu2() const;

  static void flow2remap(const cv::Mat2f & uv,
      cv::Mat2f & rmap);

  static void remap2flow(const cv::Mat2f & rmap,
      cv::Mat2f & uv);

protected:
  struct pyramid_entry {
    cv::Mat1f current_image, reference_image;
    cv::Mat1b current_mask, reference_mask;
    cv::Mat2f rmap;
    cv::Mat1f Ix, Iy;//, It;
    cv::Mat4f D;
  };

  bool convert_input_images(cv::InputArray src, cv::InputArray srcmask,
      cv::Mat1f & dst, cv::Mat1b & dst_mask) const;

  bool compute_uv(pyramid_entry & e,
      cv::Mat2f & outuv) const;

  bool pscale(cv::InputArray src, cv::Mat & dst,
      bool ismask = false) const;

  void pnormalize(cv::InputArray src, cv::OutputArray dst,
      double noise_level) const;

  double input_smooth_sigma_ = 0;
  double reference_smooth_sigma_ = 0;

  double update_multiplier_ = 1.5;
  int max_iterations_ = 1;
  int support_scale_ = 5;
  int normalization_scale_ = 0;

  std::vector<pyramid_entry> pyramid_;
  cv::Mat2f cuv;
};


/////////////////////////////////////////////////////////////////////////////////
#endif /* __ecc_align_h__ */

