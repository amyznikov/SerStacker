/*
 * ecc2.cc
 *
 *  Created on: Feb 9, 2023
 *      Author: amyznikov
 */
#include "ecc2.h"
#include <tbb/tbb.h>
#include <core/proc/downstrike.h>
#include <core/io/save_image.h>
#include <core/ssprintf.h>
#include <core/debug.h>

namespace {

typedef tbb::blocked_range<int> tbb_range;
constexpr int tbb_block_size = 512;


template<class T>
inline T square(T x)
{
  return x * x;
}

/**
 * Five-point approximation to first order image derivative.
 *  <https://en.wikipedia.org/wiki/Numerical_differentiation>
 * */
void ecc_differentiate(cv::InputArray _src, cv::Mat & gx, cv::Mat & gy, int ddepth = CV_32F)
{
  static thread_local const cv::Matx<float, 1, 5> K(
      (+1.f / 12),
      (-8.f / 12),
      0.f,
      (+8.f / 12),
      (-1.f / 12));

  if( ddepth < 0 ) {
    ddepth = std::max(_src.depth(), CV_32F);
  }

  const cv::Mat &src = _src.getMat();
  cv::filter2D(src, gx, ddepth, K, cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
  cv::filter2D(src, gy, ddepth, K.t(), cv::Point(-1, -1), 0, cv::BORDER_REPLICATE);
}

void compute_hessian_matrix(const cv::Mat1f & jac, cv::Mat1f & H, int nparams)
{
  // Hessian matrix requested

  if( jac.data == H.data ) {
    CF_FATAL("APP BUG: SRC and DST must be different objects, inplace not supported");
    exit(1);
  }

  const int h =
      jac.rows / nparams;

  H.create(nparams, nparams);

  for( int i = 0; i < nparams; i++ ) {

    const cv::Mat m(jac.rowRange(i * h, (i + 1) * h));

    const float nn = cv::norm(m);

    H[i][i] = nn * nn;  // diagonal elements

    for( int j = i + 1; j < nparams; j++ ) {
      H[i][j] = m.dot(jac.rowRange(j * h, (j + 1) * h));
      H[j][i] = H[i][j];
    }
  }
}

void project_error_image(const cv::Mat1f & jac, const cv::Mat & error_image, cv::Mat1f & dst, int nparams)
{
  // Projection to jacobian is requested

  if( error_image.data == dst.data ) {
    CF_FATAL("APP BUG: SRC and DST must be different objects, inplace not supported");
    exit(1);
  }

  const int h =
      jac.rows / nparams;

  dst.create(nparams, 1);

  for( int i = 0; i < nparams; ++i ) {
    dst[i][0] = error_image.dot(jac.rowRange(i * h, (i + 1) * h));
  }
}

} // namespace

/////////////////////////////////////////

c_ecc2::c_ecc2( c_ecc_transform * transfrom) :
    transfrom_(transfrom)
{
}

void c_ecc2::set_transform(c_ecc_transform * transfrom)
{
  transfrom_ = transfrom;
}

c_ecc_transform * c_ecc2::transfrom() const
{
  return transfrom_;
}


void c_ecc2::set_max_iterations(int v)
{
  this->max_iterations_ = v;
}

int c_ecc2::max_iterations() const
{
  return this->max_iterations_;
}

void c_ecc2::set_eps(double v)
{
  this->eps_ = v;
}

double c_ecc2::eps() const
{
  return this->eps_;
}

void c_ecc2::set_min_rho(double v)
{
  min_rho_ = v;
}

double c_ecc2::min_rho() const
{
  return min_rho_;
}


void c_ecc2::set_interpolation(enum ECC2_INTERPOLATION_METHOD  v)
{
  interpolation_ = v;
}

enum ECC2_INTERPOLATION_METHOD c_ecc2::interpolation() const
{
  return interpolation_;
}

void c_ecc2::set_input_smooth_sigma(double v)
{
  input_smooth_sigma_ = v;
}

double c_ecc2::input_smooth_sigma() const
{
  return input_smooth_sigma_;
}

void c_ecc2::set_reference_smooth_sigma(double v)
{
  reference_smooth_sigma_ = v;
}

double c_ecc2::reference_smooth_sigma() const
{
  return reference_smooth_sigma_;
}

void c_ecc2::set_update_step_scale(double v)
{
  update_step_scale_ = v;
}

double c_ecc2::update_step_scale() const
{
  return update_step_scale_;
}

double c_ecc2::rho() const
{
  return this->rho_;
}

int c_ecc2::num_iterations() const
{
  return this->num_iterations_;
}

double c_ecc2::current_eps() const
{
  return current_eps_;
}

const cv::Mat2f & c_ecc2::current_remap() const
{
  return current_remap_;
}


// convert image to 32-bit float with optional gaussian smoothing
void c_ecc2::prepare_input_image(cv::InputArray src, cv::InputArray src_mask,
    double smooth_sigma, bool force_create_mask,
    cv::Mat1f & dst, cv::Mat1b & dst_mask) const
{
  if ( smooth_sigma <= 0 ) {
    src.getMat().convertTo(dst, dst.depth());
  }
  else {

    const cv::Mat1f G =
        cv::getGaussianKernel(2 * ((int) (4 * smooth_sigma)) + 1,
            smooth_sigma);

    if ( src_mask.empty() ) {

      cv::sepFilter2D(src, dst, dst.depth(),
          G, G,
          cv::Point(-1, -1),
          0,
          cv::BORDER_REPLICATE);

    }
    else {

      const cv::Mat1b mask =
          src_mask.getMat();

      cv::Mat1f fmask;

      mask.convertTo(fmask, fmask.type(), 1. / 255);

      cv::sepFilter2D(src, dst, dst.depth(),
          G, G,
          cv::Point(-1, -1),
          0,
          cv::BORDER_REPLICATE);

      cv::sepFilter2D(fmask, fmask, fmask.depth(),
          G, G,
          cv::Point(-1, -1),
          +1e-12,
          cv::BORDER_REPLICATE);

      cv::divide(dst, fmask, dst);
    }
  }


  if ( !src_mask.empty() ) {

    if ( cv::countNonZero(src_mask) == src_mask.size().area() ) {
      src_mask.copyTo(dst_mask);
    }
    else { // may need to protect some border near mask edges because of differentiation

      cv::erode(src_mask, dst_mask, cv::Mat1b(5, 5, 255),
          cv::Point(-1, -1), 1,
          cv::BORDER_REPLICATE);

    }
  }
  else if ( force_create_mask ) {
    dst_mask.create(src.size());
    dst_mask.setTo(255);
  }
  else {
    // ensure it is empty
    dst_mask.release();
  }
}




/////////////////////////////////////////


c_ecc2_forward_additive::c_ecc2_forward_additive(c_ecc_transform * transfrom) :
    base(transfrom)
{
}

const cv::Mat1f & c_ecc2_forward_additive::input_image() const
{
  return g;
}

const cv::Mat1f & c_ecc2_forward_additive::reference_image() const
{
  return f;
}

const cv::Mat1b & c_ecc2_forward_additive::input_mask() const
{
  return gmask;
}

const cv::Mat1b & c_ecc2_forward_additive::reference_mask() const
{
  return fmask;
}

bool c_ecc2_forward_additive::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if ( referenceImage.channels() != 1 ) {
    CF_ERROR("reference image must be single-channel (grayscale)");
    return false;
  }

  if ( !referenceMask.empty() ) {
    if ( referenceMask.type() != CV_8UC1 || referenceMask.size() != referenceImage.size() ) {
      CF_ERROR("reference mask must CV_8UC1 type of the same size as input image");
      return false;
    }
  }

  // convert reference image to float
  prepare_input_image(referenceImage, referenceMask,
      reference_smooth_sigma_, false, f, fmask);

  return true;
}

bool c_ecc2_forward_additive::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  num_iterations_ = -1, rho_ = -1;
  if ( !set_reference_image(referenceImage, referenceMask) ) {
    CF_FATAL("c_ecc_forward_additive: set_reference_image() fails");
    return false;
  }
  return align_to_reference(inputImage, inputMask);
}

bool c_ecc2_forward_additive::align_to_reference(cv::InputArray inputImage, cv::InputArray inputMask)
{
  if( !transfrom_ ) {
    CF_ERROR("Pointer to image transformation interface was not set");
    return false;
  }

  if( inputImage.channels() != 1 ) {
    CF_ERROR("input image must be single-channel (grayscale)");
    return false;
  }

  if( !inputMask.empty() ) {
    if( inputMask.type() != CV_8UC1 || inputMask.size() != inputImage.size() ) {
      CF_ERROR("input mask must CV_8UC1 type of the same size as input image");
      return false;
    }
  }

  num_iterations_ = -1;
  rho_ = -1;
  if( eps_ <= 0 ) {
    eps_ = 1e-3;
  }

  if( (number_of_parameters_ = transfrom_->num_adustable_parameters()) < 1 ) {
    CF_FATAL("transfrom_->get_num_params() return %d", number_of_parameters_);
    return false;
  }

  //
  // Precompute
  //

  // convert input image (to be aligned) to float and create the mask
  prepare_input_image(inputImage, inputMask, input_smooth_sigma_, true, g, gmask);

  // Evaluate the gradient ∇G of the input image G(x)
  ecc_differentiate(g, gx, gy);
  if( !inputMask.empty() ) {
    cv::bitwise_not(gmask, iwmask);
    gx.setTo(0, iwmask);
    gy.setTo(0, iwmask);
  }

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio;
  bool failed = false;

  for( num_iterations_ = 0; num_iterations_ <= max_iterations_; ++num_iterations_ ) {

    // Warp g, gx and gy with W(x; p) to compute warped input image g(W(x; p)) and it's gradients
    if ( !transfrom_->create_remap(current_remap_, f.size()) ) {
      CF_ERROR("transfrom_->create_remap() fails");
      failed = true;
      break;
    }
    cv::remap(g, gw, current_remap_, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
    cv::remap(gx, gxw, current_remap_, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
    cv::remap(gy, gyw, current_remap_, cv::noArray(), interpolation_, cv::BORDER_REPLICATE);
    cv::remap(gmask, wmask, current_remap_, cv::noArray(), cv::INTER_AREA, cv::BORDER_CONSTANT, 0);

    if( !fmask.empty() ) {
      bitwise_and(wmask, fmask, wmask);
    }

    cv::bitwise_not(wmask, iwmask);

    gxw.setTo(0, iwmask);
    gyw.setTo(0, iwmask);

    // compute stdev ratio stdev(g)/stdev(f) and mean values
    cv::meanStdDev(f, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = gStd[0] / fStd[0];

    // create steepest descent images
    transfrom_->create_steepest_descent_images(gxw, gyw, jac);

    // calculate Hessian and its inverse
    compute_hessian_matrix(jac, H, number_of_parameters_);

    if ( !cv::invert(H, Hinv, cv::DECOMP_CHOLESKY) ) {
      CF_ERROR("[iteration %d] cv::invert(Hessian, cv::DECOMP_CHOLESKY) fails", num_iterations_);

      CF_DEBUG("cv::countNonZero(jac)=%d", cv::countNonZero(jac));
      for ( int r = 0, nr = Hinv.rows; r < nr; ++r ) {
        for ( int c = 0, nc = Hinv.cols; c < nc; ++c ) {
          CF_DEBUG("H(%3d, %3d) = %+g", r, c, H(r, c));
        }
      }

      failed = true;
      break;
    }

    // compute update parameters
    // e = -(gwzm - stdev_ratio * fzm);
    cv::scaleAdd(f, -stdev_ratio, gw, e);
    cv::subtract(e, gMean - stdev_ratio * fMean, e);
    e.setTo(0, iwmask);

    // compute projected error
    project_error_image(jac, e, ep, number_of_parameters_);

    // compute update parameters
    dp = -update_step_scale_ * (Hinv * ep);

//    CF_DEBUG("--------------------");
//
//    CF_DEBUG("ep: %dx%d", ep.rows, ep.cols);
//    for ( int r = 0, nr = ep.rows; r < nr; ++r ) {
//      for ( int c = 0, nc = ep.cols; c < nc; ++c ) {
//        CF_DEBUG("ep(%3d, %3d) = %+g", r, c, ep(r, c));
//      }
//    }
//
//    CF_DEBUG("H: %dx%d", ep.rows, ep.cols);
//    for ( int r = 0, nr = H.rows; r < nr; ++r ) {
//      for ( int c = 0, nc = H.cols; c < nc; ++c ) {
//        CF_DEBUG("H(%3d, %3d) = %+g", r, c, H(r, c));
//      }
//    }
//    CF_DEBUG("--------------------");

    // update warping matrix
    if( !transfrom_->update_forward_additive(dp, &current_eps_, f.size()) ) {
      CF_ERROR("transfrom_->update_forward_additive() fails");
      failed = true;
    }

    if( current_eps_ < eps_ || num_iterations_ == max_iterations_ || failed ) {

      rho_ = cv::computeECC(f, gw, wmask);

      if ( isnan(rho_) ) {
        CF_ERROR("[iteration %d] cv::computeECC() returns NaN", num_iterations_);
      }

      cv::subtract(f, fMean, e), e.setTo(0, iwmask);
      cv::subtract(gw, gMean, gw), gw.setTo(0, iwmask);

      rho_ = e.dot(gw) / (cv::countNonZero(wmask) * fStd[0] * gStd[0]);

      if ( isnan(rho_) ) {
        CF_ERROR("[iteration %d] e.dot(gw) returns NaN", num_iterations_);
      }


      break;
    }

  }

  return !failed && rho_ > 0;
}


///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc2_inverse_compositional::c_ecc2_inverse_compositional(c_ecc_transform * transfrom) :
    base(transfrom)
{
}

const cv::Mat1f & c_ecc2_inverse_compositional::input_image() const
{
  return g;
}

const cv::Mat1f & c_ecc2_inverse_compositional::reference_image() const
{
  return f;
}

bool c_ecc2_inverse_compositional::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  // Convert reference image to floating point,
  // Pre-Evaluate the gradient ∇T of reference image T(x),
  // Pre-Compute Jacobian ∂W/∂p at (x,0) and steepest descent images ▽f*∂W/∂p

  prepare_input_image(referenceImage, referenceMask, reference_smooth_sigma_, false, f, fmask);

  ecc_differentiate(f, fx, fy);

  if( !transfrom_->create_steepest_descent_images(fx, fy, jac_) ) {
    CF_ERROR("transfrom_->compute_jacobian() fails");
    return false;
  }

  return true;
}

bool c_ecc2_inverse_compositional::align(cv::InputArray inputImage, cv::InputArray referenceImage,
    cv::InputArray inputMask, cv::InputArray referenceMask)
{
  num_iterations_ = -1, rho_ = -1;
  if ( !set_reference_image(referenceImage, referenceMask) ) {
    CF_FATAL("c_ecc_inverse_compositional: set_reference_image() fails");
    return false;
  }
  return align_to_reference(inputImage, inputMask);
}

bool c_ecc2_inverse_compositional::align_to_reference(cv::InputArray inputImage, cv::InputArray inputMask)
{
  cv::Mat1f jac;

  num_iterations_ = -1;
  rho_ = -1;
  if ( eps_ <= 0 ) {
    eps_ = 1e-3;
  }


  if ( (number_of_parameters_ = transfrom_->num_adustable_parameters()) < 1 ) {
    CF_ERROR("c_ecc2_inverse_compositional: transfrom_->get_num_params() returns %d",
        number_of_parameters_);
    return false;
  }

  //inputOutputWarpMatrix.getMat().convertTo(p, p.type());

  // Convert input image (to be aligned) to floating point and create the mask
  prepare_input_image(inputImage, inputMask, input_smooth_sigma_, true, g, gmask);

  // Assume the initial warp matrix p is good enough to avoid the hessian matrix evaluation on each iteration
  if ( !transfrom_->create_remap(current_remap_, f.size()) ) {
    CF_ERROR("c_ecc2_inverse_compositional: transfrom_->create_remap() fails");
    return false;
  }

  cv::remap(gmask, wmask, current_remap_, cv::noArray(),
      cv::INTER_AREA,
      cv::BORDER_CONSTANT,
      cv::Scalar(0));

  if ( !fmask.empty() ) {
    bitwise_and(wmask, fmask, wmask);
  }

  wmask.copyTo(fmask);
  cv::bitwise_not(wmask, iwmask);

  const double nnz0 =
      cv::countNonZero(wmask);

  jac_.copyTo(jac);

  for ( int i = 0; i < number_of_parameters_; ++i ) {
    jac.rowRange(i * f.rows, (i + 1) * f.rows).setTo(0, iwmask);
  }


  // Compute the Hessian matrix H = ∑x[▽f*∂W/∂p]^T*[▽f*∂W/∂p] and it's inverse
  //project_onto_jacobian(jac, jac, H, number_of_parameters_);
  compute_hessian_matrix(jac, H, number_of_parameters_);
  cv::invert(H, H, cv::DECOMP_CHOLESKY);
  H *= -update_step_scale_;

  //
  // Iterate
  //

  cv::Scalar gMean, gStd, fMean, fStd;
  double stdev_ratio, nnz1;
  bool failed = false;

  for ( num_iterations_ = 0; num_iterations_ <= max_iterations_; ++num_iterations_ ) {

    // Warp g with W(x; p) to compute g(W(x; p))

    if ( num_iterations_ < 1 ) {
      nnz1 = nnz0;
    }
    else {

      if ( !transfrom_->create_remap(current_remap_, f.size()) ) {
        CF_ERROR("c_ecc2_inverse_compositional: transfrom_->create_remap() fails");
        failed = true;
        break;
      }

      cv::remap(gmask, wmask, current_remap_, cv::noArray(),
          cv::INTER_AREA,
          cv::BORDER_CONSTANT,
          cv::Scalar(0));

      cv::bitwise_not(wmask, iwmask);

      nnz1 = cv::countNonZero(wmask);
    }

    // The cv::BORDER_REPLICATE is to avoid some annoying edge effects caused by interpolation
    cv::remap(g, gw, current_remap_, cv::noArray(),
        interpolation_,
        cv::BORDER_REPLICATE);

    //
    // compute stdev ratio stdev(f)/stdev(gw) and mean values
    //
    cv::meanStdDev(f, fMean, fStd, wmask);
    cv::meanStdDev(gw, gMean, gStd, wmask);
    stdev_ratio = fStd[0] / gStd[0];

    //
    // compute the error image e = fzm - stdev_ratio * gwzm
    //
    cv::scaleAdd(gw, -stdev_ratio, f, e);  // e now contains the error image
    cv::subtract(e, fMean - stdev_ratio * gMean, e), e.setTo(0, iwmask);

    //
    // compute update parameters
    //
    project_error_image(jac, e, ep, number_of_parameters_);  // ep now contains projected error
    dp = (H * ep) * square(nnz0 / nnz1);

    //
    // update warping parameters
    //
    if ( !transfrom_->update_inverse_composite(dp, &current_eps_, f.size()) ) {
      CF_ERROR("transfrom_->update_inverse_composite() fails");
      failed = true;
      break;
    }

    // check eps
    if ( current_eps_ < eps_ || num_iterations_ == max_iterations_ || failed ) {
      cv::subtract(f, fMean, e), e.setTo(0, iwmask);
      cv::subtract(gw, gMean, gw), gw.setTo(0, iwmask);
      rho_ = e.dot(gw) / (nnz1 * fStd[0] * gStd[0]);
      break;
    }
  }

  return !failed && rho_ > min_rho_;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecch2::c_ecch2(c_ecc2 * method) :
    method_(method)
{
}

void c_ecch2::set_method(c_ecc2 * method)
{
  method_ = method;
}

c_ecc2 * c_ecch2::method() const
{
  return method_;
}

void c_ecch2::set_minimum_image_size(int v)
{
  minimum_image_size_ = v;
}

int c_ecch2::minimum_image_size() const
{
  return minimum_image_size_;
}

const std::vector<cv::Mat> & c_ecch2::image_pyramid(int index) const
{
  return image_pyramids_[index];
}

const std::vector<cv::Mat> & c_ecch2::mask_pyramid(int index) const
{
  return mask_pyramids_[index];
}

const std::vector<cv::Mat1f> & c_ecch2::transform_pyramid() const
{
  return transform_pyramid_;
}

bool c_ecch2::set_reference_image(cv::InputArray referenceImage, cv::InputArray referenceMask)
{
  if( referenceImage.channels() != 1 ) {
    CF_ERROR("c_ecch: Both input and reference images must be single-channel (grayscale)");
    return false;
  }

  // Build reference image pyramid
  constexpr int R = reference_image_index;

  transform_pyramid_.clear();
  transform_pyramid_.reserve(10);
  image_pyramids_[R].clear();
  image_pyramids_[R].reserve(10);
  mask_pyramids_[R].clear();
  mask_pyramids_[R].reserve(10);

  image_pyramids_[R].emplace_back(referenceImage.getMat());
  mask_pyramids_[R].emplace_back(referenceMask.getMat());

  // Build pyramid for reference image
  while (42) {

    const int currentMinSize =
        std::min(image_pyramids_[R].back().cols,
            image_pyramids_[R].back().rows);

    const int nextMinSize =
        currentMinSize / 2;

    if( nextMinSize < minimum_image_size_ ) {
      break;
    }

    image_pyramids_[R].emplace_back();

    cv::pyrDown(image_pyramids_[R][image_pyramids_[R].size() - 2],
        image_pyramids_[R].back());

    mask_pyramids_[R].emplace_back();

    cv::Mat & cmask =
        mask_pyramids_[R].back();

    cv::Mat & pmask =
        mask_pyramids_[R][mask_pyramids_[R].size() - 2];

    if( !pmask.empty() && cv::countNonZero(pmask) < pmask.size().area() ) {
      cv::pyrDown(pmask, cmask);
      cv::compare(cmask, 255, cmask, cv::CMP_GE);
    }
  }

  return true;
}

bool c_ecch2::align(cv::InputArray inputImage, cv::InputArray inputMask)
{
  c_ecc_transform *transform;

  constexpr int C = current_image_index;
  constexpr int R = reference_image_index;

  if( !method_ ) {
    CF_ERROR("c_ecch2: underlying align method not specified");
    return false;
  }

  if( !(transform = method_->transfrom()) ) {
    CF_ERROR("c_ecch2: image transform not specified");
    return false;
  }

  if( image_pyramids_[R].empty() ) {
    CF_ERROR("c_ecch: No reference image was set");
    return false;
  }

  if( inputImage.channels() != 1 ) {
    CF_ERROR("c_ecch2: Both input and reference images must be single-channel (grayscale)");
    return false;
  }

  cv::Mat1f T =
      transform->parameters();

  if( T.empty() ) {
    CF_ERROR("c_ecch2: method_->transfrom()->parameters() returns empty matrix");
    return false;
  }

  // Build current image and initial transform pyramid

  image_pyramids_[C].clear();
  image_pyramids_[C].reserve(10);
  mask_pyramids_[C].clear();
  mask_pyramids_[C].reserve(10);
  transform_pyramid_.clear();
  transform_pyramid_.reserve(10);

  image_pyramids_[C].emplace_back(inputImage.getMat());
  mask_pyramids_[C].emplace_back(inputMask.getMat());
  transform_pyramid_.emplace_back(T);

  while ( 42 ) {

    int I = image_pyramids_[C].size() - 1;

    const int currentMinSize = std::min(std::min(std::min(
        image_pyramids_[C][I].cols, image_pyramids_[C][I].rows),
        image_pyramids_[R][I].cols), image_pyramids_[R][I].rows);

    const int nextMinSize = currentMinSize / 2;

    if( nextMinSize < minimum_image_size_ ) {
      break;
    }

    image_pyramids_[C].emplace_back();
    cv::pyrDown(image_pyramids_[C][image_pyramids_[C].size() - 2],
        image_pyramids_[C].back());

    mask_pyramids_[C].emplace_back();

    cv::Mat & cmask =
        mask_pyramids_[C].back();

    cv::Mat & pmask =
        mask_pyramids_[C][mask_pyramids_[C].size() - 2];

    if( !pmask.empty() && cv::countNonZero(pmask) < pmask.size().area() ) {
      cv::pyrDown(pmask, cmask);
      cv::compare(cmask, 255, cmask, cv::CMP_GE);
    }

    transform_pyramid_.emplace_back(transform->scale(transform_pyramid_.back(), 0.5));
    if( transform_pyramid_.back().empty() ) {
      CF_ERROR("transform->scale() fails");
      return false;
    }
  }

  // Align pyramid images in coarse-to-fine direction
  bool eccOk = false;

  for( int i = transform_pyramid_.size() - 1; i >= 0; --i ) {

    if( !transform->set_parameters(transform_pyramid_[i]) ) {
      CF_ERROR("L[%d] transform->set_parameters() fails", i);
      return false;
    }

    bool fOk =
        method_->align(image_pyramids_[C][i],
            image_pyramids_[R][i],
            mask_pyramids_[C][i],
            mask_pyramids_[R][i]);

    if( !fOk ) {

      CF_DEBUG("L[%2d/%d] : align fails: size=%dx%d num_iterations=%d rho=%g eps=%g", i,
          transform_pyramid_.size(),
          image_pyramids_[C][i].cols, image_pyramids_[C][i].rows,
          method_->num_iterations(), method_->rho(), method_->current_eps());

      continue;
    }


    if( i == 0 ) {
      eccOk = true;
      break;
    }

    // i > 0
    if( (transform_pyramid_[i - 1] = transform->scale(transform->parameters(), 2)).empty() ) {
      CF_ERROR("L[%d] transform->scale() fails", i);
      return false;
    }
  }

  return eccOk;
}

//
//bool c_ecch2::align(cv::InputArray inputImage, cv::InputArray inputMask)
//{
//  c_ecc_transform *transform;
//
//  constexpr int C = current_image_index;
//  constexpr int R = reference_image_index;
//
//  if( !method_ ) {
//    CF_ERROR("c_ecch2: underlying align method not specified");
//    return false;
//  }
//
//  if( !(transform = method_->transfrom()) ) {
//    CF_ERROR("c_ecch2: image transform not specified");
//    return false;
//  }
//
//  if( image_pyramids_[R].empty() ) {
//    CF_ERROR("c_ecch: No reference image was set");
//    return false;
//  }
//
//  if( inputImage.channels() != 1 ) {
//    CF_ERROR("c_ecch2: Both input and reference images must be single-channel (grayscale)");
//    return false;
//  }
//
//  cv::Mat1f T =
//      transform->parameters();
//
//  if( T.empty() ) {
//    CF_ERROR("c_ecch2: method_->transfrom()->parameters() returns empty matrix");
//    return false;
//  }
//
//  // Build current image and initial transform pyramid
//
//  image_pyramids_[C].clear();
//  image_pyramids_[C].reserve(10);
//  mask_pyramids_[C].clear();
//  mask_pyramids_[C].reserve(10);
//  transform_pyramid_.clear();
//  transform_pyramid_.reserve(10);
//
//  image_pyramids_[C].emplace_back(inputImage.getMat());
//  mask_pyramids_[C].emplace_back(inputMask.getMat());
//  transform_pyramid_.emplace_back(T);
//
//  while ( 42 ) {
//
//    int I = image_pyramids_[C].size() - 1;
//
//    const int currentMinSize = std::min(std::min(std::min(
//        image_pyramids_[C][I].cols, image_pyramids_[C][I].rows),
//        image_pyramids_[R][I].cols), image_pyramids_[R][I].rows);
//
//    const int nextMinSize = currentMinSize / 2;
//
//    if( nextMinSize < minimum_image_size_ ) {
//      break;
//    }
//
//    image_pyramids_[C].emplace_back();
//
//    // FIXME: GaussianBlur with mask will produce edge effects
//    cv::GaussianBlur(image_pyramids_[C][image_pyramids_[C].size() - 2],
//        image_pyramids_[C].back(),
//        cv::Size(5, 5),
//        0, 0,
//        cv::BORDER_REPLICATE);
//
//    downstrike_uneven(image_pyramids_[C].back(),
//        image_pyramids_[C].back());
//
//    mask_pyramids_[C].emplace_back();
//    if( !mask_pyramids_[C][mask_pyramids_[C].size() - 2].empty() ) {
//      downstrike_uneven(mask_pyramids_[C][mask_pyramids_[C].size() - 2],
//          mask_pyramids_[C].back());
//    }
//
//    transform_pyramid_.emplace_back(transform->scale(transform_pyramid_.back(), 0.5));
//    if( transform_pyramid_.back().empty() ) {
//      CF_ERROR("transform->scale() fails");
//      return false;
//    }
//  }
//
//  // Align pyramid images in coarse-to-fine direction
//  bool eccOk = false;
//
//  std::string debug_path = "/mnt/data/ecc2/debug";
//
//  for( int i = transform_pyramid_.size() - 1; i >= 0; --i ) {
//
//    if( !transform->set_parameters(transform_pyramid_[i]) ) {
//      CF_ERROR("L[%d] transform->set_parameters() fails", i);
//      return false;
//    }
//
//    save_image(image_pyramids_[R][i], ssprintf("%s/r%03d.tiff", debug_path.c_str(), i));
//    save_image(image_pyramids_[C][i], ssprintf("%s/c%03d.tiff", debug_path.c_str(), i));
//
//    bool fOk =
//        method_->align(image_pyramids_[C][i],
//            image_pyramids_[R][i],
//            mask_pyramids_[C][i],
//            mask_pyramids_[R][i]);
//
//    if( !fOk ) {
//      CF_DEBUG("L[%2d] : align fails: size=%dx%d num_iterations=%d rho=%g eps=%g", i,
//          image_pyramids_[C][i].cols, image_pyramids_[C][i].rows,
//          method_->num_iterations(), method_->rho(), method_->current_eps());
//      continue;
//    }
//
//    else if( true ) {
//      CF_DEBUG("L[%2d] : eccAlign() OK: size=%dx%d num_iterations=%d rho=%g eps=%g", i,
//          image_pyramids_[C][i].cols, image_pyramids_[C][i].rows,
//          method_->num_iterations(), method_->rho(), method_->current_eps());
//    }
//
//    if( i == 0 ) {
//      eccOk = true;
//    }
//    else if( (transform_pyramid_[i - 1] = transform->scale(transform->parameters(), 2)).empty() ) {      // i > 0
//      CF_ERROR("L[%d] transform->scale() fails", i);
//      return false;
//    }
//  }
//
//  if( !eccOk ) {
//    return false;
//  }
//
//  return true;
//}


///////////////////////////////////////////////////////////////////////////////////////////////////

cv::Mat1f c_ecc_transform::scale(const cv::Mat1f & p, double factor) const
{
  return cv::Mat1f();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_translation_transform::c_ecc_translation_transform(float Tx, float Ty) :
    T_(Tx, Ty)
{
}

c_ecc_translation_transform::c_ecc_translation_transform(const cv::Vec2f & t) :
    T_(t)
{
}

void c_ecc_translation_transform::set_translation(const cv::Vec2f & v)
{
  T_  = v;
}

const cv::Vec2f & c_ecc_translation_transform::translation() const
{
  return T_;
}


cv::Mat1f c_ecc_translation_transform::parameters() const
{
  return cv::Mat1f(2, 1, (float*) T_.val).clone();
}

bool c_ecc_translation_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 2 && p.cols == 1 ) {
    T_[0] = p(0, 0);
    T_[1] = p(1, 0);
    return true;
  }

  if( p.rows == 1 && p.cols == 2 ) {
    T_[0] = p(0, 0);
    T_[1] = p(0, 1);
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1 or 1x2",
      p.rows, p.cols);

  return false;
}

cv::Mat1f c_ecc_translation_transform::scale(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 1 && p.cols == 2 ) {

    cv::Mat1f sp(1, 2);
    sp(0, 0) = p(0, 0) * factor;
    sp(0, 1) = p(0, 1) * factor;

    return sp;
  }

  if( p.rows == 2 && p.cols == 1 ) {
    cv::Mat1f sp(1, 2);
    sp(0, 0) = p(0, 0) * factor;
    sp(1, 0) = p(1, 0) * factor;
    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x1 or 1x2", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_ecc_translation_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

  const float tx = T_[0];
  const float ty = T_[1];

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [tx, ty, &map](const tbb_range & r) {
        for ( int y = r.begin(); y < r.end(); ++y ) {
          cv::Vec2f * m = map[y];
          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = x + tx;
            m[x][1] = y + ty;
          }
        }
      });

  return true;
}

int c_ecc_translation_transform::num_adustable_parameters() const
{
  return 2;
}

bool c_ecc_translation_transform::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  // SDI:
  //  [ gx * dWx/dp0 + gy * dWy/dp0]
  //  [ gx * dWx/dp1 + gy * dWy/dp1]
  //  ...
  //  [ gx * dWx/dpn + gy * dWy/dpn]


  // Wx([x,y], p) = x + a0
  // Wy([x,y], p) = y + a1

  // gx * dWx/da0 + gy * dWy/da0
  // gx * dWx/da1 + gy * dWy/da1


  const int w = gx.cols;
  const int h = gx.rows;

  dst.create(h * 2, w);

  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < w; ++x ) {
            dst[y + 0 * h][x] = gx[y][x];
            dst[y + 1 * h][x] = gy[y][x];
          }
        }
      });

  return true;
}

bool c_ecc_translation_transform::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & imageSize)
{
  const float dx = p(0, 0);
  const float dy = p(1, 0);

  T_[0] += dx;
  T_[1] += dy;

  if( e ) {
    *e = sqrt(square(dx) + square(dy));
  }
  return true;
}

bool c_ecc_translation_transform::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & imageSize)
{
  const float dx = p(0, 0);
  const float dy = p(1, 0);

  T_[0] -= dx;
  T_[1] -= dy;

  if( e ) {
    *e = sqrt(square(dx) + square(dy));
  }

  return true;
}
//
/////////////////////////////////////////////////////////////////////////////////////////////////////
///**
// * Image Euclidean transform:
// *  [Tx_, Ty_, angle_]
// *
// *  x' =  cos(angle) * x - sin(angle) * y + Tx
// *  y' =  sin(angle) * x + cos(angle) * y + Ty
// *                 =>
// *  x' =  a00 * x + a01 * y + a02
// *  y' =  a10 * x + a11 * y + a12
// *
// */
//c_ecc_euclidean_transform::c_ecc_euclidean_transform(float Tx, float Ty, float angle) :
//    T_(Tx, Ty),
//    angle_(angle)
//{
//}
//
//c_ecc_euclidean_transform::c_ecc_euclidean_transform(const cv::Vec2f & T, float angle) :
//    T_(T),
//    angle_(angle)
//{
//
//}
//
//void c_ecc_euclidean_transform::set_translation(const cv::Vec2f & v)
//{
//  T_ = v;
//}
//
//cv::Vec2f c_ecc_euclidean_transform::translation() const
//{
//  return T_;
//}
//
//void c_ecc_euclidean_transform::set_rotation(float v)
//{
//  angle_ = v;
//}
//
//float c_ecc_euclidean_transform::rotation() const
//{
//  return angle_ ;
//}
//
//int c_ecc_euclidean_transform::num_parameters() const
//{
//  return 3;
//}
//
//cv::Mat1f c_ecc_euclidean_transform::parameters() const
//{
//  cv::Mat1f p(3, 1);
//  p(0, 0) = T_[0];
//  p(1, 0) = T_[1];
//  p(2, 0) = angle_;
//  return p;
//}
//
//bool c_ecc_euclidean_transform::set_parameters(const cv::Mat1f & p)
//{
//  if( p.rows == 3 && p.cols == 1 ) {
//    T_[0] = p(0, 0);
//    T_[1] = p(1, 0);
//    angle_ = p(2, 0);
//    return true;
//  }
//
//  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 3x1", p.rows, p.cols);
//  return false;
//}
//
//cv::Mat1f c_ecc_euclidean_transform::scale(const cv::Mat1f & p, double factor) const
//{
//  if( p.rows == 3 && p.cols == 1 ) {
//
//    cv::Mat1f sp(3, 1);
//
//    sp(0, 0) = p(0, 0) * factor;
//    sp(1, 0) = p(1, 0) * factor;
//    sp(2, 0) = p(2, 0);
//
//    return sp;
//  }
//
//  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 3x1", p.rows, p.cols);
//  return cv::Mat1f();
//}
//
//bool c_ecc_euclidean_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
//{
//  map.create(size);
//
//  const float ca = std::cos(angle_);
//  const float sa = std::sin(angle_);
//  const float a00 = ca, a01 = -sa, a02 = T_[0];
//  const float a10 = sa, a11 = ca, a12 = T_[1];
//
//  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
//      [&map, a00, a01, a02, a10, a11, a12](const tbb_range & r) {
//
//        for ( int y = r.begin(); y < r.end(); ++y ) {
//
//          cv::Vec2f * m = map[y];
//
//          for ( int x = 0; x < map.cols; ++x ) {
//
//            m[x][0] = a00 * x + a01 * y + a02;
//            m[x][1] = a10 * x + a11 * y + a12;
//          }
//        }
//      });
//
//  return true;
//}
//
//bool c_ecc_euclidean_transform::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
//{
//  // SDI:
//  //  [ gx * dWx/dp0 + gy * dWy/dp0]
//  //  [ gx * dWx/dp1 + gy * dWy/dp1]
//  //  ...
//  //  [ gx * dWx/dpn + gy * dWy/dpn]
//
//  // Map:
//  // x' =  cos(angle) * x - sin(angle) * y + Tx
//  // y' =  sin(angle) * x + cos(angle) * y + Ty
//
//  // dWx/dTx = 1
//  // dWx/dTy = 0
//  // dWx/dangle = -sin(angle)*X - cos(angle)*Y
//
//  // dWy/dTx = 0
//  // dWy/dTy = 1
//  // dWy/dangle = cos(angle)*X - sin(angle)*Y
//
//
//  const int w = gx.cols;
//  const int h = gx.rows;
//
//  const float ca = std::cos(angle_);
//  const float sa = std::sin(angle_);
//
//  dst.create(h * 3, w);
//
//  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
//      [ca, sa, w, h, &gx, &gy, &dst](const tbb_range & r) {
//        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
//          for ( int x = 0; x < w; ++x ) {
//            dst[y + 0 * h][x] = gx[y][x];
//            dst[y + 1 * h][x] = gy[y][x];
//            dst[y + 2 * h][x] = -gx[y][x] * (sa * x + ca * y) + gy[y][x] * (ca * x - sa * y);
//          }
//        }
//      });
//
//  return true;
//}
//
//bool c_ecc_euclidean_transform::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & imageSize)
//{
//  const float dx = p(0, 0);
//  const float dy = p(1, 0);
//  const float da = p(2, 0);
//
//  T_[0] += dx;
//  T_[1] += dy;
//  angle_ += da;
//
//  if( e ) {
//    const float sa = std::sin(da);
//    *e = sqrt(square(dx) + square(dy) +
//        square(imageSize.width * sa) +
//        square(imageSize.height * sa));
//  }
//
//  return true;
//}
//
//bool c_ecc_euclidean_transform::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
//{
//  return false;
//}

///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 *  x' =  scale * cos(angle) * x - scale * sin(angle) * y + Tx
 *  y' =  scale * sin(angle) * x + scale * cos(angle) * y + Ty
 *                 =>
 *  x' =  a00 * x + a01 * y + a02
 *  y' =  a10 * x + a11 * y + a12
 **/

//
//  x' =  scale * ( cos(angle) * (x - tx) - sin(angle) * (y - ty))
//  y' =  scale * ( sin(angle) * (x - tx) + cos(angle) * (y - ty))
//
//
//

c_ecc_euclidean_transform::c_ecc_euclidean_transform(float Tx, float Ty, float angle, float scale)
{
  Tx_ = Tx;
  Ty_ = Ty;
  angle_ = angle;
  scale_ = scale;
}

c_ecc_euclidean_transform::c_ecc_euclidean_transform(const cv::Vec2f & T, float angle, float scale)
{
  Tx_ = T[0];
  Ty_ = T[1];
  angle_ = angle;
  scale_ = scale;
}

void c_ecc_euclidean_transform::set_translation(const cv::Vec2f & v)
{
  Tx_ = v[0];
  Ty_ = v[1];
}

cv::Vec2f c_ecc_euclidean_transform::translation() const
{
  return cv::Vec2f(Tx_, Ty_);
}

void c_ecc_euclidean_transform::set_rotation(float v)
{
  angle_ = v;
}

float c_ecc_euclidean_transform::rotation() const
{
  return angle_;
}

void c_ecc_euclidean_transform::set_scale(float v)
{
  scale_ = v;
}

float c_ecc_euclidean_transform::scale() const
{
  return scale_;
}

void c_ecc_euclidean_transform::set_fix_translation(bool v)
{
  fix_translation_ = v;
}

bool c_ecc_euclidean_transform::fix_translation() const
{
  return fix_translation_;
}

void c_ecc_euclidean_transform::set_fix_rotation(bool v)
{
  fix_rotation_ = v;
}

bool c_ecc_euclidean_transform::fix_rotation() const
{
  return fix_rotation_;
}

void c_ecc_euclidean_transform::set_fix_scale(bool v)
{
  fix_scale_ = v;
}

bool c_ecc_euclidean_transform::fix_scale() const
{
  return fix_scale_;
}

cv::Mat1f c_ecc_euclidean_transform::parameters() const
{
  return cv::Mat1f(4, 1, (float*)a).clone();
}

bool c_ecc_euclidean_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 4 && p.cols == 1 ) {
    Tx_ = p(0, 0);
    Ty_ = p(1, 0);
    angle_ = p(2, 0);
    scale_ = p(3, 0);
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 4x1", p.rows, p.cols);
  return false;
}

cv::Mat1f c_ecc_euclidean_transform::scale(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 4 && p.cols == 1 ) {

    cv::Mat1f ss =
        p.clone();

    ss(0, 0) *= factor;
    ss(1, 0) *= factor;

    return ss;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 4x1", p.rows, p.cols);
  return cv::Mat1f();
}


bool c_ecc_euclidean_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

  //  Wx =  scale * ( ca * (x - tx) - sa * (y - ty))
  //  Wy =  scale * ( sa * (x - tx) + ca * (y - ty))

  const float tx = Tx_;
  const float ty = Ty_;
  const float ca = std::cos(angle_);
  const float sa = std::sin(angle_);
  const float scale = scale_;

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [&map, tx, ty, sa, ca, scale](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            m[x][0] = scale * (ca * (x - tx) - sa * (y - ty));
            m[x][1] = scale * (sa * (x - tx) + ca * (y - ty));
          }
        }
      });

  return true;
}

int c_ecc_euclidean_transform:: num_adustable_parameters() const
{
  int n = 0;

  if ( !fix_translation_ ) {
    n += 2;
  }

  if ( !fix_rotation_ ) {
    n += 1;
  }

  if ( !fix_scale_ ) {
    n += 1;
  }

  return n;
}


bool c_ecc_euclidean_transform::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  // STEEPEST DESCENT IMAGES:
  //  [ gx * dWx / dp0 + gy * dWy / dp0]
  //  [ gx * dWx / dp1 + gy * dWy / dp1]
  //  ...
  //  [ gx * dWx / dpn + gy * dWy / dpn]

  // W([x,y], p):
  //  Wx =  scale * ( cos(angle) * (x - tx) - sin(angle) * (y - ty))
  //  Wy =  scale * ( sin(angle) * (x - tx) + cos(angle) * (y - ty))

  //  Wx =  scale * ( ca * (x - tx) - sa * (y - ty))
  //  Wy =  scale * ( sa * (x - tx) + ca * (y - ty))

  // dWx / dtx = scale * ( -ca )
  // dWx / dty = scale * ( +sa )
  // dWx / da = scale * ( -sa * (x - tx) - ca * (y - ty))
  // dWx / ds = (ca * (x - tx) - sa * (y - ty))

  // dWy / dtx = scale * ( -sa )
  // dWy / dty = scale * ( -ca )
  // dWy / da = scale * ( ca * (x - tx) - sa * (y - ty))
  // dWy / ds = (sa * (x - tx) + ca * (y - ty))


  const  int n =
      num_adustable_parameters();

  if ( n < 1 ) {
    CF_ERROR("All parameters are fixed, can not compute steepest_descent_images");
    return false;
  }

  const int w = gx.cols;
  const int h = gx.rows;


  dst.create(h * n, w);

  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [ this, w, h, &gx, &gy, &dst](const tbb_range & r) {

        const float tx = Tx_;
        const float ty = Ty_;
        const float scale = scale_;
        const float sa = std::sin(angle_);
        const float ca = std::cos(angle_);

        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {

          int i = 0;

          const float yy = y - ty;

          if( !fix_translation_ ) {

            for ( int x = 0; x < w; ++x ) {
              const float xx = x - tx;
              dst[y + 0 * h][x] = -scale * ( gx[y][x] * ca - gy[y][x] * sa );
              dst[y + 1 * h][x] = -scale * ( gx[y][x] * sa + gy[y][x] * ca );
            }

            i += 2;
          }

          if( !fix_rotation_ ) {

            for ( int x = 0; x < w; ++x ) {
              const float xx = x - tx;
              dst[y + i * h][x] = scale * ( -gx[y][x] * ( sa * xx + ca * yy) + gy[y][x] * (ca * xx - sa * yy) );
            }

            i += 1;
          }

          if( !fix_scale_ ) {

            for ( int x = 0; x < w; ++x ) {
              const float xx = x - tx;
              dst[y + i * h][x] = gx[y][x] * (ca * xx - sa * yy) + gy[y][x] * (sa * xx + ca * yy);
            }
          }

        }
      });



  //CF_DEBUG("Tx: %+g Ty: %+g angle: %+g scale: %+g", Tx_, Ty_, angle_ * 180 / M_PI, scale_);


  return true;
}

bool c_ecc_euclidean_transform::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  const  int n =
      num_adustable_parameters();

  if( p.rows != n || p.cols != 1 ) {
    CF_ERROR("Invalid size of parameters matrix %dx%d. Must be %dx1", p.rows, p.cols, n);
    return false;
  }

  float dx = 0;
  float dy = 0;
  float da = 0;
  float ds = 0;
  int i = 0;

  if ( !fix_translation_ ) {
    dx = p(i++, 0);
    dy = p(i++, 0);
  }

  if ( !fix_rotation_ ) {
    da = p(i++, 0);
  }

  if ( !fix_scale_ ) {
    ds = p(i++, 0);
  }

  CF_DEBUG("Tx:(%+g %+g) Ty:(%+g %+g) angle:(%+g %+g) scale:(%+g %+g)", Tx_, dx, Ty_, dy, angle_ * 180 / M_PI, da  * 180 / M_PI, scale_, ds);

  Tx_ += dx;
  Ty_ += dy;
  angle_ += da;
  scale_ += ds;

  if( e ) {
    const float sa = sin(da);
    *e = sqrt(square(dx) + square(dy) +
        square(size.width * sa) + square(size.height * sa));
  }

  return true;
}

bool c_ecc_euclidean_transform::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & imageSize)
{
  return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * Image affine transform with scale:
 *  x' =  a00 * x + a01 * y + a02
 *  y' =  a10 * x + a11 * y + a12
 *
 */
c_ecc_affine_transform::c_ecc_affine_transform()
{
}

c_ecc_affine_transform::c_ecc_affine_transform(const float _a[2][3])
{
  memcpy(this->a, _a, sizeof(this->a));
}

c_ecc_affine_transform::c_ecc_affine_transform(const cv::Matx23f & _a)
{
  memcpy(this->a, _a.val, sizeof(this->a));
}

c_ecc_affine_transform::c_ecc_affine_transform(const cv::Mat1f & a)
{
  if( a.rows != 2 || a.cols != 3 || a.channels() != 1 ) {
    CF_ERROR("APP BUG: argument must be 2x3 single channel 32-bit floating point matrix.\n"
        "Actual argument is %dx%d %d channels", a.rows, a.cols, a.channels());
  }
  else {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        this->a[i][j] = a[i][j];
      }
    }
  }
}

c_ecc_affine_transform::c_ecc_affine_transform(float a00, float a01, float a02, float a10, float a11, float a12)
{
  a[0][0] = a00;
  a[0][1] = a01;
  a[0][2] = a02;

  a[1][0] = a10;
  a[1][1] = a11;
  a[1][2] = a12;
}

void c_ecc_affine_transform::set_translation(const cv::Vec2f & v)
{
  a[0][2] = v[0];
  a[1][2] = v[1];
}

cv::Vec2f c_ecc_affine_transform::translation() const
{
  return cv::Vec2f(a[0][2], a[1][2]);
}

cv::Mat1f c_ecc_affine_transform::parameters() const
{
  return cv::Mat1f(2, 3, (float*) a).clone();
}

bool c_ecc_affine_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 2 && p.cols == 3 ) {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        a[i][j] = p[i][j];
      }
    }

    return true;
  }

  if( p.rows == 6 && p.cols == 1 ) {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        a[i][j] = p(i * 3 + j, 0);
      }
    }
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x3", p.rows, p.cols);
  return false;
}

cv::Mat1f c_ecc_affine_transform::scale(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 2 && p.cols == 3 ) {

    cv::Mat1f sp(2, 3);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;
    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x3", p.rows, p.cols);
  return cv::Mat1f();
}


bool c_ecc_affine_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  // Map:
  //  x' =  a00 * x + a01 * y + a02
  // y' =  a11 * x + a11 * y + a12

  map.create(size);

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            m[x][0] = a[0][0] * x + a[0][1] * y + a[0][2];
            m[x][1] = a[1][0] * x + a[1][1] * y + a[1][2];
          }
        }
      });

  return true;
}

int c_ecc_affine_transform::num_adustable_parameters() const
{
  return 6;
}

bool c_ecc_affine_transform::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  // SDI:
  //  [ gx * dWx/dp0 + gy * dWy/dp0]
  //  [ gx * dWx/dp1 + gy * dWy/dp1]
  //  ...
  //  [ gx * dWx/dpn + gy * dWy/dpn]

  // Map:
  // *  x' =  a00 * x + a01 * y + a02
  // *  y' =  a10 * x + a11 * y + a12

  // dWx/da00 = x
  // dWx/da01 = y
  // dWx/da02 = 1
  // dWx/da10 = 0
  // dWx/da11 = 0
  // dWx/da12 = 0

  // dWy/da00 = 0
  // dWy/da01 = 0
  // dWy/da02 = 0
  // dWy/da10 = x
  // dWy/da11 = y
  // dWy/da12 = 1

  const int w = gx.cols;
  const int h = gx.rows;

  dst.create(h * 6, w);

  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < w; ++x ) {

            dst[y + 0 * h][x] = gx[y][x] * x;   // a00
            dst[y + 1 * h][x] = gx[y][x] * y;   // a01
            dst[y + 2 * h][x] = gx[y][x];       // a02

            dst[y + 3 * h][x] = gy[y][x] * x;   // a10
            dst[y + 4 * h][x] = gy[y][x] * y;   // a11
            dst[y + 5 * h][x] = gy[y][x];       // a12
          }
        }
      });

  return true;
}

bool c_ecc_affine_transform::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  a[0][0] += p(0, 0);
  a[0][1] += p(1, 0);
  a[0][2] += p(2, 0);

  a[1][0] += p(3, 0);
  a[1][1] += p(4, 0);
  a[1][2] += p(5, 0);

  if( e ) {
    *e = sqrt(square(p(2, 0)) + square(p(5, 0)) +
        square(size.width * p(0, 0)) + square(size.width * p(3, 0)) +
        square(size.height * p(1, 0)) + square(size.height * p(4, 0)));
  }

  return true;
}

bool c_ecc_affine_transform::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  const float a00 = p(0, 0);
  const float a01 = p(1, 0);
  const float a02 = p(2, 0);

  const float a10 = p(3, 0);
  const float a11 = p(4, 0);
  const float a12 = p(5, 0);

  cv::Matx33f P(
      a[0][0], a[0][1], a[0][2],
      a[1][0], a[1][1], a[1][2],
        0,     0,      1);

  cv::Matx33f dP(
      1 + a00, a01, a02,
      a10, 1 + a11, a12,
      0, 0, 1);

  bool isok = false;

  dP = dP.inv(cv::DECOMP_LU, &isok);
  if ( !isok ) {
    CF_ERROR("dP.inv() fails");
    return false;
  }

  P = P * dP;

  a[0][0] = P(0, 0);
  a[0][1] = P(0, 1);
  a[0][2] = P(0, 2);

  a[1][0] = P(1, 0);
  a[1][1] = P(1, 1);
  a[1][2] = P(1, 2);


  if( e ) {
    *e = sqrt(square(a02) + square(12) +
        square(size.width * a00) + square(size.width * a10) +
        square(size.height * a01) + square(size.height * a11));
  }

  return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////
/**
 * Image homography transform:
 * w  =  (x * a20 + y * a21 + a22)
 * x' =  (x * a00 + y * a01 + a02) / w
 * y' =  (x * a10 + y * a11 + a12) / w
 */

c_ecc_homography_transform::c_ecc_homography_transform()
{
}

c_ecc_homography_transform::c_ecc_homography_transform(const float a[3][3])
{
  memcpy(this->a, a, sizeof(this->a));
}

c_ecc_homography_transform::c_ecc_homography_transform(const cv::Matx33f & a)
{
  memcpy(this->a, a.val, sizeof(this->a));
}

c_ecc_homography_transform::c_ecc_homography_transform(const cv::Mat1f & a)
{
  if( a.rows != 3 || a.cols != 3 || a.channels() != 1 ) {
    CF_ERROR("APP BUG: argument must be 3x3 single channel 32-bit floating point matrix.\n"
        "Actual argument is %dx%d %d channels", a.rows, a.cols, a.channels());
  }
  else {

    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        this->a[i][j] = a[i][j];
      }
    }
  }
}

c_ecc_homography_transform::c_ecc_homography_transform(float a00, float a01, float a02,
    float a10, float a11, float a12,
    float a20, float a21, float a22)
{
  a[0][0] = a00;
  a[0][1] = a01;
  a[0][2] = a02;

  a[1][0] = a10;
  a[1][1] = a11;
  a[1][2] = a12;

  a[2][0] = a20;
  a[2][1] = a21;
  a[2][2] = a22;
}

void c_ecc_homography_transform::set_translation(const cv::Vec2f & v)
{
  a[0][2] = v[0];
  a[1][2] = v[1];
}

cv::Vec2f c_ecc_homography_transform::translation() const
{
  return cv::Vec2f(a[0][2], a[1][2]);
}

cv::Mat1f c_ecc_homography_transform::parameters() const
{
  return cv::Mat1f(3, 3, (float*) a).clone();
}

bool c_ecc_homography_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 3 && p.cols == 3 ) {
    for( int i = 0; i < 3; ++i ) {
      for( int j = 0; j < 3; ++j ) {
        a[i][j] = p[i][j];
      }
    }
    return true;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 3x3", p.rows, p.cols);
  return false;
}

cv::Mat1f c_ecc_homography_transform::scale(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 3 && p.cols == 3 ) {

    cv::Mat1f sp(3, 3);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;

    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;

    sp(2, 0) = p(2, 0) / factor;
    sp(2, 1) = p(2, 1) / factor;
    sp(2, 2) = p(2, 2);

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 3x3", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_ecc_homography_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {

            float w = a[2][0] * x + a[2][1] * y + a[2][2];
            if ( w ) {
              w = 1 / w;
            }
            else {
              w = 1;
            }

            m[x][0] = (a[0][0] * x + a[0][1] * y + a[0][2]) * w;
            m[x][1] = (a[1][0] * x + a[1][1] * y + a[1][2]) * w;
          }
        }
      });

  return true;
}

int c_ecc_homography_transform::num_adustable_parameters() const
{
  return 8;
}


bool c_ecc_homography_transform::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  // w  =  (x * a20 + y * a21 + a22)
  // x' =  (x * a00 + y * a01 + a02) / w
  // y' =  (x * a10 + y * a11 + a12) / w

  const int w = gx.cols;
  const int h = gx.rows;

  dst.create(h * 8, w);

  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [this, w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < w; ++x ) {

            const float den = 1.f / (x * a[2][0] + y * a[2][1] + 1.f );
            const float hatX = -(x * a[0][0] + y * a[0][1] + a[0][2]) * den;
            const float hatY = -(x * a[1][0] + y * a[1][1] + a[1][2]) * den;

            const float ggx = gx[y][x] * den;
            const float ggy = gy[y][x] * den;
            const float gg = hatX * ggx + hatY * ggy;

            dst[y + 0 * h ][x] = ggx * x;
            dst[y + 1 * h ][x] = ggy * x;
            dst[y + 2 * h ][x] = gg * x;
            dst[y + 3 * h ][x] = ggx * y;
            dst[y + 4 * h ][x] = ggy * y;
            dst[y + 5 * h ][x] = gg * y;
            dst[y + 6 * h ][x] = ggx;
            dst[y + 7 * h ][x] = ggy;
          }
        }
      });

  return true;
}

bool c_ecc_homography_transform::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  // w  =  (x * a20 + y * a21 + a22)
  // x' =  (x * a00 + y * a01 + a02) / w
  // y' =  (x * a10 + y * a11 + a12) / w

  a[0][0] += p(0, 0);
  a[0][1] += p(3, 0);
  a[0][2] += p(6, 0);
  a[1][0] += p(1, 0);
  a[1][1] += p(4, 0);
  a[1][2] += p(7, 0);
  a[2][0] += p(2, 0);
  a[2][1] += p(5, 0);

  if( e ) {
    // FIXME?: this estimate does not account for w
    *e = sqrt(square(p(6, 0)) + square(p(7, 0)) +
        square(size.width * p(0, 0)) + square(size.height * p(3, 0)) +
        square(size.width * p(1, 0)) + square(size.height * p(4, 0)));
  }

  return true;
}

bool c_ecc_homography_transform::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  //  x' =  (x * a00 + y * a01 + a02) / w
  //  y' =  (x * a10 + y * a11 + a12) / w
  //  w  =  (x * a20 + y * a21 + a22)

  //  a[0][0] => p(0, 0);
  //  a[0][1] => p(3, 0);
  //  a[0][2] => p(6, 0);
  //  a[1][0] => p(1, 0);
  //  a[1][1] => p(4, 0);
  //  a[1][2] => p(7, 0);
  //  a[2][0] => p(2, 0);
  //  a[2][1] => p(5, 0);
  //  a[2][2] => 1;

  cv::Matx33f P(
      a[0][0], a[0][1], a[0][2],
      a[1][0], a[1][1], a[1][2],
      a[2][0], a[2][1], 1.0);

  cv::Matx33f dP(
      1 + p(0, 0), p(3, 0), p(6, 0),
      p(1, 0), 1 + p(4, 0), p(7, 0),
      p(2, 0), p(5, 0), 1.);

  bool isok = false;

  dP = dP.inv(cv::DECOMP_LU, &isok);
  if( !isok ) {
    CF_ERROR("dP.inv() fails");
    return false;
  }

  P = P * dP;

  a[0][0] = P(0, 0);
  a[0][1] = P(0, 1);
  a[0][2] = P(0, 2);

  a[1][0] = P(1, 0);
  a[1][1] = P(1, 1);
  a[1][2] = P(1, 2);

  a[2][0] = P(2, 0);
  a[2][1] = P(2, 1);
  a[2][2] = 1;

  if( e ) {
    *e = sqrt(square(p(6, 0)) + square(p(7, 0)) +
        square(size.width * p(0, 0)) + square(size.height * p(3, 0)) +
        square(size.width * p(1, 0)) + square(size.height * p(4, 0)));
  }

  return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

c_ecc_quadratic_transform::c_ecc_quadratic_transform()
{
}

c_ecc_quadratic_transform::c_ecc_quadratic_transform(const float a[2][6])
{
  memcpy(this->a, a, sizeof(this->a));
}

c_ecc_quadratic_transform::c_ecc_quadratic_transform(const cv::Matx<float, 2, 6> & a)
{
  memcpy(this->a, a.val, sizeof(this->a));
}

c_ecc_quadratic_transform::c_ecc_quadratic_transform(const cv::Mat1f & a)
{
  if( a.rows != 2 || a.cols != 6 || a.channels() != 1 ) {
    CF_ERROR("APP BUG: argument must be 2x6 single channel 32-bit floating point matrix.\n"
        "Actual argument is %dx%d %d channels", a.rows, a.cols, a.channels());
  }
  else {

    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 6; ++j ) {
        this->a[i][j] = a[i][j];
      }
    }
  }
}

c_ecc_quadratic_transform::c_ecc_quadratic_transform(float a00, float a01, float a02, float a03, float a04, float a05,
    float a10, float a11, float a12, float a13, float a14, float a15)
{
  a[0][0] = a00;
  a[0][1] = a01;
  a[0][2] = a02;
  a[0][3] = a03;
  a[0][4] = a04;
  a[0][5] = a05;

  a[1][0] = a10;
  a[1][1] = a11;
  a[1][2] = a12;
  a[1][3] = a13;
  a[1][4] = a14;
  a[1][5] = a15;
}

cv::Mat1f c_ecc_quadratic_transform::parameters() const
{
  return cv::Mat1f(2, 6, (float*) a).clone();
}

bool c_ecc_quadratic_transform::set_parameters(const cv::Mat1f & p)
{
  if( p.rows == 2 && p.cols == 6 ) {
    for( int i = 0; i < 2; ++i ) {
      for( int j = 0; j < 6; ++j ) {
        a[i][j] = p[i][j];
      }
    }
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x6", p.rows, p.cols);
  return false;
}

cv::Mat1f c_ecc_quadratic_transform::scale(const cv::Mat1f & p, double factor) const
{
  if( p.rows == 3 && p.cols == 3 ) {

    cv::Mat1f sp(3, 3);

    sp(0, 0) = p(0, 0);
    sp(0, 1) = p(0, 1);
    sp(0, 2) = p(0, 2) * factor;
    sp(0, 3) = p(0, 3) / factor;
    sp(0, 4) = p(0, 4) / factor;
    sp(0, 5) = p(0, 5) / factor;

    sp(1, 0) = p(1, 0);
    sp(1, 1) = p(1, 1);
    sp(1, 2) = p(1, 2) * factor;
    sp(1, 3) = p(1, 3) / factor;
    sp(1, 4) = p(1, 4) / factor;
    sp(1, 5) = p(1, 5) / factor;

    return sp;
  }

  CF_ERROR("Invalid size of parameters matrix %dx%d. Must be 2x6", p.rows, p.cols);
  return cv::Mat1f();
}

bool c_ecc_quadratic_transform::create_remap(cv::Mat2f & map, const cv::Size & size) const
{
  map.create(size);

  tbb::parallel_for(tbb_range(0, map.rows, tbb_block_size),
      [this, &map](const tbb_range & r) {

        for ( int y = r.begin(); y < r.end(); ++y ) {

          cv::Vec2f * m = map[y];

          for ( int x = 0; x < map.cols; ++x ) {
            m[x][0] = a[0][0] * x + a[0][1] * y + a[0][2] + a[0][3] * x * y + a[0][4] * x * x + a[0][5] * y * y;
            m[x][1] = a[1][0] * x + a[1][1] * y + a[1][2] + a[1][3] * x * y + a[1][4] * x * x + a[1][5] * y * y;
          }
        }
      });

  return true;
}

int  c_ecc_quadratic_transform::num_adustable_parameters() const
{
  return 12;
}

bool c_ecc_quadratic_transform::create_steepest_descent_images(const cv::Mat1f & gx, const cv::Mat1f & gy, cv::Mat1f & dst) const
{
  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  const int w = gx.cols;
  const int h = gx.rows;

  dst.create(h * 12, w);

  tbb::parallel_for(tbb_range(0, h, tbb_block_size),
      [w, h, &gx, &gy, &dst](const tbb_range & r) {
        for ( int y = r.begin(), ny = r.end(); y < ny; ++y ) {
          for ( int x = 0; x < w; ++x ) {
            dst[y + 0 * h][x] = gx[y][x] * x;
            dst[y + 1 * h][x] = gy[y][x] * x;
            dst[y + 2 * h][x] = gx[y][x] * y;
            dst[y + 3 * h][x] = gy[y][x] * y;
            dst[y + 4 * h][x] = gx[y][x];
            dst[y + 5 * h][x] = gy[y][x];
            dst[y + 6 * h][x] = gx[y][x] * x * y;
            dst[y + 7 * h][x] = gy[y][x] * x * y;
            dst[y + 8 * h][x] = gx[y][x] * x * x;
            dst[y + 9 * h][x] = gy[y][x] * x * x;
            dst[y + 10 * h][x] = gx[y][x] * y * y;
            dst[y + 11 * h][x] = gy[y][x] * y * y;
          }
        }
      });

  return true;
}

bool c_ecc_quadratic_transform::update_forward_additive(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  // x' =  a00 * x + a01 * y + a02 + a03 * x * y + a04 * x * x + a05 * y * y
  // y' =  a10 * x + a11 * y + a12 + a13 * x * y + a14 * x * x + a15 * y * y

  a[0][0] += p(0, 0);
  a[0][1] += p(2, 0);
  a[0][2] += p(4, 0);
  a[0][3] += p(6, 0);
  a[0][4] += p(8, 0);
  a[0][5] += p(10, 0);

  a[1][0] += p(1, 0);
  a[1][1] += p(3, 0);
  a[1][2] += p(5, 0);
  a[1][3] += p(7, 0);
  a[1][4] += p(9, 0);
  a[1][5] += p(11, 0);

  if( e ) {
    *e = sqrt(square(p(4, 0)) + square(p(5, 0)) +
        square(size.width * p(0, 0)) + square(size.height * p(2, 0)) +
        square(size.width * p(1, 0)) + square(size.height * p(3, 0)) +
        square(size.width * size.height * p(6, 0)) + square(size.width * size.height * p(7, 0)) +
        square(size.width * size.width * p(8, 0)) + square(size.width * size.width * p(9, 0)) +
        square(size.height * size.height * p(10, 0)) + square(size.height * size.height * p(11, 0)));
  }

  return true;
}

bool c_ecc_quadratic_transform::update_inverse_composite(const cv::Mat1f & p, float * e, const cv::Size & size)
{
  return false;
}



///////////////////////////////////////////////////////////////////////////////////////////////////


